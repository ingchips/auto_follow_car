#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "platform_api.h"
#include "att_db.h"
#include "gap.h"
#include "att_dispatch.h"
#include "btstack_util.h"
#include "btstack_event.h"
#include "btstack_defines.h"
#include "gatt_client.h"
#include "kv_storage.h"
#include "ll_api.h"
#include "ad_parser.h"
#include "sig_uuid.h"
#include "gatt_client_util.h"
#include "kalman.h"

#include "ant_id_mapping_4x4.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "service_console.h"

#include "profile.h"
#include "vehicle.h"

// GATT characteristic handles
#include "../data/gatt.const"

#include "ota_service.h"

#define iprintf(...)

#ifndef REF_RSSI
#define REF_RSSI -48
#endif

#define SPEED_OF_LIGHT      (299792458.0f)
#ifndef M_PI
#define M_PI                (3.1415926535897932384626433832795f)
#endif

#define RSSI_LOCK   (REF_RSSI + 5)
#define RSSI_UNLOCK (REF_RSSI - 6)

#define ANTENNA_ARRAY_4x4   0
#define ANTENNA_ARRAY_1x4   1
#define ANTENNA_ARRAY_1x3   2
#define ANTENNA_ARRAY_2x2   3
#define ANTENNA_ARRAY_3x3   4
#define ANTENNA_ARRAY_1100  5
#define ANTENNA_ARRAY_1010  6
#define ANTENNA_ARRAY_1001  7

#ifndef USE_PI
#define CURRENT_ARRAY       ANTENNA_ARRAY_1010
#endif

#ifndef CURRENT_ARRAY
#define CURRENT_ARRAY       ANTENNA_ARRAY_1x4
#endif

#if ((CURRENT_ARRAY == ANTENNA_ARRAY_4x4) || (CURRENT_ARRAY == ANTENNA_ARRAY_3x3) || (CURRENT_ARRAY == ANTENNA_ARRAY_2x2))
#define TWO_DIMENSION
#endif

#define INVALID_HANDLE 0xffff

const static uint8_t adv_data[] = {
    #include "../data/advertising.adv"
};

const static uint8_t scan_data[] = {
    #include "../data/scan_response.adv"
};

const static uint8_t profile_data[] = {
    #include "../data/gatt.profile"
};

hci_con_handle_t handle_send = INVALID_HANDLE;
static uint8_t notify_enable = 0;
struct gatt_client_discoverer *discoverer = NULL;

extern void set_sample_offset(int n);

int current_pattern = -1;
hci_con_handle_t conn_handle = INVALID_HANDLE;

static settings_t _settings = {0};

settings_t *settings = &_settings;

uint8_t scanner_configured = 0;

static TimerHandle_t ctrl_lost_timer = 0;

#ifdef USE_PI
static TimerHandle_t pi_login_timer = 0;

const char state_wait_pi[] = "PI Booting...";
const char state_logging_in[] = "LOG IN ...";
const char state_pi_ready[] = "PI READY";

const char *current_state = state_wait_pi;

enum
{
    PI_UNKNOWN,
    PI_NAME_SENT,
    PI_PWD_SENT,
    PI_ALG_STARTED,
    PI_DATA_SENT,
} pi_state = PI_UNKNOWN;

int pi_uart_mode = 0;
int pi_rx_cnt = 0;
char pi_rx_str[500];
char pi_copy_str[500];
int pi_ready = 0;
int copy_in_use = 0;

static int wait_for_result = 0;

#else
const char state_ready[] = "READY";
const char *current_state = state_ready;
#endif

static target_pos_t target_pos = {0};

static char target_name[30] = {0};
static bd_addr_t target_addr = {0};

static kalman_filter_t dist_filter = {0};
static kalman_filter_t azi_filter = {0};

void rx_uart_byte(const uint8_t b)
{
    //platform_printf("%c", b);
#ifdef USE_PI
    if (pi_rx_cnt >= sizeof(pi_rx_str) - 1) pi_rx_cnt--;
    pi_rx_str[pi_rx_cnt++] = (char)b;

    if (pi_uart_mode == 0)
    {
        pi_rx_str[pi_rx_cnt] = '\0';
        if (strcmp(pi_rx_str, "raspberrypi login: ") == 0)
            btstack_push_user_msg(MSG_ID_PI_REQUEST_LOGIN, NULL, 0);
        else if (strcmp(pi_rx_str, "pi@raspberrypi:~$ ") == 0)
            btstack_push_user_msg(MSG_ID_PI_LOGGED_IN, NULL, 0);
        else if (strstr(pi_rx_str, "{\"status\":") != NULL)
        {
            int offset = strstr(pi_rx_str, "{\"status\":") - pi_rx_str;
            if (offset > 0)
            {
                memmove(pi_rx_str, pi_rx_str + offset, pi_rx_cnt - offset);
                pi_rx_cnt -= offset;
            }
            pi_uart_mode = 1;
        }
    }

    if ((b != '\r') && (b != '\n')) return;

    if (pi_uart_mode == 0)
    {
        pi_rx_cnt = 0;
        return;
    }

    pi_rx_str[--pi_rx_cnt] = '\0';

    if (strstr(pi_rx_str, "{\"status\":") == pi_rx_str)
    {
        if (copy_in_use == 0)
        {
            copy_in_use = 1;
            strcpy(pi_copy_str, pi_rx_str);
            btstack_push_user_msg(MSG_ID_PI_ALG_RESPONSE, pi_copy_str, 0);
        }
    }
    else
    {
        if (pi_rx_cnt > 0)
        {
            btstack_push_user_msg(MSG_ID_PI_UNKNOWN, NULL, 0);
        }
    }
    pi_rx_cnt = 0;

#endif
}

#ifdef USE_PI

void rx_uart_release_str(char *s)
{
    copy_in_use = 0;
}

extern void pi_write_str(const char *s);
#endif

char *base64_encode(const uint8_t *data, int data_len,
                    char *res, int buffer_size);

static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
                                  uint8_t * buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {

    default:
        return ota_read_callback(att_handle, offset, buffer, buffer_size);
    }
}

static btstack_packet_callback_registration_t hci_event_callback_registration;

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
                              uint16_t offset, const uint8_t *buffer, uint16_t buffer_size)
{
    switch (att_handle)
    {
    case HANDLE_GENERIC_OUTPUT + 1:
        if(*(uint16_t *)buffer == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION)
            notify_enable = 1;
        else
            notify_enable = 0;
        return 0;
    case HANDLE_GENERIC_INPUT:
        console_rx_data((const char *)buffer, buffer_size);
        return 0;
    default:
        return ota_write_callback(att_handle, transaction_mode, offset, buffer, buffer_size);
    }
}

#ifdef TWO_DIMENSION
void update_distance(float evelation)
{
    float v;
    if (evelation < 10) v = 6;
    else if (evelation > 80) v = 0.1;
    else v = 1.0 / tanf(evelation / 180.0 * M_PI);
    target_pos.distance = kalman_update(&dist_filter, v);
}
#else

float estimate_distance(int ref_power, int rssi)
{
    return powf(10, (ref_power - rssi) / 20.0f);
}

void update_distance(int rssi)
{
    float v = estimate_distance(REF_RSSI, rssi);
    target_pos.distance = kalman_update(&dist_filter, v);
}
#endif

void safty_manage(const le_meta_pro_connless_iq_report_t *report)
{
    int rssi = report->rssi / 10;
    static uint8_t locked = 0;
    if (locked)
    {
        if (rssi < RSSI_UNLOCK)
        {
            vehicle_unlock();
            locked = 0;
        }
    }
    else
    {
        if (rssi > RSSI_LOCK)
        {
            vehicle_lock();
            locked = 1;
        }
    }
}

void recv_pro_connless_iq_report(const le_meta_pro_connless_iq_report_t *report)
{
    static char iq_str_buffer[500];
    int len;
#ifdef USE_PI
    int encode_en = 1;
#else
    int encode_en = 0;
#endif

    xTimerReset(ctrl_lost_timer, portMAX_DELAY);

    safty_manage(report);

    if (target_name[0] == '\0')
    {
        memcpy(target_addr, report->addr, BD_ADDR_LEN);
        sprintf(target_name, "%02X%02X%02X:%02X%02X%02X",
                target_addr[5], target_addr[4], target_addr[3],
                target_addr[2], target_addr[1], target_addr[0]);
    }
    else if (memcmp(report->addr, target_addr, BD_ADDR_LEN))
        return;

#ifdef USE_PI
    if (wait_for_result) return;
#else
    encode_en = notify_enable;
#endif

#ifndef TWO_DIMENSION
    update_distance(report->rssi / 10);
#endif

    if (encode_en)
    {
        sprintf(iq_str_buffer, "EXT%d:", current_pattern);
        len = strlen(iq_str_buffer);
        base64_encode((const uint8_t *)report,
            sizeof(*report) + report->sample_count * sizeof(report->samples[0]),
            iq_str_buffer + len, sizeof(iq_str_buffer) - len - 1);
    }

    if (notify_enable)
        att_server_notify(handle_send, HANDLE_GENERIC_OUTPUT, (uint8_t *)iq_str_buffer, strlen(iq_str_buffer) + 1);
    else
    {
#ifdef USE_PI
        pi_write_str(iq_str_buffer);
        wait_for_result = 1;
#else
        float two_ant_simple_aoa(const le_meta_pro_connless_iq_report_t *report);
        int16_t theta = (int16_t)two_ant_simple_aoa(report);
        btstack_push_user_msg(MSG_ID_AoA_RESULT, NULL, (uint16_t)theta);
#endif
    }
}

void stack_notify_tx_data()
{
    if (notify_enable)
    {
        uint16_t len;
        uint8_t *data = console_get_clear_tx_data(&len);
        att_server_notify(handle_send, HANDLE_GENERIC_OUTPUT, data, len);
    }
}

int parse_int_value(const char *str, const char *name, int *value)
{
    int len = strlen(name);
    const char *res = strstr(str, name);
    if (NULL == res) return -1;
    res += len;
    *value = atoi(res);
    return 0;
}

int parse_result(const char *str, int *azimuth, int *elevation)
{
    int r = parse_int_value(str, "\"azimuth\": ", azimuth);
    if (r != 0) return r;
    if (elevation) r = parse_int_value(str, "\"elevation\": ", elevation);
    return r;
}

const char *get_sys_info(void)
{
    return current_state;
}

void get_lastest_pos(target_pos_t *pos)
{
    *pos = target_pos;
}

const char *get_target_name(void)
{
    return target_name;
}

static void user_msg_handler(uint32_t msg_id, void *data, uint16_t size)
{
    int azimuth, elevation;
    // platform_printf("-=> %d\n", msg_id);
    switch (msg_id)
    {
#ifdef USE_PI
    case MSG_ID_PI_REQUEST_LOGIN:
        current_state = state_logging_in;
        xTimerStop(pi_login_timer, portMAX_DELAY);
        pi_write_str("pi");
        vTaskDelay(1000);
        pi_write_str("raspberry");
        break;
    case MSG_ID_PI_LOGGED_IN:
#if (CURRENT_ARRAY == ANTENNA_ARRAY_4x4)
        pi_write_str("~/rtl/alg -array 4x4");
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_3x3)
        pi_write_str("~/rtl/alg -array 3x3");
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_2x2)
        pi_write_str("~/rtl/alg -array 2x2");
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1x4)
        pi_write_str("~/rtl/alg -array 1x4");
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1x3)
        pi_write_str("~/rtl/alg -array 1x3");
#else
        pi_write_str("~/rtl/alg -array 1x2");
#endif
        vTaskDelay(100);
        pi_write_str("");
        break;
    case MSG_ID_PI_ALG_RESPONSE:
        xTimerStop(pi_login_timer, portMAX_DELAY);
        pi_ready = 1;
        current_state = state_pi_ready;

#ifdef TWO_DIMENSION
        if (0 == parse_result((char *)data, &azimuth, &elevation))
        {
            target_pos.azimuth = azimuth - 90;
            target_pos.elevation = elevation;
            update_distance(elevation);
            //platform_printf("azimuth: %d\n", target_pos.azimuth);
            vehicle_update(&target_pos);
        }
#else
        if (0 == parse_result((char *)data, &azimuth, NULL))
        {
            target_pos.azimuth = azimuth - 90;
            //platform_printf("azimuth: %d\n", target_pos.azimuth);
            vehicle_update(&target_pos);
        }
#endif

        wait_for_result = 0;

        rx_uart_release_str((char *)data);

        //pi_write_str(test_data);
        break;
    case MSG_ID_PI_UNKNOWN:
        //platform_printf("MSG_ID_PI_UNKNOWN\n");
        break;
#else
    case MSG_ID_AoA_RESULT:
        target_pos.azimuth = (int16_t)size;
        vehicle_update(&target_pos);
        break;
#endif
    case MSG_ID_CTRL_LOST:
        target_name[0] = '\0';
        memset(&target_pos, 0, sizeof(target_pos));
        vehicle_reset();
        break;
    default:
        break;
    }
}

static initiating_phy_config_t phy_configs[] =
{
    {
        .phy = PHY_1M,
        .conn_param =
        {
            .scan_int = 200,
            .scan_win = 150,
            .interval_min = 30,
            .interval_max = 30,
            .latency = 0,
            .supervision_timeout = 200,
            .min_ce_len = 20,
            .max_ce_len = 20
        }
    }
};

void config_switching_pattern(void)
{
    current_pattern++;
    if (current_pattern >= PAT_NUMBER)
        current_pattern = 0;
    if (settings->patterns[current_pattern].len <= 0)
        current_pattern = 0;

    if (settings->patterns[current_pattern].len <= 2)
        settings->patterns[current_pattern].len = 2;

    const uint8_t *mapped = switch_pattern_mapping(settings->patterns[current_pattern].len, settings->patterns[current_pattern].ant_ids);

    ll_scanner_enable_iq_sampling(CTE_AOA,
                                  settings->slot_duration,
                                  settings->patterns[current_pattern].len,
                                  mapped,
                                  12, 1);
}

void set_channel(void)
{
    uint8_t channel = settings->channel;
    uint32_t low = channel <= 31 ? 1ul << channel : 0;
    uint8_t high = channel >= 32 ? 1 << (channel - 32) : 0;
    gap_set_host_channel_classification(low, high);
}

void set_responder_led(int r, int g, int b)
{
    #define HANDLE_RGB_LIGHTING_CONTROL                          6
    uint8_t v[3] = { (uint8_t)r, (uint8_t)g, (uint8_t)b };
    if (conn_handle != INVALID_HANDLE)
        gatt_client_write_value_of_characteristic_without_response(conn_handle,
                                                                   HANDLE_RGB_LIGHTING_CONTROL,
                                                                   sizeof(v),
                                                                   v);
}

static void setup_adv()
{
    gap_set_ext_adv_para(0,
                            CONNECTABLE_ADV_BIT | SCANNABLE_ADV_BIT | LEGACY_PDU_BIT,
                            800, 800,                  // Primary_Advertising_Interval_Min, Primary_Advertising_Interval_Max
                            PRIMARY_ADV_ALL_CHANNELS,  // Primary_Advertising_Channel_Map
                            BD_ADDR_TYPE_LE_RANDOM,    // Own_Address_Type
                            BD_ADDR_TYPE_LE_PUBLIC,    // Peer_Address_Type (ignore)
                            NULL,                      // Peer_Address      (ignore)
                            ADV_FILTER_ALLOW_ALL,      // Advertising_Filter_Policy
                            0x00,                      // Advertising_Tx_Power
                            PHY_1M,                    // Primary_Advertising_PHY
                            0,                         // Secondary_Advertising_Max_Skip
                            PHY_1M,                    // Secondary_Advertising_PHY
                            0x00,                      // Advertising_SID
                            0x00);                     // Scan_Request_Notification_Enable
    gap_set_ext_adv_data(0, sizeof(adv_data), (uint8_t*)adv_data);
    gap_set_ext_scan_response_data(0, sizeof(scan_data), (uint8_t*)scan_data);
}

void setup_ll_param(void)
{
    if (settings)
    {
        set_sample_offset(settings->iq_select);
    }
}

static const scan_phy_config_t configs[] =
{
    {
        .phy = PHY_1M,
        .type = SCAN_PASSIVE,
        .interval = 200,
        .window = 90
    },
};

static bd_addr_t peer_addr = { 0 };
static bd_addr_type_t peer_addr_type = BD_ADDR_TYPE_LE_PUBLIC;

static void write_characteristic_value_callback(uint8_t packet_type, uint16_t _, const uint8_t *packet, uint16_t size)
{
    switch (packet[0])
    {
    case GATT_EVENT_QUERY_COMPLETE:
        iprintf("peer cte enabled\n");
        config_switching_pattern();
        set_channel();
        break;
    }
}

static void enable_peer_cte(service_node_t *first, void *user_data, int err_code)
{
    if (err_code) platform_reset();
    char_node_t *ch = gatt_client_util_find_char_uuid16(discoverer, SIG_UUID_CHARACT_CONSTANT_TONE_EXTENSION_ENABLE);
    if (ch == NULL) platform_reset();
    uint8_t enable = 1;
    gatt_client_write_value_of_characteristic(write_characteristic_value_callback, conn_handle, ch->chara.value_handle, sizeof(enable), &enable);
    gatt_client_util_free(discoverer);
    discoverer = NULL;
}

static void user_packet_handler(uint8_t packet_type, uint16_t channel, const uint8_t *packet, uint16_t size)
{
    const static ext_adv_set_en_t adv_sets_en[1] = {{.handle = 0, .duration = 0, .max_events = 0}};
    static const bd_addr_t rand_addr = { 0xD4, 0x29, 0xF6, 0xBE, 0xF3, 0x26 };
    uint8_t event = hci_event_packet_get_type(packet);
    const btstack_user_msg_t *p_user_msg;
    const event_disconn_complete_t *disconn_event;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (event)
    {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
            break;
        gap_set_random_device_address(rand_addr);

        gap_set_adv_set_random_addr(0, rand_addr);
        setup_adv();
        gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);

        gap_set_ext_scan_para(BD_ADDR_TYPE_LE_RANDOM, SCAN_ACCEPT_ALL_EXCEPT_NOT_DIRECTED,
                              sizeof(configs) / sizeof(configs[0]),
                              configs);
        gap_set_ext_scan_enable(1, 0, 0, 0);
        break;

    case HCI_EVENT_LE_META:
        switch (hci_event_le_meta_get_subevent_code(packet))
        {
        case HCI_SUBEVENT_LE_ENHANCED_CONNECTION_COMPLETE:
            {
                const le_meta_event_enh_create_conn_complete_t *conn_complete
                    = decode_hci_le_meta_event(packet, le_meta_event_enh_create_conn_complete_t);
                iprintf("connected,%d,%d\n", conn_complete->status,conn_complete->role);
                if (conn_complete->status != 0)
                {
                    gap_ext_create_connection(    INITIATING_ADVERTISER_FROM_PARAM, // Initiator_Filter_Policy,
                                          BD_ADDR_TYPE_LE_RANDOM,           // Own_Address_Type,
                                          peer_addr_type,                   // Peer_Address_Type,
                                          peer_addr,                        // Peer_Address,
                                          sizeof(phy_configs) / sizeof(phy_configs[0]),
                                          phy_configs);
                    break;
                }

                if (HCI_ROLE_SLAVE == conn_complete->role)
                {
                    handle_send = conn_complete->handle;
                    att_set_db(handle_send, profile_data);
                }
                else
                {
                    conn_handle = conn_complete->handle;
                    gatt_client_is_ready(conn_handle);
                    current_pattern = -1;
                    discoverer = gatt_client_util_discover_all(conn_handle, enable_peer_cte, NULL);
                }
            }
            break;
        case HCI_SUBEVENT_LE_EXTENDED_ADVERTISING_REPORT:
            if (0 == scanner_configured)
            {
                scanner_configured = 1;
                config_switching_pattern();
            }
            break;
        case HCI_SUBEVENT_LE_VENDOR_PRO_CONNECTIONLESS_IQ_REPORT:
            recv_pro_connless_iq_report(decode_hci_le_meta_event(packet, le_meta_pro_connless_iq_report_t));
            break;
        default:
            break;
        }

        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        disconn_event = decode_hci_event_disconn_complete(packet);
        iprintf("disc:%d,%d,%d\n",disconn_event->conn_handle,handle_send,conn_handle);
        if (disconn_event->conn_handle == handle_send)
        {
            handle_send = INVALID_HANDLE;
            notify_enable = 0;
            gap_set_ext_adv_enable(1, sizeof(adv_sets_en) / sizeof(adv_sets_en[0]), adv_sets_en);
        }
        else
        {
            conn_handle = INVALID_HANDLE;
            gap_ext_create_connection(    INITIATING_ADVERTISER_FROM_PARAM, // Initiator_Filter_Policy,
                                          BD_ADDR_TYPE_LE_RANDOM,           // Own_Address_Type,
                                          peer_addr_type,                   // Peer_Address_Type,
                                          peer_addr,                        // Peer_Address,
                                          sizeof(phy_configs) / sizeof(phy_configs[0]),
                                          phy_configs);
        }
        break;

    case L2CAP_EVENT_CAN_SEND_NOW:
        // add your code
        break;

    case BTSTACK_EVENT_USER_MSG:
        p_user_msg = hci_event_packet_get_user_msg(packet);
        user_msg_handler(p_user_msg->msg_id, p_user_msg->data, p_user_msg->len);
        break;

    default:
        break;
    }
}

const static uint8_t pattern[] =
#if (CURRENT_ARRAY == ANTENNA_ARRAY_4x4)
{ 12, 8, 4, 0, 13, 9, 5, 1, 14, 10, 6, 2, 15, 11, 7, 3}
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_3x3)
{ 12, 8, 4, 13, 9, 5, 14, 10, 6 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_2x2)
{ 8, 4, 9, 5 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1x4)
{ 12, 8, 4, 0 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1x3)
{ 12, 8, 4 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1100)
{ 8, 4 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1010)
{ 12, 4 }
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1001)
{ 12, 0 }
#else
#error unknown array: CURRENT_ARRAY
#endif
;

#ifdef USE_PI
static void pi_login_callback(TimerHandle_t xTimer)
{
    platform_printf("write...");
    pi_write_str("");
    platform_printf("\n");
}
#endif

static void ctrl_lost_callback(TimerHandle_t xTimer)
{
    platform_printf("lost...");
    btstack_push_user_msg(MSG_ID_CTRL_LOST, NULL, 0);
}

uint32_t setup_profile(void *data, void *user_data)
{
    iprintf("setup profile\n");

    vehicle_init();

    kalman_init(&dist_filter, 0.22, 0.92);
    kalman_init(&azi_filter, 0.22, 0.62);

    memset(settings, 0, sizeof(settings_t));
    settings->slot_duration = 1;
    settings->patterns[0].len = sizeof(pattern);
    memcpy(settings->patterns[0].ant_ids, pattern, sizeof(pattern));

#ifdef USE_PI
    pi_login_timer = xTimerCreate("app",
                            pdMS_TO_TICKS(1000),
                            pdTRUE,
                            NULL,
                            pi_login_callback);
    xTimerStart(pi_login_timer, portMAX_DELAY);
#endif

    ctrl_lost_timer = xTimerCreate("lost",
                            pdMS_TO_TICKS(1000),
                            pdFALSE,
                            NULL,
                            ctrl_lost_callback);

    setup_ll_param();
    att_server_init(att_read_callback, att_write_callback);
    hci_event_callback_registration.callback = user_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(user_packet_handler);
    gatt_client_register_handler(user_packet_handler);
    return 0;
}


prog_ver_t prog_ver = {
    .major = 5,
    .minor = 1,
    .patch = 0,
};

float freqOfChannelIndexMHz(int channelIndex) {
    if (0 > channelIndex)
        return 0.0f;

    if (channelIndex <= 10)
        return 2404.0f + 2 * channelIndex;
    else if (channelIndex <= 36)
        return 2428.0f + 2 * (channelIndex - 11);
    else
        return 0.0f;
}

#if (CURRENT_ARRAY == ANTENNA_ARRAY_1100)
#define ANT_DISTANCE        (0.04f * 1)
#elif (CURRENT_ARRAY == ANTENNA_ARRAY_1010)
#define ANT_DISTANCE        (0.04f * 2)
#else
#define ANT_DISTANCE        (0.04f * 3)
#endif

float two_ant_simple_aoa(const le_meta_pro_connless_iq_report_t *report)
{
    struct
    {
        int i;
        int q;
    } sample = {0};

    int i;
    int c = 0;
    for (i = 8; i < report->sample_count; i += 2, c++)
    {
        sample.i += report->samples[i].i;
        sample.q += report->samples[i].q;
    }
    sample.i /= c;
    sample.q /= c;

    float freq = freqOfChannelIndexMHz(report->channel_index) * 1000000.0f;
    float phase_diff = atan2f(sample.q, sample.i);
    float lambda = SPEED_OF_LIGHT / freq;
    float v = lambda * phase_diff / (2 * M_PI * ANT_DISTANCE);
    float theta;
    if (fabs(v) <= 1.0)
        theta = acosf(v) - M_PI / 2;
    else if (v > 0)
        theta = M_PI / 2;
    else
        theta = -M_PI / 2;
    return theta * (180 / M_PI);
}
