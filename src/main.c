#include <stdio.h>
#include <string.h>
#include "profile.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "kv_storage.h"
#include "FreeRTOS.h"
#include "task.h"
#include "trace.h"

#include "pin_def.h"

#ifdef USE_DISPLAY
#include "oled_ssd1306.c"
#include "FreeRTOS.h"
#include "task.h"
#endif

uint32_t cb_hard_fault(hard_fault_info_t *info, void *_)
{
    platform_printf("HARDFAULT:\nPC : 0x%08X\nLR : 0x%08X\nPSR: 0x%08X\n"
                    "R0 : 0x%08X\nR1 : 0x%08X\nR2 : 0x%08X\nP3 : 0x%08X\n"
                    "R12: 0x%08X\n",
                    info->pc, info->lr, info->psr,
                    info->r0, info->r1, info->r2, info->r3, info->r12);
    for (;;);
}

uint32_t cb_assertion(assertion_info_t *info, void *_)
{
    platform_printf("[ASSERTION] @ %s:%d\n",
                    info->file_name,
                    info->line_no);
    for (;;);
}

#define PRINT_PORT    APB_UART0
#define  COMM_PORT    APB_UART1

void comm_uart_send_byte(uint8_t b)
{
    while (apUART_Check_TXFIFO_FULL(COMM_PORT) == 1) ;
    UART_SendData(COMM_PORT, b);
}

void comm_write_str(const char *s)
{
    while (*s)
    {
        comm_uart_send_byte((uint8_t)*s);
        s++;
    }
    comm_uart_send_byte(10);
}

#ifdef USE_PI
void pi_write_str(const char *s)
{
    comm_write_str(s);
}
#endif

uint32_t cb_putc(char *c, void *dummy)
{
    while (apUART_Check_TXFIFO_FULL(PRINT_PORT) == 1);
    UART_SendData(PRINT_PORT, (uint8_t)*c);
    return 0;
}

int fputc(int ch, FILE *f)
{
    cb_putc((char *)&ch, NULL);
    return ch;
}

void config_uart(uint32_t freq, uint32_t baud)
{
    UART_sStateStruct config;

    config.word_length       = UART_WLEN_8_BITS;
    config.parity            = UART_PARITY_NOT_CHECK;
    config.fifo_enable       = 1;
    config.two_stop_bits     = 0;
    config.receive_en        = 1;
    config.transmit_en       = 1;
    config.UART_en           = 1;
    config.cts_en            = 0;
    config.rts_en            = 0;
    config.rxfifo_waterlevel = 1;
    config.txfifo_waterlevel = 1;
    config.ClockFrequency    = freq;
    config.BaudRate          = baud;

    apUART_Initialize(PRINT_PORT, &config, 0);

    apUART_Initialize(COMM_PORT, &config, (1 << bsUART_RECEIVE_INTENAB));
}

void setup_peripherals(void)
{
    SYSCTRL_ClearClkGateMulti(  (1 << SYSCTRL_ClkGate_APB_GPIO)
                              | (1 << SYSCTRL_ClkGate_APB_PinCtrl)
                              | (1 << SYSCTRL_ClkGate_APB_I2C0)
                              | (1 << SYSCTRL_ClkGate_APB_UART1)
                              | (1 << SYSCTRL_ClkGate_APB_PWM)    );
    config_uart(OSC_CLK_FREQ, 115200);

    static const uint8_t ant_pins[] = {7, 8, 10, 11};
    PINCTRL_EnableAntSelPins(sizeof(ant_pins) / sizeof(ant_pins[0]), ant_pins);

    PINCTRL_SetPadMux(PIN_COMM_UART_TX, IO_SOURCE_UART1_TXD);
    PINCTRL_SetPadMux(PIN_COMM_UART_RX, IO_SOURCE_GENERAL);
    PINCTRL_SelUartRxdIn(UART_PORT_1, PIN_COMM_UART_RX);
}

uint32_t on_deep_sleep_wakeup(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    setup_peripherals();
    return 0;
}

uint32_t query_deep_sleep_allowed(void *dummy, void *user_data)
{
    (void)(dummy);
    (void)(user_data);
    // TODO: return 0 if deep sleep is not allowed now; else deep sleep is allowed
    return 0;
}

void set_sample_offset(int n)
{
    volatile uint32_t *reg = (volatile uint32_t *)0x40090200;
    *reg = (*reg & ~(0x1f << 15)) | ((n & 0x1f) << 15);
}

uint32_t cb_lle_init(char *c, void *dummy)
{
    extern void setup_ll_param(void);
    setup_ll_param();
    return 0;
}

extern void rx_uart_byte(const uint8_t b);

uint32_t uart_isr(void *user_data)
{
    uint32_t status;

    while(1)
    {
        status = apUART_Get_all_raw_int_stat(COMM_PORT);
        if (status == 0)
            break;

        COMM_PORT->IntClear = status;

        // rx int
        if (status & (1 << bsUART_RECEIVE_INTENAB))
        {
            while (apUART_Check_RXFIFO_EMPTY(COMM_PORT) != 1)
            {
                uint8_t b = COMM_PORT->DataRead;
                rx_uart_byte(b);
            }
        }
    }
    return 0;
}

#ifdef USE_DISPLAY

static void display_task(void *pdata)
{
    static char temp[100];
    target_pos_t pos;

    OLED_Init();

    OLED_Clear();

    for (;;)
    {
        get_lastest_pos(&pos);
        OLED_ShowString(0, 0, get_sys_info(), 1, 16);

        sprintf(temp, "AZI : %5.1f", pos.azimuth);
        OLED_ShowString(0, 2, temp, 1, 16);

        sprintf(temp, "DIST: %5.1f", pos.distance);
        //sprintf(temp, "ELE : %5.1f", pos.elevation);
        OLED_ShowString(0, 4, temp, 1, 16);

        const char *name = get_target_name();
        if (name[0])
            OLED_ShowString(0, 6, get_target_name(), 1, 10);
        else
            OLED_ShowString(0, 6, "---------------", 1, 10);

        vTaskDelay(300);
    }
}

static void display_init(void)
{
    xTaskCreate(display_task,
           "w",
           200,
           NULL,
           0,
           NULL);
}

#endif

trace_rtt_t trace_ctx = {0};

int app_main()
{
    platform_set_evt_callback(PLATFORM_CB_EVT_PROFILE_INIT, setup_profile, NULL);

    // setup handlers
    platform_set_evt_callback(PLATFORM_CB_EVT_HARD_FAULT, (f_platform_evt_cb)cb_hard_fault, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_ASSERTION, (f_platform_evt_cb)cb_assertion, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_ON_DEEP_SLEEP_WAKEUP, on_deep_sleep_wakeup, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_QUERY_DEEP_SLEEP_ALLOWED, query_deep_sleep_allowed, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_PUTC, (f_platform_evt_cb)cb_putc, NULL);
    platform_set_evt_callback(PLATFORM_CB_EVT_LLE_INIT, (f_platform_evt_cb)cb_lle_init, NULL);

#ifdef USE_PI
    platform_config(PLATFORM_CFG_LL_DBG_FLAGS, LL_FLAG_DISABLE_CTE_PREPROCESSING);
#endif

    setup_peripherals();

    platform_set_irq_callback(PLATFORM_CB_IRQ_UART1, (f_platform_irq_cb)uart_isr, NULL);

    trace_rtt_init(&trace_ctx);
    platform_set_evt_callback(PLATFORM_CB_EVT_TRACE, (f_platform_evt_cb)cb_trace_rtt, &trace_ctx);
    platform_config(PLATFORM_CFG_TRACE_MASK, 0xff);

#ifdef USE_DISPLAY
    display_init();
#endif

    return 0;
}
