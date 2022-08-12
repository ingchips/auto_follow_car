#ifndef _PROFILESTASK_H_
#define _PROFILESTASK_H_

#include <stdint.h>

#define PAT_NUMBER  3

typedef struct
{
    int8_t len;
    int8_t def;
    uint8_t ant_ids[75];
} pattern_info_t;

typedef struct
{
    uint8_t slot_duration;
    uint8_t iq_select;
    uint8_t channel;
    pattern_info_t patterns[PAT_NUMBER];
} settings_t;

uint32_t setup_profile(void *data, void *user_data);

#ifdef USE_PI
#define MSG_ID_PI_REQUEST_LOGIN             1
#define MSG_ID_PI_LOGGED_IN                 3
#define MSG_ID_PI_ALG_RESPONSE              4
#define MSG_ID_PI_UNKNOWN                   10
#else
#define MSG_ID_AoA_RESULT                   20
#endif

#define MSG_ID_CTRL_LOST                    100

typedef struct
{
    float azimuth;
    float elevation;
    float distance;
} target_pos_t;

const char *get_sys_info(void);
void get_lastest_pos(target_pos_t *pos);
const char *get_target_name(void);

#endif


