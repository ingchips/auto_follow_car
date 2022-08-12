#include "vehicle.h"
#include "ingsoc.h"
#include "peripheral_pwm.h"
#include "peripheral_pinctrl.h"
#include "platform_api.h"

#include "FreeRTOS.h"

#include "pin_def.h"

#define PWM_CH_STEERING         5
#define PWM_CH_MOTOR_LEFT       0
#define PWM_CH_MOTOR_RIGHT      1

#define STEER_SERVO_PWM_FREQ        50

#define MOTOR_PWM_FREQ              1000

#define UPDATE_PERIOD               (300000)

#define STEERING_MAX_OFFSET         45

typedef struct
{
    float factor;
    float value;
} iir_1st_order_t;

typedef struct
{
    int value;
    int error;
    int error_sum;
    float kp, ki, kd;
} pid_state_t;

typedef struct
{
    iir_1st_order_t filter_steering;
    iir_1st_order_t filter_distance;
    uint64_t last_update;
    target_pos_t acc;
    int count;
    uint8_t locked;
    pid_state_t pid_steering;
    pid_state_t pid_speed;
} vehicle_state_t;

vehicle_state_t vehicle_state = {
    .filter_steering ={ .factor = 0.7, .value = 0 },
    .filter_distance ={ .factor = 0.95, .value = 0 },
    .pid_steering = {.kd = 0.3, .ki = 0, .kp = 0.1},
};

// first order IIR to filter distance
float iir_update(iir_1st_order_t *iir, float value)
{
    iir->value = iir->value * iir->factor + value * (1 - iir->factor);
    return iir->value;
}

void pid_reset(pid_state_t *pid)
{
    pid->error_sum = 0;
    pid->error = 0;
}

int pid_update(pid_state_t *pid, int value)
{
    int err = value - pid->value;
    pid->error_sum += err;
    int d = err - pid->error;
    pid->error = err;
    return (int)(pid->kp * err + pid->ki * pid->error_sum + pid->kd * d);
}


// Servo MG996R PWM 50Hz (period 20ms)
// duty (ms)         angle (degree)
// 0.5                  0   (2.5%)
// 1.0                  45  (5%)
// 1.5                  90  (7.5%)
// 2.5                  180 (12.5%)

// deg: [-45, 45] (+/- STEERING_MAX_OFFSET)
// deg < 0: turn left
// deg > 0: turn right
void vehicle_set_steering(int deg)
{
    #define DEG_MIN     10
    #define DEG_MAX     (180 - DEG_MIN)

    if (deg < -STEERING_MAX_OFFSET)
        deg = -STEERING_MAX_OFFSET;
    else if (deg > STEERING_MAX_OFFSET)
        deg = STEERING_MAX_OFFSET;

    vehicle_state.pid_steering.value = deg;

    deg *= 2;
    deg += 90;
    if (deg < DEG_MIN) deg = DEG_MIN;
    else if (deg > DEG_MAX) deg = DEG_MAX;

    #define PERA (PWM_CLOCK_FREQ / STEER_SERVO_PWM_FREQ)
    uint32_t low = deg * PERA / 180 / 10 + (int)(2.5 * PERA / 100);
    PWM_SetHighThreshold(PWM_CH_STEERING, 0, PERA - low);
}

void steering_pid_process(int azimuth)
{
    pid_state_t *pid = &vehicle_state.pid_steering;
    int u = pid_update(pid, azimuth);
    vehicle_set_steering(pid->value + u);
}

// speed: 0~100
void vehicle_set_motor_speed(int speed)
{
    #define MOTOR_PERA (PWM_CLOCK_FREQ / MOTOR_PWM_FREQ)
    uint32_t high = (100 - speed) * MOTOR_PERA / 100;
    PWM_SetHighThreshold(PWM_CH_MOTOR_LEFT, 0, high);
    PWM_SetHighThreshold(PWM_CH_MOTOR_RIGHT, 0, high);
}

void vehicle_init(void)
{
    PINCTRL_SetGeneralPadMode(PIN_STEERING, IO_MODE_PWM, PWM_CH_STEERING, 0);
    PWM_SetupSimple(PWM_CH_STEERING, STEER_SERVO_PWM_FREQ, 8);

    PINCTRL_SetGeneralPadMode(PIN_LEFT_WHEEL, IO_MODE_PWM, PWM_CH_MOTOR_LEFT, 0);
    PWM_SetupSimple(PWM_CH_MOTOR_LEFT, MOTOR_PWM_FREQ, 0);
    PINCTRL_SetGeneralPadMode(PIN_RIGHT_WHEEL, IO_MODE_PWM, PWM_CH_MOTOR_RIGHT, 0);
    PWM_SetupSimple(PWM_CH_MOTOR_RIGHT, MOTOR_PWM_FREQ, 0);

    vehicle_reset();
}

void vehicle_reset(void)
{
    pid_reset(&vehicle_state.pid_steering);
    pid_reset(&vehicle_state.pid_speed);
    vehicle_state.filter_steering.value = 0.0;
    vehicle_state.filter_distance.value = 0.0;
    vehicle_set_steering(0);
    vehicle_set_motor_speed(0);
}

void vehicle_lock(void)
{
    vehicle_state.locked = 1;
    vehicle_set_motor_speed(0);
}

void vehicle_unlock(void)
{
    vehicle_state.locked = 0;
}

int dist_to_speed(float dist)
{
    if (dist > 5) return 70;
    else if (dist > 3) return 50;
    else if (dist > 1) return 30;
    else return 0;
}

void vehicle_update_steering(float azimuth)
{
    if (vehicle_state.locked) return;

    steering_pid_process(azimuth);
}

void vehicle_update(target_pos_t *pos)
{
    int steering = 0;
    if ((-STEERING_MAX_OFFSET <= pos->azimuth) && (pos->azimuth <= STEERING_MAX_OFFSET)
        && (pos->elevation < 70))
        vehicle_update_steering(-pos->azimuth);

    iir_update(&vehicle_state.filter_distance, pos->distance);

    if (platform_get_us_time() - vehicle_state.last_update < UPDATE_PERIOD)
        return;

    if (vehicle_state.locked) goto reset_param;

    steering = -(int)vehicle_state.filter_steering.value;
    if ((steering < -STEERING_MAX_OFFSET) || (steering > STEERING_MAX_OFFSET))
    {
        vehicle_set_motor_speed(0);
    }
    else
    {
        //steering_pid_process(steering);
        vehicle_set_motor_speed(dist_to_speed(vehicle_state.filter_distance.value));
    }

reset_param:
    vehicle_state.last_update = platform_get_us_time();
}
