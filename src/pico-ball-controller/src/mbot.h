#ifndef MBOT_H
#define MBOT_H

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/defs/common_defs.h>
#include <rc/defs/mbot_diff_defs.h>
#include <rc/fram/fram.h>
#include <rc/math/filter.h>
#include <rc/mpu/mpu.h>

#include <math.h>
#include <inttypes.h>

// Hardware info
#define GEAR_RATIO 78.0
#define ENCODER_RES 20.0
#define WHEEL_RADIUS 0.043

#define MOTOR_POL 1
#define ENC_POL 1

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_motor_command{
    uint8_t intake; // 1 is on, 0 is off
    uint8_t shoot_ball; // 1 is shoot, 0 is don't; If 1, this overrides intake
} mbot_ball_command_t;

#endif

