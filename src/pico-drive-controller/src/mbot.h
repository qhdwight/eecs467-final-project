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
#define WHEEL_RADIUS 0.043
#define GEAR_RATIO 78.0
#define ENCODER_RES 20.0
#define WHEEL_BASE 0.159 // wheel separation distance in meters
#define MAX_FWD_VEL 0.8 // max forward speed (m/s)
#define MESSAGE_CONFIRMATION_CHANNEL "MSG_CONFIRM"
#define MAX_TURN_VEL 2.5 // max turning speed (rad/s)
#define PI 3.141592653589793

// TODO: Enter the polarity values for your motors and encoders
#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL -1
#define RIGHT_MOTOR_POL 1

// TODO: Populate with calibration data (recommended to generate these for reverse direction as well)
#define BOUND_L_N       -0.12
#define SLOPE_L_N       1.6489216
#define INTERCEPT_L_N   -0.11543867
#define BOUND_L_P       0.08
#define SLOPE_L_P       1.6181966
#define INTERCEPT_L_P   0.10764143
#define BOUND_R_N       -0.12
#define SLOPE_R_N       1.58455956
#define INTERCEPT_R_N   -0.12126849
#define BOUND_R_P       0.12
#define SLOPE_R_P       1.6181966
#define INTERCEPT_R_P   0.10764143

// TODO: Decide which controller is used, open loop = 1, PID = 0
#define OPEN_LOOP 0

// Decide whether to use gyro when calculating odometry
#define GYRODOMETRY 0
#define GYRO_THRESH 0.3
double gyro_reading = 0.0;

// Decide which case to use
#define CASE_3 0
#define CASE_4 1

// data to hold current mpu state (not used)
//static mb_mpu_data_t mpu_data;
static i2c_inst_t *i2c;

uint64_t timestep_us = 0;

// data to hold calibration coefficients
float coeffs[4];

// data to hold the PID values
static mbot_pid_gains_t mbot_pid_gains;

typedef struct pid_parameters pid_parameters_t;
struct pid_parameters
{
    float kp;
    float ki;
    float kd;
    float dFilterHz;
};

float clamp_duty(float duty);

// data to hold the IMU results
mbot_imu_t current_imu = {0};
// current odometry state
odometry_t current_odom = {0};
// current encoder states
mbot_encoder_t current_encoders = {0};
// current body frame command
mbot_motor_command_t current_cmd = {0};

/**
 * Example filter and PID parameter initialization
 *
 * rc_filter_t my_filter;
 *
 * pid_parameters_t pid_params = {
 *    .kp = 1.0,
 *    .ki = 0.0,
 *    .kd = 0.0,
 *    .dFilterHz = 25.0
 * };
 */

rc_filter_t left_pid;
rc_filter_t right_pid;
rc_filter_t fwd_vel_pid;
rc_filter_t turn_vel_pid;
rc_filter_t pwm_left_pid;
rc_filter_t pwm_right_pid;

pid_parameters_t left_pid_params = {
    .kp = 2.80,
    .ki = 15, // 1.25 /// 15
    .kd = 0.5,
    .dFilterHz = 25.0,
};
pid_parameters_t right_pid_params = {
    .kp = 2.80,
    .ki = 15, // 1.25 // 15
    .kd = 0.5,
    .dFilterHz = 25.0,
};
pid_parameters_t fwd_vel_pid_params = {
    .kp = 0.8,
    .ki = 0.1,
    .kd = 0.0,
    .dFilterHz = 10.0,
};
pid_parameters_t turn_vel_pid_params = {
    .kp = 0.8,
    .ki = 0.1,
    .kd = 0.0,
    .dFilterHz = 10.0,
};
pid_parameters_t pwm_left_pid_params = {
    .kp = 2.0,
    .ki = 15,
    .kd = 0.001,
    .dFilterHz = 10.0,
};
pid_parameters_t pwm_right_pid_params = {
    .kp = 2.0,
    .ki = 15,
    .kd = 0.001,
    .dFilterHz = 10.0,
};


float clamp_duty(float duty);

float clamp_angle(float angle);

#endif

