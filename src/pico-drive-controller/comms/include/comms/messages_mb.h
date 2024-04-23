#include <stdio.h>
#include <stdint.h>
// #include <memory.h>
#include <string.h>
#include <pico/binary_info.h> 

#ifndef MESSAGES_MB_H
#define MESSAGES_MB_H

enum message_topics{
    MBOT_TIMESYNC = 201, 
    MBOT_PIDS = 202, 
    ODOMETRY = 210, 
    RESET_ODOMETRY = 211, 
    MBOT_IMU = 220, 
    MBOT_MOTOR_COMMAND = 230, 
    OMNI_MOTOR_COMMAND = 230, 
    MBOT_ENCODERS = 240, 
    OMNI_ENCODERS = 241, 
    RESET_ENCODERS = 242
};

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) odometry{
    float x;
    float y;
    float theta;
} odometry_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_imu{
    float gyro[3];
    float accel[3];
    float mag[3];
    float tb[3];
    float temperature;
} mbot_imu_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_encoder{
    int64_t leftticks;
    int64_t rightticks;
    int16_t left_delta;
    int16_t right_delta;
} mbot_encoder_t;

typedef struct __attribute__((__packed__ )) omni_encoder{
    int64_t aticks;
    int64_t bticks;
    int64_t cticks;
    int16_t a_delta;
    int16_t b_delta;
    int16_t c_delta;
} omni_encoder_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_motor_command{
    float trans_v;
    float angular_v;
} mbot_motor_command_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) omni_motor_command{
    float vx;
    float vy;
    float wz;
} omni_motor_command_t;

// need to be packed in order to make the byte copy from numpy and C to play nicely
typedef struct __attribute__((__packed__ )) mbot_pid_gains{
    float motor_a_kp;
    float motor_a_ki;
    float motor_a_kd;
    float motor_a_Tf;

    float motor_b_kp;
    float motor_b_ki;
    float motor_b_kd;
    float motor_b_Tf;

    float motor_c_kp;
    float motor_c_ki;
    float motor_c_kd;
    float motor_c_Tf;

    float bf_trans_kp;
    float bf_trans_ki;
    float bf_trans_kd;
    float bf_trans_Tf;

    float bf_rot_kp;
    float bf_rot_ki;
    float bf_rot_kd;
    float bf_rot_Tf;
} mbot_pid_gains_t;

#endif


