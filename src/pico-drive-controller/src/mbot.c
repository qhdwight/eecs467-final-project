/**
 * This file is the main executable for the MBot firmware.
 */

#include <pico/stdlib.h>
#include <pico/mutex.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>

#include <math.h>
#include <inttypes.h>
#include "mbot.h"

#define LED_PIN 25
#define MBOT_MAIN_LOOP_HZ 25.0 // 50 hz loop
#define MBOT_MAIN_LOOP_PERIOD (1.0f / MBOT_MAIN_LOOP_HZ)

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

// current body frame command
mbot_motor_command_t current_cmd = {0};
mutex_t motor_command_mutex;

float prev_x = 0;
float prev_theta = 0;

float enc2meters = ((2.0 * M_PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

float left_target_vel = 0;
float right_target_vel = 0;
float left_pwm = 0;
float right_pwm = 0;
float left_error;
float right_error;

float l_vel_to_duty(float vel) {
    if (vel > BOUND_L_N && vel < BOUND_L_P) {
        return 0;
    }
    else if (vel < 0) {
        return SLOPE_L_N * vel + INTERCEPT_L_N;
    }
    else {
        return SLOPE_L_P * vel + INTERCEPT_L_P;
    }
}

float r_vel_to_duty(float vel) {
    if (vel > BOUND_R_N && vel < BOUND_R_P) {
        return 0;
    }
    else if (vel < 0) {
        return SLOPE_R_N * vel + INTERCEPT_R_N;
    }
    else {
        return SLOPE_R_P * vel + INTERCEPT_R_P;
    }
}

bool timer_cb(repeating_timer_t *rt)
{
    // read the encoders
    int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
    int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);

    int16_t l_cmd, r_cmd;                 // left and right motor commands
    float left_sp, right_sp;              // speed in m/s
    float measured_vel_l, measured_vel_r; // measured velocity in m/s
    float l_duty, r_duty;                 // duty cycle in range [-1, 1]

    // Find target left and right velocities
    mutex_enter_blocking(&motor_command_mutex);
    float linear_vel = current_cmd.trans_v;
    float angular_vel = current_cmd.angular_v;
    mutex_exit(&motor_command_mutex);
    left_sp = linear_vel - (MBOT_WHEEL_BASE * angular_vel / 2);
    right_sp = linear_vel + (MBOT_WHEEL_BASE * angular_vel / 2);

    // Find current left and right velocities
    const float RPM_conversion_factor = 60 / (GEAR_RATIO * MBOT_MAIN_LOOP_PERIOD * ENCODER_RES);
    measured_vel_l = RPM_conversion_factor * enc_delta_l * 2 * WHEEL_RADIUS * M_PI / 60;
    measured_vel_r = RPM_conversion_factor * enc_delta_r * 2 * WHEEL_RADIUS * M_PI / 60;

    if (OPEN_LOOP)
    {
        /*************************************************************
         * TODO:
         *      -SECTION 1.2 OPEN LOOP CONTROL
         *      - Implement the open loop motor controller to compute the left
         *          and right wheel commands
         *      - Determine the setpoint velocities for left and right motor using the wheel velocity model
         ************************************************************/

        // Set duty cycles from calibration only
        l_duty = l_vel_to_duty(left_sp);
        r_duty = r_vel_to_duty(right_sp);

        left_target_vel = left_sp;
        right_target_vel = right_sp;
        left_pwm = l_duty;
        right_pwm = r_duty;


        /*************************************************************
         * End of TODO
         *************************************************************/
    }
    else
    {
        /*************************************************************
         * TODO:
         *       -SECTION 1.2 PID CONTROL
         *      - Implement the closed loop motor controller to compute the left
         *          and right wheel commands
         *      - To calculate the measured velocity, use MBOT_MAIN_LOOP_PERIOD or latency_time
         *          as the timestep
         *      - Compute the error between the setpoint velocity and the measured velocity
         *      - We recommend to use the open loop controller as part of the closed loop to improve
         *          performance.
         *          Example: open_loop_control(LEFT_MOTOR_CHANNEL, left_sp)
         *      - Use the PID filters defined in mbot.h and main() function to calculate desired
         *          duty, use rc_filter_march() function.
         *          Example: rc_filter_march(&left_pid, left_error)
         * TODO:
                -SECTION 1.5 FRAME VELOCITY CONTROL
            *      - Compute the error between the target and measured translation+rotation velocity
            *      - Compute the new forward and rotation setpoints that will be used in
            *          the wheel speed PID (these should be written in lines above
            *          the previous controller)
            *      - Update and the fwd_sp and turn_sp variables for this.
            *
            ************************************************************/

        if (linear_vel == 0 && angular_vel == 0) {
            l_duty = 0;
            r_duty = 0;
        }
        else {
            if (CASE_4) {
                float l_cal_duty = l_vel_to_duty(left_sp);
                float r_cal_duty = r_vel_to_duty(right_sp);
                float l_error = left_sp - measured_vel_l;
                float r_error = right_sp - measured_vel_r;

                l_duty = l_cal_duty + rc_filter_march(&pwm_left_pid, l_error);
                r_duty = r_cal_duty + rc_filter_march(&pwm_right_pid, r_error);

                float measured_vel_fwd = (measured_vel_l + measured_vel_r) / 2.0;
                float measured_vel_turn = (measured_vel_r - measured_vel_l) / MBOT_WHEEL_BASE;

                printf("%.3f,%.3f\n", measured_vel_fwd, measured_vel_turn);
            }
            else {
                if (CASE_3) {
                    // Frame velocity controller
                    float measured_vel_fwd = (measured_vel_l + measured_vel_r) / 2.0;
                    float measured_vel_turn = (measured_vel_r - measured_vel_l) / MBOT_WHEEL_BASE;
                    float vel_fwd_error = linear_vel - measured_vel_fwd;
                    float vel_turn_error = angular_vel - measured_vel_turn;
                    linear_vel = measured_vel_fwd + rc_filter_march(&fwd_vel_pid, vel_fwd_error);
                    angular_vel = measured_vel_turn + rc_filter_march(&turn_vel_pid, vel_turn_error);
                    left_sp = linear_vel - (MBOT_WHEEL_BASE * angular_vel / 2.0);
                    right_sp = linear_vel + (MBOT_WHEEL_BASE * angular_vel / 2.0);
                }

                /**
                 *  Example closed loop controller
                 *      (assuming the error between the target and measured is computed)
                 *
                 * float pid_delta_vel = rc_filter_march(&pid_filter, error);
                 * float desired_vel = commanded_val + pid_delta_vel;
                 */

                left_error = left_sp - measured_vel_l;
                right_error = right_sp - measured_vel_r;
                float l_delta_vel = rc_filter_march(&left_pid, left_error);
                float r_delta_vel = rc_filter_march(&right_pid, right_error);
                float goal_left_vel = measured_vel_l + l_delta_vel;
                float goal_right_vel = measured_vel_r + r_delta_vel;
                l_duty = l_vel_to_duty(goal_left_vel);
                r_duty = r_vel_to_duty(goal_right_vel);
            }
        }

        /*************************************************************
         * End of TODO
         *************************************************************/
    }
    // Clamp duty cycle to [-1, 1]
    l_duty = clamp_duty(l_duty);
    r_duty = clamp_duty(r_duty);

    // duty to motor command
    l_cmd = LEFT_MOTOR_POL * (int)(l_duty * 0.95 * pow(2, 15));
    r_cmd = RIGHT_MOTOR_POL * (int)(r_duty * 0.95 * pow(2, 15));

    // set left and right motor command
    rc_motor_set(LEFT_MOTOR_CHANNEL, l_cmd);
    rc_motor_set(RIGHT_MOTOR_CHANNEL, r_cmd);

    return true;
}

void comms_listener_loop(void)
{
    printf("Running listener loop\n");
    bool running = true;
    while(running)
    {
        // read from serial until we get first byte of header 0xAB
        char sync_byte = 0;
        do 
        {
            stdio_usb_in_chars_itf(1, &sync_byte, 1);
        } while(sync_byte != 0xAB);

        // Verify second byte of header is 0xCD
        stdio_usb_in_chars_itf(1, &sync_byte, 1);
        if (sync_byte != 0xCD)
        {
            printf("Failed receiving second header byte\n");
            continue;
        }

        // read the rest of the data
        mbot_motor_command_t motor_command;
        stdio_usb_in_chars_itf(1, (char*)(&motor_command), sizeof(motor_command));

        // Read footer 0xEF
        stdio_usb_in_chars_itf(1, &sync_byte, 1);
        if (sync_byte != 0xEF)
        {
            printf("Failed receiving footer\n");
            continue;
        }

        // Should have received good data so update motor command
        printf("Received new motor command: v=%.03f, w=%.03f\n", motor_command.trans_v, motor_command.angular_v);
        mutex_enter_blocking(&motor_command_mutex);
        current_cmd = motor_command;
        mutex_exit(&motor_command_mutex);

        sleep_us(1); // brief sleep to allow FIFO to flush
    }
}

int main()
{
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    set_sys_clock_khz(125000, true);

    stdio_init_all();
    sleep_ms(1500); // quick sleep so we can catch the bootup process in terminal
    printf("\nMBot Booting Up!\n");

    printf("initing motors...\n");
    rc_motor_init();
    printf("initing encoders...\n");
    rc_encoder_init();

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    mutex_init(&motor_command_mutex);
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MBOT_MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

    printf("Done Booting Up!\n\n");
    /*************************************************************
     * TODO:
     *      - Initilize the PID Filters rc_filter_empty()
     *      - Set the PID gains rc_filter_pid()
     * TODO:
     *      - Initialize the PID filters for translation and rotation vel
     *      - Set the PID gains for translation and rotation vel
     *
     *************************************************************/

    // Example initialization of a PID filter defined in mbot.h
    // my_filter = rc_filter_empty();

    // Example of assigning PID parameters (using pid_parameters_t from mbot.h)
    // rc_filter_pid(&my_filter,
    //             pid_params.kp,
    //             pid_params.ki,
    //             pid_params.kd,
    //             1.0 / pid_params.dFilterHz,
    //             1.0 / MBOT_MAIN_LOOP_HZ);

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    left_pid = rc_filter_empty();
    right_pid = rc_filter_empty();
    fwd_vel_pid = rc_filter_empty();
    turn_vel_pid = rc_filter_empty();
    pwm_left_pid = rc_filter_empty();
    pwm_right_pid = rc_filter_empty();

    rc_filter_pid(&left_pid, left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_pid, right_pid_params.kp, right_pid_params.ki, right_pid_params.kd, right_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);
    rc_filter_pid(&fwd_vel_pid, fwd_vel_pid_params.kp, fwd_vel_pid_params.ki, fwd_vel_pid_params.kd, fwd_vel_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);
    rc_filter_pid(&turn_vel_pid, turn_vel_pid_params.kp, turn_vel_pid_params.ki, turn_vel_pid_params.kd, turn_vel_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);
    rc_filter_pid(&pwm_left_pid, pwm_left_pid_params.kp, pwm_left_pid_params.ki, pwm_left_pid_params.kd, pwm_left_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);
    rc_filter_pid(&pwm_right_pid, pwm_right_pid_params.kp, pwm_right_pid_params.ki, pwm_right_pid_params.kd, pwm_right_pid_params.dFilterHz, MBOT_MAIN_LOOP_PERIOD);

    /*************************************************************
     * End of TODO
     *************************************************************/

    if (OPEN_LOOP)
    {
        printf("Running in open loop mode\n");
    }
    else
    {
        printf("Running in closed loop mode\n");
    }

    while (true) {
        sleep_ms(1000);
    }

}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty)
{
    if (duty > 1.0)
    {
        return 1.0;
    }
    else if (duty < -1.0)
    {
        return -1.0;
    }
    return duty;
}

