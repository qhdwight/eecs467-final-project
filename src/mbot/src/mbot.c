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
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <comms/messages_mb.h>

#include <math.h>
#include <inttypes.h>
#include "mbot.h"

#define LED_PIN 25
#define MAIN_LOOP_HZ 25.0 // 50 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

// data to hold current mpu state
static mb_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

float prev_x = 0;
float prev_theta = 0;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

float left_target_vel = 0;
float right_target_vel = 0;
float left_pwm = 0;
float right_pwm = 0;
float left_error;
float right_error;

void timestamp_cb(timestamp_t *received_timestamp)
{
    // if we havent set the offset yet
    if (timestamp_offset == 0)
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t *received_encoder_vals)
{
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t *received_odom)
{
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return mb_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t *received_pid_gains)
{
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients(i2c);
}

void register_topics()
{
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize)&timestamp_t_deserialize, (Serialize)&timestamp_t_serialize, (MsgCb)&timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize)&odometry_t_deserialize, (Serialize)&odometry_t_serialize, (MsgCb)&reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize)&mbot_imu_t_deserialize, (Serialize)&mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize)&mbot_encoder_t_deserialize, (Serialize)&mbot_encoder_t_serialize, (MsgCb)&reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize)&mbot_motor_command_t_deserialize, (Serialize)&mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize)&mbot_pid_gains_t_deserialize, (Serialize)&mbot_pid_gains_t_serialize, (MsgCb)&pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t *i2c)
{
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (mb_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0)
    {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    }
    else
    {
        printf("reading PID gains from fram failure.\n");
    }
}

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
    // Read the PID values
    if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains))
    {
        uint64_t msg_time = current_pico_time;
        // Print the PID values
        printf("Left: %f, %f, %f, %f", mbot_pid_gains.motor_a_kp,
               mbot_pid_gains.motor_a_ki,
               mbot_pid_gains.motor_a_kd,
               1.0 / mbot_pid_gains.motor_a_Tf);

        // update the PID gains of the left motor PID controller
        rc_filter_pid(&left_pid,
                      mbot_pid_gains.motor_a_kp,
                      mbot_pid_gains.motor_a_ki,
                      mbot_pid_gains.motor_a_kd,
                      1.0 / mbot_pid_gains.motor_a_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the right motor PID controller
        rc_filter_pid(&right_pid,
                      mbot_pid_gains.motor_c_kp,
                      mbot_pid_gains.motor_c_ki,
                      mbot_pid_gains.motor_c_kd,
                      1.0 / mbot_pid_gains.motor_c_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame controller
        rc_filter_pid(&fwd_vel_pid,
                      mbot_pid_gains.bf_trans_kp,
                      mbot_pid_gains.bf_trans_ki,
                      mbot_pid_gains.bf_trans_kd,
                      1.0 / mbot_pid_gains.bf_trans_Tf,
                      1.0 / MAIN_LOOP_HZ);

        // update the PID gains of the body frame rotation controller
        rc_filter_pid(&turn_vel_pid,
                      mbot_pid_gains.bf_rot_kp,
                      mbot_pid_gains.bf_rot_ki,
                      mbot_pid_gains.bf_rot_kd,
                      1.0 / mbot_pid_gains.bf_rot_Tf,
                      1.0 / MAIN_LOOP_HZ);
    }
    // only run if we've received a timesync message...
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time))
    {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time()) + timestamp_offset;
        uint64_t latency_time = cur_pico_time - current_pico_time;
        current_pico_time = cur_pico_time;
        // first, get the IMU data and send across the wire
        current_imu.utime = cur_pico_time; // received_time.utime;

        // read the encoders
        int enc_cnt_l = LEFT_ENC_POL * rc_encoder_read_count(LEFT_MOTOR_CHANNEL);
        int enc_delta_l = LEFT_ENC_POL * rc_encoder_read_delta(LEFT_MOTOR_CHANNEL);
        int enc_cnt_r = RIGHT_ENC_POL * rc_encoder_read_count(RIGHT_MOTOR_CHANNEL);
        int enc_delta_r = RIGHT_ENC_POL * rc_encoder_read_delta(RIGHT_MOTOR_CHANNEL);
        current_encoders.utime = cur_pico_time; // received_time.utime;
        current_encoders.right_delta = enc_delta_r;
        current_encoders.rightticks = enc_cnt_r;
        current_encoders.left_delta = enc_delta_l;
        current_encoders.leftticks = enc_cnt_l;

        // compute new odometry
        /*************************************************************
         * TODO:
         *       -SECTION 1.3 ODOMETRY
         *      - Populate the odometry messages.
         *          -The struct for odometry_t is defined in comms/include/comms/messages_mb.h (DO NOT EDIT THIS FILE)
         *      - Note that the way we compute the displacement of the motor is from encoder readings and
         *        that we convert encoder readings to meters using the enc2meters variable defined above.
         *      - Use the equations provided in the document to compute the odometry components
         *      - Remember to clamp the orientation between [0, 2pi]!
         *************************************************************/
        float delta_left = current_encoders.left_delta * enc2meters;
        float delta_right = current_encoders.right_delta * enc2meters;
        float delta_theta = 0.88 * (delta_right - delta_left) / WHEEL_BASE;

        if (GYRODOMETRY) {
            float temp_delta_theta = mpu_data.dmp_TaitBryan[2] - prev_theta;
            prev_theta = mpu_data.dmp_TaitBryan[2];
            if (abs(temp_delta_theta) > GYRO_THRESH) {
                delta_theta = temp_delta_theta;
            }
        }

        float delta_d = (delta_right + delta_left) / 2;
        float delta_x = delta_d * cos(current_odom.theta + delta_theta / 2);
        float delta_y = delta_d * sin(current_odom.theta + delta_theta / 2);
        current_odom.x += delta_x;
        current_odom.y += delta_y;
        current_odom.theta += delta_theta;
        current_odom.utime = cur_pico_time;

        /*************************************************************
         * End of TODO
         *************************************************************/

        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd))
        {
            int16_t l_cmd, r_cmd;                 // left and right motor commands
            float left_sp, right_sp;              // speed in m/s
            float measured_vel_l, measured_vel_r; // measured velocity in m/s
            float l_duty, r_duty;                 // duty cycle in range [-1, 1]
            float dt = MAIN_LOOP_PERIOD;          // time since last update in seconds

            // Find target left and right velocities
            float linear_vel = current_cmd.trans_v;
            float angular_vel = current_cmd.angular_v;
            left_sp = linear_vel - (WHEEL_BASE * angular_vel / 2);
            right_sp = linear_vel + (WHEEL_BASE * angular_vel / 2);

            // Find current left and right velocities
            const float RPM_conversion_factor = 60 / (GEAR_RATIO * MAIN_LOOP_PERIOD * ENCODER_RES);
            measured_vel_l = RPM_conversion_factor * current_encoders.left_delta * 2 * WHEEL_RADIUS * PI / 60;
            measured_vel_r = RPM_conversion_factor * current_encoders.right_delta * 2 * WHEEL_RADIUS * PI / 60;

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
                 *      - To calculate the measured velocity, use MAIN_LOOP_PERIOD or latency_time
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
                float fwd_sp, turn_sp;                     // forward and turn setpoints in m/s and rad/s

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
                        float measured_vel_turn = (measured_vel_r - measured_vel_l) / WHEEL_BASE;

                        printf("%.3f,%.3f,%.3f,%.3f,%.3f,%lu\n", current_odom.x, current_odom.y, current_odom.theta, measured_vel_fwd, measured_vel_turn, current_odom.utime);
                    }
                    else {
                        if (CASE_3) {
                            // Frame velocity controller
                            float measured_vel_fwd = (measured_vel_l + measured_vel_r) / 2.0;
                            float measured_vel_turn = (measured_vel_r - measured_vel_l) / WHEEL_BASE;
                            float vel_fwd_error = linear_vel - measured_vel_fwd;
                            float vel_turn_error = angular_vel - measured_vel_turn;
                            linear_vel = measured_vel_fwd + rc_filter_march(&fwd_vel_pid, vel_fwd_error);
                            angular_vel = measured_vel_turn + rc_filter_march(&turn_vel_pid, vel_turn_error);
                            left_sp = linear_vel - (WHEEL_BASE * angular_vel / 2.0);
                            right_sp = linear_vel + (WHEEL_BASE * angular_vel / 2.0);

                            //printf("%.3f,%.3f,%.3f,%.3f,%.3f,%lu\n", current_odom.x, current_odom.y, current_odom.theta, measured_vel_fwd, measured_vel_turn, current_odom.utime);
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
        }

        // write the encoders to serial
        comms_write_topic(MBOT_ENCODERS, &current_encoders);
        // send odom on wire
        comms_write_topic(ODOMETRY, &current_odom);

        // write the IMU to serial
        comms_write_topic(MBOT_IMU, &current_imu);
        uint64_t fn_run_len = to_us_since_boot(get_absolute_time()) + timestamp_offset - cur_pico_time;
    }

    return true;
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

    // Pins
    // for the i2c to the IMU
    const uint sda_pin = 4;
    const uint scl_pin = 5;

    // Ports
    i2c = i2c0;
    // Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);
    // Initialize I2C pins
    printf("setting i2c functions...\n");
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    printf("setting i2c pullups...\n");
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(sda_pin, scl_pin, GPIO_FUNC_I2C));

    printf("setting heartbeat LED GPIOs...\n");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    printf("initializing DMP...\n");
    mb_mpu_config_t mpu_config = mb_mpu_default_config();
    mpu_config.i2c_bus = i2c;
    mpu_config.dmp_fetch_accel_gyro = 1;
    mpu_config.enable_magnetometer = 0;
    mpu_config.read_mag_after_callback = 0;
    mpu_config.orient = ORIENTATION_Z_UP;
    mpu_config.dmp_sample_rate = 200;
    mb_mpu_reset_accel_cal(mpu_config.i2c_bus);
    mb_mpu_calibrate_gyro_routine(mpu_config);
    // sleep_ms(2000);
    // mb_mpu_calibrate_accel_routine(mpu_config);
    sleep_ms(500);
    mb_mpu_initialize_dmp(&mpu_data, mpu_config);
    gpio_set_irq_enabled_with_callback(MB_MPU_INTERRUPT_GPIO, GPIO_IRQ_EDGE_FALL, true, &mb_dmp_callback);
    printf("MPU Initialized!\n");

    // create topics and register the serialize/deserialize functions
    printf("init comms...\r\n");
    comms_init_protocol();
    comms_init_topic_data();
    register_topics();

    // launch the other core's comm loop
    printf("starting comms on core 1...\r\n");
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    int running = 1;

    // run the main loop as a timed interrupt
    printf("starting the timed interrupt...\r\n");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_PERIOD * 1000, timer_cb, NULL, &loop_timer); // 1000x to convert to ms

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
    //             1.0 / MAIN_LOOP_HZ);

    // Example of setting limits to the output of the filter
    // rc_filter_enable_saturation(&my_filter, min_val, max_val);

    left_pid = rc_filter_empty();
    right_pid = rc_filter_empty();
    fwd_vel_pid = rc_filter_empty();
    turn_vel_pid = rc_filter_empty();
    pwm_left_pid = rc_filter_empty();
    pwm_right_pid = rc_filter_empty();

    rc_filter_pid(&left_pid, left_pid_params.kp, left_pid_params.ki, left_pid_params.kd, left_pid_params.dFilterHz, MAIN_LOOP_PERIOD);
    rc_filter_pid(&right_pid, right_pid_params.kp, right_pid_params.ki, right_pid_params.kd, right_pid_params.dFilterHz, MAIN_LOOP_PERIOD);
    rc_filter_pid(&fwd_vel_pid, fwd_vel_pid_params.kp, fwd_vel_pid_params.ki, fwd_vel_pid_params.kd, fwd_vel_pid_params.dFilterHz, MAIN_LOOP_PERIOD);
    rc_filter_pid(&turn_vel_pid, turn_vel_pid_params.kp, turn_vel_pid_params.ki, turn_vel_pid_params.kd, turn_vel_pid_params.dFilterHz, MAIN_LOOP_PERIOD);
    rc_filter_pid(&pwm_left_pid, pwm_left_pid_params.kp, pwm_left_pid_params.ki, pwm_left_pid_params.kd, pwm_left_pid_params.dFilterHz, MAIN_LOOP_PERIOD);
    rc_filter_pid(&pwm_right_pid, pwm_right_pid_params.kp, pwm_right_pid_params.ki, pwm_right_pid_params.kd, pwm_right_pid_params.dFilterHz, MAIN_LOOP_PERIOD);

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

    while (running)
    {
        // printf("\033[2A\r|      SENSORS      |           ODOMETRY          |     SETPOINTS     |\n\r|  L_ENC  |  R_ENC  |    X    |    Y    |    θ    |   FWD   |   ANG   \n\r|%7lld  |%7lld  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |", current_encoders.leftticks, current_encoders.rightticks, current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v);
        // if (current_odom.x != prev_x) {
        //     prev_x = current_odom.x;
        //     printf("%.3f\n", current_odom.x);
        // }
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