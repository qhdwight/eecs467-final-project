/**
 * This file is the main executable for the MBot firmware.
 */

#include <comms/common.h>
#include <comms/listener.h>
#include <comms/messages_mb.h>
#include <comms/protocol.h>
#include <comms/topic_data.h>
#include <pico/multicore.h>
#include <pico/mutex.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <rc/encoder/encoder.h>
#include <rc/motor/motor.h>
#include <rc/mpu/mpu.h>

#include "mbot.h"
#include <inttypes.h>
#include <math.h>

#define LED_PIN 25
#define MAIN_LOOP_HZ 50.0 // 50 hz loop
#define MAIN_LOOP_PERIOD (1.0f / MAIN_LOOP_HZ)

#define LEFT_MOTOR_CHANNEL 1
#define RIGHT_MOTOR_CHANNEL 3

// data to hold current mpu state
static mb_mpu_data_t mpu_data;

uint64_t timestamp_offset = 0;
uint64_t current_pico_time = 0;

float enc2meters = ((2.0 * PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

void timestamp_cb(timestamp_t* received_timestamp) {
    // if we havent set the offset yet
    if (timestamp_offset == 0) {
        uint64_t cur_pico_time = to_us_since_boot(get_absolute_time());
        timestamp_offset = received_timestamp->utime - cur_pico_time;
    }
}

void reset_encoders_cb(mbot_encoder_t* received_encoder_vals) {
    rc_encoder_write(LEFT_MOTOR_CHANNEL, received_encoder_vals->leftticks);
    rc_encoder_write(RIGHT_MOTOR_CHANNEL, received_encoder_vals->rightticks);
}

void reset_odometry_cb(odometry_t* received_odom) {
    current_odom.utime = received_odom->utime;
    current_odom.x = received_odom->x;
    current_odom.y = received_odom->y;
    current_odom.theta = received_odom->theta;
}

int write_pid_coefficients(i2c_inst_t* i2c) {
    uint8_t pid_bytes[PID_VALUES_LEN];
    memcpy(pid_bytes, &mbot_pid_gains, PID_VALUES_LEN);
    return mb_write_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, &pid_bytes[0]);
}

void pid_values_cb(mbot_pid_gains_t* received_pid_gains) {
    memcpy(&mbot_pid_gains, received_pid_gains, sizeof(mbot_pid_gains_t));
    write_pid_coefficients(i2c);
}

void register_topics() {
    // timesync topic
    comms_register_topic(MBOT_TIMESYNC, sizeof(timestamp_t), (Deserialize) &timestamp_t_deserialize, (Serialize) &timestamp_t_serialize, (MsgCb) &timestamp_cb);
    // odometry topic
    comms_register_topic(ODOMETRY, sizeof(odometry_t), (Deserialize) &odometry_t_deserialize, (Serialize) &odometry_t_serialize, NULL);
    // reset odometry topic
    comms_register_topic(RESET_ODOMETRY, sizeof(odometry_t), (Deserialize) &odometry_t_deserialize, (Serialize) &odometry_t_serialize, (MsgCb) &reset_odometry_cb);
    // IMU topic
    comms_register_topic(MBOT_IMU, sizeof(mbot_imu_t), (Deserialize) &mbot_imu_t_deserialize, (Serialize) &mbot_imu_t_serialize, NULL);
    // encoders topic
    comms_register_topic(MBOT_ENCODERS, sizeof(mbot_encoder_t), (Deserialize) &mbot_encoder_t_deserialize, (Serialize) &mbot_encoder_t_serialize, NULL);
    // reset encoders topic
    comms_register_topic(RESET_ENCODERS, sizeof(mbot_encoder_t), (Deserialize) &mbot_encoder_t_deserialize, (Serialize) &mbot_encoder_t_serialize, (MsgCb) &reset_encoders_cb);
    // motor commands topic
    comms_register_topic(MBOT_MOTOR_COMMAND, sizeof(mbot_motor_command_t), (Deserialize) &mbot_motor_command_t_deserialize, (Serialize) &mbot_motor_command_t_serialize, NULL);
    // PID values topic
    comms_register_topic(MBOT_PIDS, sizeof(mbot_pid_gains_t), (Deserialize) &mbot_pid_gains_t_deserialize, (Serialize) &mbot_pid_gains_t_serialize, (MsgCb) &pid_values_cb);
}

void read_pid_coefficients(i2c_inst_t* i2c) {
    uint8_t pid_bytes[PID_VALUES_LEN];

    if (mb_read_fram(i2c, PID_VALUES_ADDR, PID_VALUES_LEN, pid_bytes) > 0) {
        printf("reading fram success.\n");
        memcpy(&mbot_pid_gains, pid_bytes, PID_VALUES_LEN);
        printf("read gains from fram!\r\n");
    } else {
        printf("reading PID gains from fram failure.\n");
    }
}

float open_loop_control(int channel, float sp) {
    if (channel == LEFT_MOTOR_CHANNEL) {
        if (sp > 0) {
            return SLOPE_L * sp + INTERCEPT_L;
        } else {
            return SLOPE_L_REVERSE * sp + INTERCEPT_L_REVERSE;
        }
    } else if (channel == RIGHT_MOTOR_CHANNEL) {
        if (sp > 0) {
            return SLOPE_R * sp + INTERCEPT_R;
        } else {
            return SLOPE_R_REVERSE * sp + INTERCEPT_R_REVERSE;
        }
    }

    return 0;
}

float clamp_orientation(float theta) {
    while (theta < 0) {
        theta += 2 * PI;
    }

    while (theta >= 2 * PI) {
        theta -= 2 * PI;
    }

    return theta;
}

bool timer_cb(repeating_timer_t* rt) {
    // Read the PID values
    if (comms_get_topic_data(MBOT_PIDS, &mbot_pid_gains)) {
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
    if (comms_get_topic_data(MBOT_TIMESYNC, &received_time)) {
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
        float delta_d, delta_theta; // displacement in meters and rotation in radians
        float delta_theta_thres = 0.125;

        // update gyro data
        float curr_gyro = mpu_data.dmp_TaitBryan[2];
        float delta_theta_gyro = curr_gyro - current_imu.tb[2];
        current_imu.tb[2] = curr_gyro;

        // compute odometry parameters
        float delta_sL = enc_delta_l * enc2meters;
        float delta_sR = enc_delta_r * enc2meters;

        delta_d = (delta_sL + delta_sR) / 2;
        delta_theta = (delta_sR - delta_sL) / WHEEL_BASE; // delta_theta_odo
        float delta_GO = delta_theta_gyro - delta_theta;

        if (GYRODOMETRY && abs(delta_GO) > delta_theta_thres) {
            delta_theta = delta_theta_gyro;
        }

        float delta_x = delta_d * cos(current_odom.theta + delta_theta / 2);
        float delta_y = delta_d * sin(current_odom.theta + delta_theta / 2);

        current_odom.utime = cur_pico_time;
        current_odom.x += delta_x;
        current_odom.y += delta_y;
        current_odom.theta += delta_theta;
        current_odom.theta = clamp_orientation(current_odom.theta);

        /*************************************************************
         * End of TODO
         *************************************************************/

        // get the current motor command state (if we have one)
        if (comms_get_topic_data(MBOT_MOTOR_COMMAND, &current_cmd)) {
            // Clamp duty cycle to [-1, 1]
            float roller_duty = clamp_duty(current_cmd.angular_v);
            float kicker_duty = clamp_duty(current_cmd.trans_v);
            // duty to motor command
            int16_t roller_cmd = (int) (roller_duty * 0.95 * pow(2, 15));
            int16_t kicker_cmd = (int) (kicker_duty * 0.95 * pow(2, 15));
            // set roller motor command
            rc_motor_set(RIGHT_MOTOR_CHANNEL, roller_cmd);
            rc_motor_set(LEFT_MOTOR_CHANNEL, kicker_cmd);
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

int main() {
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
    uint const sda_pin = 4;
    uint const scl_pin = 5;

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
    rc_filter_pid(&left_pid,
                  left_pid_params.kp,
                  left_pid_params.ki,
                  left_pid_params.kd,
                  1.0 / left_pid_params.dFilterHz,
                  1.0 / MAIN_LOOP_HZ);
    // rc_filter_enable_saturation(&left_pid, -1, 1);

    right_pid = rc_filter_empty();
    rc_filter_pid(&right_pid,
                  right_pid_params.kp,
                  right_pid_params.ki,
                  right_pid_params.kd,
                  1.0 / right_pid_params.dFilterHz,
                  1.0 / MAIN_LOOP_HZ);
    // rc_filter_enable_saturation(&right_pid, -1, 1);

    fwd_vel_pid = rc_filter_empty();
    rc_filter_pid(&fwd_vel_pid,
                  fwd_vel_pid_params.kp,
                  fwd_vel_pid_params.ki,
                  fwd_vel_pid_params.kd,
                  1.0 / fwd_vel_pid_params.dFilterHz,
                  1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&fwd_vel_pid, -0.05, 0.05);

    turn_vel_pid = rc_filter_empty();
    rc_filter_pid(&turn_vel_pid,
                  turn_vel_pid_params.kp,
                  turn_vel_pid_params.ki,
                  turn_vel_pid_params.kd,
                  1.0 / turn_vel_pid_params.dFilterHz,
                  1.0 / MAIN_LOOP_HZ);
    rc_filter_enable_saturation(&turn_vel_pid, -2 / 3, 2 / 3);

    /*************************************************************
     * End of TODO
     *************************************************************/

    if (OPEN_LOOP) {
        printf("Running in open loop mode\n");
    } else {
        printf("Running in closed loop mode\n");
    }

    while (running) {
        printf("\033[2A\r|      SENSORS      |           ODOMETRY          |     SETPOINTS     |\n\r|  L_ENC  |  R_ENC  |    X    |    Y    |    θ    |   FWD   |   ANG   \n\r|%7lld  |%7lld  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |%7.3f  |", current_encoders.leftticks, current_encoders.rightticks, current_odom.x, current_odom.y, current_odom.theta, current_cmd.trans_v, current_cmd.angular_v);
    }
}

/**
 * @brief Clamp duty cycle between -1 and 1. If not applied, robot drives in reverse only
 *
 * @param duty
 */
float clamp_duty(float duty) {
    if (duty > 1.0) {
        return 1.0;
    } else if (duty < -1.0) {
        return -1.0;
    }
    return duty;
}
