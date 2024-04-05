#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/time.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <comms/common.h>
#include <comms/protocol.h>
#include <comms/listener.h>
#include <comms/topic_data.h>
#include <rc/defs/common_defs.h>
#include <rc/math/filter.h>

//auto update(repeating_timer_t *rt) -> bool {
//    return false;
//}

constexpr float MAIN_LOOP_PERIOD_S = 1.0f / MAIN_LOOP_HZ;

constexpr float HALF_WHEEL_BASE = WHEEL_BASE / 2;

std::array<int, 2> ENCODER_POLARITIES{1, 1};

std::array<rc_filter_t, 2> motor_pids{rc_filter_empty(), rc_filter_empty()};

auto twist_callback(geometry_msgs::Twist const &twist) -> void {
    rc_filter_pid(motor_pids.data() + 0, 0, 0, 0, 0, MAIN_LOOP_PERIOD_S);
    rc_filter_pid(motor_pids.data() + 1, 0, 0, 0, 0, MAIN_LOOP_PERIOD_S);

    std::array<int, 2> encoder_ticks{};
    for (std::size_t i = 0; i < encoder_ticks.size(); i++)
        encoder_ticks[i] = ENCODER_POLARITIES[i] * rc_encoder_read_count(i);

    std::array<int, 2> encoder_velocities{};
    for (std::size_t i = 0; i < encoder_velocities.size(); i++)
        encoder_velocities[i] = ENCODER_POLARITIES[i] * rc_encoder_read_delta(i);

    auto linear_velocity = static_cast<float>(twist.linear.x);
    auto angular_velocity = static_cast<float>(twist.angular.z);

    float left_speed = linear_velocity - angular_velocity * HALF_WHEEL_BASE;
    float right_speed = linear_velocity + angular_velocity * HALF_WHEEL_BASE;

    std::array<float, 2> duty_cycles{};

    
}

auto initialize_hardware() -> void {
    bi_decl(bi_program_description("The main binary for the MBot Pico Board."));
    bi_decl(bi_1pin_with_name(LED_PIN, "On-board LED"));
    set_sys_clock_khz(125000, true);

    stdio_init_all();
    sleep_ms(1500);
    ROS_INFO("MBot Booting Up!");

    ROS_INFO("Initializing motors...");
    rc_motor_init();
    ROS_INFO("Initializing encoders...");
    rc_encoder_init();

    ROS_INFO("Setting heartbeat LED GPIOs...");
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Create topics and register the serialize/deserialize functions
    ROS_INFO("Initializing communications...");
    comms_init_protocol();
    comms_init_topic_data();

    // Launch the other core's comm loop
    ROS_INFO("Starting communications on core 1...");
    multicore_launch_core1(comms_listener_loop);

    // Wait for other core to get rolling
    sleep_ms(500);

//    // Run the main loop as a timed interrupt
//    ROS_INFO("Starting the timed interrupt...");
//    repeating_timer_t loop_timer;
//    add_repeating_timer_ms(MAIN_LOOP_MS, update, nullptr, &loop_timer);

    ROS_INFO("Done Booting Up!");
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "drive_pico");
    ros::NodeHandle nh;

    initialize_hardware();

    std::ignore = nh.subscribe("cmd_vel", 1, twist_callback);

    ros::spin();

    return EXIT_SUCCESS;
}