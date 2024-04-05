#include <ros/node_handle.h>

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

constexpr uint LED_PIN = 25;
constexpr uint MAIN_LOOP_HZ = 100;
constexpr float MAIN_LOOP_PERIOD_S = 1.0f / MAIN_LOOP_HZ;
constexpr float MAIN_LOOP_MS = MAIN_LOOP_PERIOD_S * 1000;

auto update(repeating_timer_t *rt) -> bool {
    return false;
}

auto init() -> void {
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

    int running = 1;

    // run the main loop as a timed interrupt
    ROS_INFO("Starting the timed interrupt...");
    repeating_timer_t loop_timer;
    add_repeating_timer_ms(MAIN_LOOP_MS, update, nullptr, &loop_timer); // 1000x to convert to ms

    ROS_INFO("Done Booting Up!");
}

auto main(int argc, char **argv) -> int {
    ros::init(argc, argv, "drive_pico");
    ros::NodeHandle nh;

    init();

    ros::spin();

    return EXIT_SUCCESS;
}