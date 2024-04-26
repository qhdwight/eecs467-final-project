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

#define SHOOTER_MOTOR_POL -1

//#define SHOOT_ROLLER_TIME_MS 100 // Bot 0
#define SHOOT_ROLLER_TIME_MS 110 // Bot 1

#define ROLLER_MOTOR_CHANNEL 3
#define SOLENOID_CHANNEL 1

float enc2meters = ((2.0 * M_PI * WHEEL_RADIUS) / (GEAR_RATIO * ENCODER_RES));

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
        mbot_ball_command_t ball_command;
        stdio_usb_in_chars_itf(1, (char*)(&ball_command), sizeof(ball_command));

        // Read footer 0xEF
        stdio_usb_in_chars_itf(1, &sync_byte, 1);
        if (sync_byte != 0xEF)
        {
            printf("Failed receiving footer\n");
            continue;
        }

        // Should have received good data so update motor command
        printf("Received new ball command: intake=%u, shoot=%,03f\n", ball_command.intake, ball_command.shoot_ball);
        if (ball_command.shoot_ball) {
            // Perform shoot ball sequence
            int32_t top_roller_duty = -pow(2, 15);
            rc_motor_set(ROLLER_MOTOR_CHANNEL, top_roller_duty * SHOOTER_MOTOR_POL);
            sleep_ms(SHOOT_ROLLER_TIME_MS);
            rc_motor_set(SOLENOID_CHANNEL, pow(2, 15) - 1);
            sleep_ms(200);
            rc_motor_set(ROLLER_MOTOR_CHANNEL, 0);
            rc_motor_set(SOLENOID_CHANNEL, 0);
        }
        else {
            // Set top roller
            int32_t duty_cyle = ball_command.intake * pow(2, 15) - 1;
            rc_motor_set(ROLLER_MOTOR_CHANNEL, duty_cyle * SHOOTER_MOTOR_POL);
        }

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
    multicore_launch_core1(comms_listener_loop);

    // wait for other core to get rolling
    sleep_ms(500);

    printf("Done Booting Up!\n\n");

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

