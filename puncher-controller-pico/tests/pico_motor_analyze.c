#include <stdio.h>
#include <stdint.h>
#include <rc/motor/motor.h>
#include <rc/encoder/encoder.h>
#include <pico/stdlib.h>
#include <hardware/adc.h>

#define INT_16_MAX 32768
#define ENCODER_RESOLUTION 20.0
#define GEAR_RATIO 78.0
#define TIMESTEP_S 1.5
#define NUM_POINTS 25

#define LEFT_ENC_POL 1
#define RIGHT_ENC_POL -1
#define LEFT_MOTOR_POL 1
#define RIGHT_MOTOR_POL -1

#define WHEEL_DIAMETER 0.08 // diameter of wheel in meters
#define PI 3.141592653589793

void blink();

int main()
{
    const float RPM_conversion_factor = 60 / (GEAR_RATIO * TIMESTEP_S * ENCODER_RESOLUTION);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    rc_motor_init();
    rc_encoder_init();
    blink();
    sleep_ms(1000 * TIMESTEP_S);
    int32_t d = 0;
    int64_t l_cmd, r_cmd;
    int encoder_reading_l, encoder_reading_r;
    float current_reading;
    float wheel_speed_l, wheel_speed_r;
    printf("Left operation only\n");
    printf("\nDuty (PWM),Left Speed (m/s),Right Speed (m/s)\n");
    adc_select_input(0);
    for (float duty = 0.0; duty <= 1.05; duty += 0.04)
    {
        l_cmd = (int)(duty * 0.95 * pow(2, 15));
        r_cmd = (int)(duty * 0.95 * pow(2, 15));
        rc_motor_set(1, LEFT_MOTOR_POL * l_cmd);
        rc_motor_set(3, 0);
        encoder_reading_l = LEFT_ENC_POL * rc_encoder_read_delta(1);
        encoder_reading_r = RIGHT_ENC_POL * rc_encoder_read_delta(3);
        wheel_speed_l = RPM_conversion_factor * encoder_reading_l * WHEEL_DIAMETER * PI / 60; // m/s
        wheel_speed_r = RPM_conversion_factor * encoder_reading_r * WHEEL_DIAMETER * PI / 60; // m/s
        printf("%f,%f,%f\n", (float)duty, wheel_speed_l, wheel_speed_r);
        sleep_ms(1000 * TIMESTEP_S);
    }

    printf("Right operation only\n");
    printf("\nDuty (PWM),Left Speed (m/s),Right Speed (m/s)\n");
    for (float duty = 0.0; duty <= 1.05; duty += 0.04)
    {
        l_cmd = (int)(duty * 0.95 * pow(2, 15));
        r_cmd = (int)(duty * 0.95 * pow(2, 15));
        rc_motor_set(1, 0);
        rc_motor_set(3, RIGHT_MOTOR_POL * r_cmd);
        encoder_reading_l = LEFT_ENC_POL * rc_encoder_read_delta(1);
        encoder_reading_r = RIGHT_ENC_POL * rc_encoder_read_delta(3);
        wheel_speed_l = RPM_conversion_factor * encoder_reading_l * WHEEL_DIAMETER * PI / 60; // m/s
        wheel_speed_r = RPM_conversion_factor * encoder_reading_r * WHEEL_DIAMETER * PI / 60; // m/s
        printf("%f,%f,%f\n", (float)duty, wheel_speed_l, wheel_speed_r);
        sleep_ms(1000 * TIMESTEP_S);
    }
    rc_motor_set(1, 0);
    rc_motor_set(3, 0);

    blink();
    printf("\nDone!\n");

    rc_motor_cleanup();
    blink();
    return 0;
}

void blink()
{
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, false);
}
