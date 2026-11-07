/* EECS149 Buckler / nRF52832 firmware.
 *
 * Default build: LED-only heartbeat. Motor/servo outputs are COMPILE-TIME
 * gated until the RSLK-Buckler wiring and Romi arm calibration are verified.
 *
 * Optional gated diagnostic: press BUCKLER_BUTTON0 to rotate wheels at low
 * duty while chassis is on blocks. This is NOT autonomous navigation.
 */
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "app_error.h"
#include "buckler.h"
#include "robot_wiring.h"

#if ROBOT_WIRING_VERIFIED
#include "app_pwm.h"

APP_PWM_INSTANCE(motor_pwm, 1);
#if ROBOT_ARM_CALIBRATED
APP_PWM_INSTANCE(arm_pwm01, 2);
APP_PWM_INSTANCE(arm_pwm2, 3);
#endif

static uint8_t clamp_duty(int16_t requested) {
    int16_t magnitude = requested < 0 ? -requested : requested;
    return (uint8_t)(magnitude > ROBOT_MAX_TEST_DUTY_PERCENT
            ? ROBOT_MAX_TEST_DUTY_PERCENT : magnitude);
}

static void set_channel(const app_pwm_t *pwm, uint8_t channel, uint8_t duty) {
    ret_code_t rc;
    do {
        rc = app_pwm_channel_duty_set(pwm, channel, duty);
    } while (rc == NRF_ERROR_BUSY);
    APP_ERROR_CHECK(rc);
}

static void motor_set(int16_t left_percent, int16_t right_percent) {
    /* Phase is board-specific. Confirm both wheel directions empirically. */
    nrf_gpio_pin_write(ROBOT_LEFT_DIR, left_percent < 0);
    nrf_gpio_pin_write(ROBOT_RIGHT_DIR, right_percent < 0);
    set_channel(&motor_pwm, 0, clamp_duty(left_percent));
    set_channel(&motor_pwm, 1, clamp_duty(right_percent));
}

static void motor_stop(void) { motor_set(0, 0); }

static void motor_init(void) {
    /* Keep drivers asleep until outputs are initialized and zeroed. */
    nrf_gpio_cfg_output(ROBOT_LEFT_SLEEP);
    nrf_gpio_cfg_output(ROBOT_RIGHT_SLEEP);
    nrf_gpio_pin_clear(ROBOT_LEFT_SLEEP);
    nrf_gpio_pin_clear(ROBOT_RIGHT_SLEEP);
    nrf_gpio_cfg_output(ROBOT_LEFT_DIR);
    nrf_gpio_cfg_output(ROBOT_RIGHT_DIR);
    nrf_gpio_pin_clear(ROBOT_LEFT_DIR);
    nrf_gpio_pin_clear(ROBOT_RIGHT_DIR);
    app_pwm_config_t cfg = APP_PWM_DEFAULT_CONFIG_2CH(
        1000L, ROBOT_LEFT_PWM, ROBOT_RIGHT_PWM);
    cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    APP_ERROR_CHECK(app_pwm_init(&motor_pwm, &cfg, NULL));
    app_pwm_enable(&motor_pwm);
    motor_stop();
    nrf_gpio_pin_set(ROBOT_LEFT_SLEEP);
    nrf_gpio_pin_set(ROBOT_RIGHT_SLEEP);
}

#if ROBOT_ARM_CALIBRATED
static uint16_t clamp_pulse(uint16_t us) {
    if (us < ROBOT_SERVO_MIN_US) return ROBOT_SERVO_MIN_US;
    if (us > ROBOT_SERVO_MAX_US) return ROBOT_SERVO_MAX_US;
    return us;
}

static void servo_pulse(const app_pwm_t *pwm, uint8_t channel, uint16_t us) {
    uint32_t period_ticks = app_pwm_cycle_ticks_get(pwm);
    uint16_t width_ticks = (uint16_t)((period_ticks * clamp_pulse(us)) / 20000U);
    ret_code_t rc;
    do {
        rc = app_pwm_channel_duty_ticks_set(pwm, channel, width_ticks);
    } while (rc == NRF_ERROR_BUSY);
    APP_ERROR_CHECK(rc);
}

static void arm_set(uint16_t lift_us, uint16_t tilt_us, uint16_t grip_us) {
    servo_pulse(&arm_pwm01, 0, lift_us);
    servo_pulse(&arm_pwm01, 1, tilt_us);
    servo_pulse(&arm_pwm2, 0, grip_us);
}

static void arm_init(void) {
    app_pwm_config_t cfg01 = APP_PWM_DEFAULT_CONFIG_2CH(
        20000L, ROBOT_SERVO_LIFT, ROBOT_SERVO_TILT);
    cfg01.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    cfg01.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    app_pwm_config_t cfg2 = APP_PWM_DEFAULT_CONFIG_1CH(
        20000L, ROBOT_SERVO_GRIP);
    cfg2.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    APP_ERROR_CHECK(app_pwm_init(&arm_pwm01, &cfg01, NULL));
    APP_ERROR_CHECK(app_pwm_init(&arm_pwm2, &cfg2, NULL));
    app_pwm_enable(&arm_pwm01);
    app_pwm_enable(&arm_pwm2);
    /* Neutral signals are not guaranteed to be safe mechanically: calibrate. */
    arm_set(1500, 1500, 1500);
}
#endif

static void button_test(void) {
    /* Open-loop diagnostic ONLY. No encoder, obstacle, or emergency-stop logic. */
    motor_set(ROBOT_MAX_TEST_DUTY_PERCENT, ROBOT_MAX_TEST_DUTY_PERCENT);
    nrf_delay_ms(250);
    motor_stop();
}
#endif

int main(void) {
    const uint32_t leds[3] = {BUCKLER_LED0, BUCKLER_LED1, BUCKLER_LED2};
    if (!nrfx_gpiote_is_init()) {
        APP_ERROR_CHECK(nrfx_gpiote_init());
    }
    const nrfx_gpiote_out_config_t led_cfg = NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    for (uint32_t i = 0; i < 3; ++i) {
        /* Use the vendor-supported Buckler LED initialization path. */
        APP_ERROR_CHECK(nrfx_gpiote_out_init(leds[i], &led_cfg));
    }
#if ROBOT_WIRING_VERIFIED
    motor_init();
#if ROBOT_ARM_CALIBRATED
    arm_init();
#endif
#if ROBOT_ALLOW_BUTTON_DEMO
    nrf_gpio_cfg_input(BUCKLER_BUTTON0, NRF_GPIO_PIN_PULLUP);
#endif
#endif
    uint32_t led_index = 0;
    while (1) {
        nrf_gpio_pin_clear(leds[led_index]);
        nrf_delay_ms(150);
        nrf_gpio_pin_set(leds[led_index]);
        led_index = (led_index + 1) % 3;
#if ROBOT_WIRING_VERIFIED && ROBOT_ALLOW_BUTTON_DEMO
        if (nrf_gpio_pin_read(BUCKLER_BUTTON0) == 0) {
            button_test();
            while (nrf_gpio_pin_read(BUCKLER_BUTTON0) == 0) {
                nrf_delay_ms(30);
            }
        }
#endif
    }
}
