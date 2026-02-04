/* hal_mock.c - Mock HAL implementation for examples and testing
 *
 * This provides minimal HAL function implementations for standalone
 * examples and tests that don't run on real hardware.
 */

#include "hal.h"
#include <string.h>

/* Mock HAL functions */
uint32_t hal_millis(void) {
    static uint32_t mock_time = 0;
    return mock_time++;
}

uint32_t hal_micros(void) {
    return hal_millis() * 1000;
}

void hal_delay_ms(uint32_t ms) {
    (void)ms;
    /* No-op in mock */
}

size_t hal_serial_read(hal_port_t port, uint8_t *dst, size_t cap) {
    (void)port;
    (void)dst;
    (void)cap;
    return 0;  /* No data available in mock */
}

size_t hal_serial_write(hal_port_t port, const uint8_t *src, size_t len) {
    (void)port;
    (void)src;
    return len;  /* Pretend all bytes were written */
}

size_t hal_serial_write_str(hal_port_t port, const char *s) {
    (void)port;
    (void)s;
    return 0;
}

void hal_gpio_write(uint32_t pin_id, hal_pin_state_t state) {
    (void)pin_id;
    (void)state;
}

hal_pin_state_t hal_gpio_read(uint32_t pin_id) {
    (void)pin_id;
    return HAL_PIN_LOW;
}

void hal_poll(void) {
    /* No-op in mock */
}

void hal_stepper_enable(bool en) {
    (void)en;
}

void hal_stepper_set_dir(hal_axis_t axis, bool dir_positive) {
    (void)axis;
    (void)dir_positive;
}

void hal_stepper_step_pulse(hal_axis_t axis) {
    (void)axis;
}

void hal_stepper_step_clear(hal_axis_t axis) {
    (void)axis;
}

void hal_stepper_pulse_mask(uint32_t axis_mask) {
    (void)axis_mask;
}

void hal_spindle_set(hal_spindle_dir_t dir, float pwm) {
    (void)dir;
    (void)pwm;
}

void hal_coolant_mist(bool on) {
    (void)on;
}

void hal_coolant_flood(bool on) {
    (void)on;
}

void hal_read_inputs(hal_inputs_t *out) {
    if (out) {
        memset(out, 0, sizeof(*out));
    }
}

void hal_tick_1khz_isr(void) {
    /* No-op in mock */
}

hal_status_t hal_init(void) {
    return HAL_OK;
}

void hal_start(void) {
    /* No-op in mock */
}

void hal_deinit(void) {
    /* No-op in mock */
}
