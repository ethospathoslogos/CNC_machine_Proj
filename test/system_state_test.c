/* system_state_test.c - Unit tests for system state management */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "../src/system_state.h"

/* Mock HAL functions for testing */
uint32_t hal_millis(void) {
    static uint32_t mock_time = 0;
    return mock_time++;
}

uint32_t hal_micros(void) {
    return hal_millis() * 1000;
}

void hal_delay_ms(uint32_t ms) {
    (void)ms;
}

size_t hal_serial_read(hal_port_t port, uint8_t *dst, size_t cap) {
    (void)port;
    (void)dst;
    (void)cap;
    return 0;
}

size_t hal_serial_write(hal_port_t port, const uint8_t *src, size_t len) {
    (void)port;
    (void)src;
    return len;
}

void hal_poll(void) {
    /* No-op for testing */
}

void hal_stepper_enable(bool en) {
    (void)en;
}

void hal_spindle_set(hal_spindle_dir_t dir, float pwm) {
    (void)dir;
    (void)pwm;
}

void hal_read_inputs(hal_inputs_t *out) {
    if (out) {
        memset(out, 0, sizeof(*out));
    }
}

/* Mock kinematics interface */
kin_iface_t g_kin = {0};

/* ----------------------------- Tests ----------------------------- */

void test_system_init() {
    printf("Testing system_init...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    assert(sys.state == SYS_STATE_IDLE);
    assert(sys.alarm == SYS_ALARM_NONE);
    assert(sys.homed == false);
    assert(sys.limits_enabled == true);
    assert(sys.total_lines_processed == 0);
    assert(sys.total_errors == 0);
    
    printf("  [PASSED]\n");
}

void test_state_transitions() {
    printf("Testing state transitions...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Test valid transition: IDLE -> RUNNING */
    assert(system_set_state(&sys, SYS_STATE_RUNNING) == true);
    assert(sys.state == SYS_STATE_RUNNING);
    
    /* Test valid transition: RUNNING -> HOLD */
    assert(system_set_state(&sys, SYS_STATE_HOLD) == true);
    assert(sys.state == SYS_STATE_HOLD);
    
    /* Test valid transition: HOLD -> IDLE */
    assert(system_set_state(&sys, SYS_STATE_IDLE) == true);
    assert(sys.state == SYS_STATE_IDLE);
    
    printf("  [PASSED]\n");
}

void test_alarm_handling() {
    printf("Testing alarm handling...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Trigger an alarm */
    system_trigger_alarm(&sys, SYS_ALARM_HARD_LIMIT);
    
    assert(sys.state == SYS_STATE_ALARM);
    assert(sys.alarm == SYS_ALARM_HARD_LIMIT);
    assert(system_is_alarmed(&sys) == true);
    
    /* Try to transition to running (should fail) */
    assert(system_set_state(&sys, SYS_STATE_RUNNING) == false);
    assert(sys.state == SYS_STATE_ALARM);
    
    /* Clear alarm */
    assert(system_clear_alarm(&sys) == true);
    assert(sys.state == SYS_STATE_IDLE);
    assert(sys.alarm == SYS_ALARM_NONE);
    assert(system_is_alarmed(&sys) == false);
    
    printf("  [PASSED]\n");
}

void test_feed_hold_and_resume() {
    printf("Testing feed hold and cycle start...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Start running */
    system_set_state(&sys, SYS_STATE_RUNNING);
    
    /* Feed hold */
    system_feed_hold(&sys);
    assert(sys.state == SYS_STATE_HOLD);
    
    /* Cycle start (resume) */
    system_cycle_start(&sys);
    assert(sys.state == SYS_STATE_RUNNING);
    
    printf("  [PASSED]\n");
}

void test_soft_reset() {
    printf("Testing soft reset...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Modify some state */
    sys.total_lines_processed = 100;
    sys.total_errors = 5;
    system_set_state(&sys, SYS_STATE_RUNNING);
    
    /* Soft reset */
    system_soft_reset(&sys);
    
    assert(sys.state == SYS_STATE_IDLE);
    assert(sys.alarm == SYS_ALARM_NONE);
    /* Statistics should be preserved or reset depending on implementation */
    
    printf("  [PASSED]\n");
}

void test_position_management() {
    printf("Testing position management...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Set machine position */
    sys.machine_x = 10.0f;
    sys.machine_y = 20.0f;
    sys.machine_z = 5.0f;
    
    /* Get machine position */
    float mx, my, mz;
    system_get_machine_position(&sys, &mx, &my, &mz);
    assert(mx == 10.0f);
    assert(my == 20.0f);
    assert(mz == 5.0f);
    
    /* Set work offset */
    system_set_work_offset(&sys, 5.0f, 10.0f, 2.0f);
    
    /* Get work position */
    float wx, wy, wz;
    system_get_work_position(&sys, &wx, &wy, &wz);
    assert(wx == 5.0f);  /* 10.0 - 5.0 */
    assert(wy == 10.0f); /* 20.0 - 10.0 */
    assert(wz == 3.0f);  /* 5.0 - 2.0 */
    
    printf("  [PASSED]\n");
}

void test_status_report() {
    printf("Testing status report generation...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Set some state */
    sys.state = SYS_STATE_RUNNING;
    sys.machine_x = 10.5f;
    sys.machine_y = 20.3f;
    sys.machine_z = 0.0f;
    
    /* Generate status report */
    char buf[256];
    size_t len = system_get_status_report(&sys, buf, sizeof(buf));
    
    assert(len > 0);
    assert(strstr(buf, "Run") != NULL);
    assert(strstr(buf, "MPos") != NULL);
    assert(strstr(buf, "WPos") != NULL);
    
    printf("  Status: %s\n", buf);
    printf("  [PASSED]\n");
}

void test_state_string_conversion() {
    printf("Testing state string conversion...\n");
    
    assert(strcmp(system_state_string(SYS_STATE_IDLE), "Idle") == 0);
    assert(strcmp(system_state_string(SYS_STATE_RUNNING), "Run") == 0);
    assert(strcmp(system_state_string(SYS_STATE_ALARM), "Alarm") == 0);
    assert(strcmp(system_state_string(SYS_STATE_HOMING), "Home") == 0);
    
    assert(strcmp(system_alarm_string(SYS_ALARM_NONE), "None") == 0);
    assert(strcmp(system_alarm_string(SYS_ALARM_HARD_LIMIT), "Hard limit triggered") == 0);
    assert(strcmp(system_alarm_string(SYS_ALARM_ESTOP), "Emergency stop") == 0);
    
    printf("  [PASSED]\n");
}

void test_homing() {
    printf("Testing homing...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    assert(system_is_homed(&sys) == false);
    
    /* Start homing (simplified - immediate in test) */
    bool result = system_start_homing(&sys, 0x03); /* X and Y axes */
    assert(result == true);
    
    /* Check homed flag */
    assert(system_is_homed(&sys) == true);
    assert(sys.state == SYS_STATE_IDLE); /* Should return to idle after homing */
    
    printf("  [PASSED]\n");
}

void test_soft_limits() {
    printf("Testing soft limits...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    /* Enable soft limits */
    system_set_soft_limits_enabled(&sys, true);
    assert(sys.soft_limits_enabled == true);
    
    /* Check valid position */
    assert(system_check_soft_limits(&sys, 100.0f, 100.0f, -10.0f) == true);
    
    /* Check invalid positions */
    assert(system_check_soft_limits(&sys, -10.0f, 100.0f, -10.0f) == false); /* X too low */
    assert(system_check_soft_limits(&sys, 300.0f, 100.0f, -10.0f) == false); /* X too high */
    assert(system_check_soft_limits(&sys, 100.0f, -10.0f, -10.0f) == false); /* Y too low */
    
    /* Disable soft limits */
    system_set_soft_limits_enabled(&sys, false);
    assert(system_check_soft_limits(&sys, -10.0f, 100.0f, -10.0f) == true); /* Should pass now */
    
    printf("  [PASSED]\n");
}

void test_is_idle() {
    printf("Testing system_is_idle...\n");
    
    system_context_t sys;
    system_init(&sys);
    
    assert(system_is_idle(&sys) == true);
    
    system_set_state(&sys, SYS_STATE_RUNNING);
    assert(system_is_idle(&sys) == false);
    
    system_set_state(&sys, SYS_STATE_IDLE);
    assert(system_is_idle(&sys) == true);
    
    printf("  [PASSED]\n");
}

/* ----------------------------- Main test runner ----------------------------- */

int main(void) {
    printf("=== System State Tests ===\n\n");
    
    test_system_init();
    test_state_transitions();
    test_alarm_handling();
    test_feed_hold_and_resume();
    test_soft_reset();
    test_position_management();
    test_status_report();
    test_state_string_conversion();
    test_homing();
    test_soft_limits();
    test_is_idle();
    
    printf("\n=== All tests passed! ===\n");
    return 0;
}
