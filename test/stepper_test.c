#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../src/stepper.h"
#include "../src/planner.h"
#include "../src/hal.h"

/* Mock HAL functions for testing */
static bool mock_motors_enabled = false;
static uint32_t mock_time_us = 0;
static uint32_t mock_time_ms = 0;
static bool mock_dir_state[HAL_AXIS_MAX];
static bool mock_step_pulse_state[HAL_AXIS_MAX];

uint32_t hal_millis(void) {
    return mock_time_ms;
}

uint32_t hal_micros(void) {
    return mock_time_us;
}

void hal_delay_ms(uint32_t ms) {
    mock_time_ms += ms;
    mock_time_us += ms * 1000;
}

void hal_stepper_enable(bool en) {
    mock_motors_enabled = en;
}

void hal_stepper_set_dir(hal_axis_t axis, bool dir_positive) {
    if (axis < HAL_AXIS_MAX) {
        mock_dir_state[axis] = dir_positive;
    }
}

void hal_stepper_step_pulse(hal_axis_t axis) {
    if (axis < HAL_AXIS_MAX) {
        mock_step_pulse_state[axis] = true;
    }
}

void hal_stepper_step_clear(hal_axis_t axis) {
    if (axis < HAL_AXIS_MAX) {
        mock_step_pulse_state[axis] = false;
    }
}

void hal_stepper_pulse_mask(uint32_t axis_mask) {
    (void)axis_mask;
}

/* Mock kinematics */
kin_iface_t g_kin = {0};

static void mock_steps_to_cart(const kin_steps_t *steps, kin_cart_t *out_cart) {
    /* Simple 1:1 mapping for testing */
    for (int i = 0; i < 3 && i < KIN_MAX_CART_AXES; i++) {
        out_cart->v[i] = (float)steps->v[i];
    }
}

void reset_mocks(void) {
    mock_motors_enabled = false;
    mock_time_us = 0;
    mock_time_ms = 0;
    memset(mock_dir_state, 0, sizeof(mock_dir_state));
    memset(mock_step_pulse_state, 0, sizeof(mock_step_pulse_state));
    
    /* Set up minimal kinematics */
    g_kin.cart_axes = 3;
    g_kin.joint_axes = 3;
    g_kin.steps_to_cart = mock_steps_to_cart;
}

/* Test stepper initialization */
void test_stepper_init(void) {
    printf("Testing stepper initialization...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_config_t config = {
        .step_pulse_us = 10,
        .step_idle_delay_us = 100,
        .dir_setup_us = 5,
        .motors_enabled = false,
        .idle_disable = true,
        .idle_timeout_ms = 30000
    };
    
    stepper_init(&ctx, &config);
    
    /* Verify initialization */
    assert(ctx.state == STEPPER_IDLE);
    assert(ctx.config.step_pulse_us == 10);
    assert(ctx.current_block == NULL);
    assert(ctx.current_speed == 0.0f);
    assert(!mock_motors_enabled);
    
    printf("[passed]\n");
}

/* Test stepper initialization with NULL config (default config) */
void test_stepper_init_default(void) {
    printf("Testing stepper initialization with default config...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Verify default configuration is set */
    assert(ctx.state == STEPPER_IDLE);
    assert(ctx.config.step_pulse_us > 0);
    assert(ctx.config.idle_timeout_ms > 0);
    
    printf("[passed]\n");
}

/* Test stepper reset */
void test_stepper_reset(void) {
    printf("Testing stepper reset...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Set some state */
    ctx.state = STEPPER_RUNNING;
    ctx.current_speed = 100.0f;
    
    /* Reset */
    stepper_reset(&ctx);
    
    /* Verify reset state */
    assert(ctx.state == STEPPER_IDLE);
    assert(ctx.current_block == NULL);
    assert(ctx.current_speed == 0.0f);
    
    printf("[passed]\n");
}

/* Test motor enable/disable */
void test_stepper_motors(void) {
    printf("Testing stepper motor enable/disable...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Enable motors */
    stepper_enable_motors(&ctx, true);
    assert(mock_motors_enabled);
    assert(stepper_motors_enabled(&ctx));
    
    /* Disable motors */
    stepper_enable_motors(&ctx, false);
    assert(!mock_motors_enabled);
    assert(!stepper_motors_enabled(&ctx));
    
    printf("[passed]\n");
}

/* Test loading a block */
void test_stepper_load_block(void) {
    printf("Testing stepper load block...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Create a valid planner block */
    planner_block_t block;
    planner_block_init(&block);
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.exit_speed = 50.0f;
    block.acceleration = 500.0f;
    block.millimeters = 10.0f;
    block.step_event_count = 1000;
    block.direction_bits = 0x01;  /* X axis positive */
    
    /* Load the block */
    bool result = stepper_load_block(&ctx, &block);
    
    /* Verify block was loaded */
    assert(result);
    assert(ctx.current_block == &block);
    assert(ctx.state == STEPPER_RUNNING);
    assert(mock_motors_enabled);  /* Motors should be enabled */
    
    printf("[passed]\n");
}

/* Test loading invalid block (NULL) */
void test_stepper_load_null_block(void) {
    printf("Testing stepper load NULL block...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Try to load NULL block */
    bool result = stepper_load_block(&ctx, NULL);
    
    /* Verify load failed */
    assert(!result);
    assert(ctx.current_block == NULL);
    assert(ctx.state == STEPPER_IDLE);
    
    printf("[passed]\n");
}

/* Test stepper state queries */
void test_stepper_queries(void) {
    printf("Testing stepper state queries...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Check idle state */
    assert(stepper_is_idle(&ctx));
    assert(!stepper_is_executing(&ctx));
    assert(stepper_get_state(&ctx) == STEPPER_IDLE);
    
    /* Load a block */
    planner_block_t block;
    planner_block_init(&block);
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.step_event_count = 100;
    
    stepper_load_block(&ctx, &block);
    
    /* Check running state */
    assert(!stepper_is_idle(&ctx));
    assert(stepper_is_executing(&ctx));
    assert(stepper_get_state(&ctx) == STEPPER_RUNNING);
    
    printf("[passed]\n");
}

/* Test hold and resume */
void test_stepper_hold_resume(void) {
    printf("Testing stepper hold and resume...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Load a block */
    planner_block_t block;
    planner_block_init(&block);
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.step_event_count = 100;
    
    stepper_load_block(&ctx, &block);
    assert(ctx.state == STEPPER_RUNNING);
    
    /* Hold */
    stepper_hold(&ctx);
    assert(ctx.state == STEPPER_HOLD);
    
    /* Resume */
    stepper_resume(&ctx);
    assert(ctx.state == STEPPER_RUNNING);
    
    printf("[passed]\n");
}

/* Test stop */
void test_stepper_stop(void) {
    printf("Testing stepper stop...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Load a block */
    planner_block_t block;
    planner_block_init(&block);
    block.entry_speed = 100.0f;
    block.nominal_speed = 200.0f;
    block.step_event_count = 100;
    
    stepper_load_block(&ctx, &block);
    assert(ctx.state == STEPPER_RUNNING);
    
    /* Stop */
    stepper_stop(&ctx);
    assert(ctx.state == STEPPER_STOPPING);
    
    printf("[passed]\n");
}

/* Test get position */
void test_stepper_get_position(void) {
    printf("Testing stepper get position...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Get initial position */
    kin_steps_t pos;
    stepper_get_position(&ctx, &pos);
    
    /* Should be zero initially */
    for (int i = 0; i < KIN_MAX_JOINT_AXES; i++) {
        assert(pos.v[i] == 0);
    }
    
    /* Get Cartesian position */
    kin_cart_t cart;
    stepper_get_cart_position(&ctx, &cart);
    
    /* Should also be zero */
    for (int i = 0; i < KIN_MAX_CART_AXES; i++) {
        assert(cart.v[i] == 0.0f);
    }
    
    printf("[passed]\n");
}

/* Test configuration get/set */
void test_stepper_config(void) {
    printf("Testing stepper configuration get/set...\n");
    reset_mocks();
    
    stepper_context_t ctx;
    stepper_init(&ctx, NULL);
    
    /* Create new config */
    stepper_config_t new_config = {
        .step_pulse_us = 20,
        .step_idle_delay_us = 200,
        .dir_setup_us = 10,
        .motors_enabled = true,
        .idle_disable = false,
        .idle_timeout_ms = 60000
    };
    
    /* Set config */
    stepper_set_config(&ctx, &new_config);
    
    /* Get config and verify */
    stepper_config_t read_config;
    stepper_get_config(&ctx, &read_config);
    
    assert(read_config.step_pulse_us == 20);
    assert(read_config.step_idle_delay_us == 200);
    assert(read_config.dir_setup_us == 10);
    assert(read_config.idle_timeout_ms == 60000);
    
    printf("[passed]\n");
}

int main(void) {
    printf("Running stepper tests...\n\n");
    
    test_stepper_init();
    test_stepper_init_default();
    test_stepper_reset();
    test_stepper_motors();
    test_stepper_load_block();
    test_stepper_load_null_block();
    test_stepper_queries();
    test_stepper_hold_resume();
    test_stepper_stop();
    test_stepper_get_position();
    test_stepper_config();
    
    printf("\nAll stepper tests passed!\n");
    return 0;
}
