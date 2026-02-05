/* stepper.h - Stepper motor control and step generation
 *
 * Purpose:
 *  - Execute motion blocks from the planner
 *  - Generate step pulses for stepper motors
 *  - Interface with HAL for motor control
 *  - Convert between Cartesian coordinates and motor steps
 *
 * Design:
 *  - Consumes planner blocks containing motion commands
 *  - Uses kinematics to convert positions to joint/step space
 *  - Uses HAL functions to control physical stepper motors
 *  - Manages step timing and direction control
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "planner.h"
#include "kinematics.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- Stepper state ----------------------------- */

/* Stepper execution state */
typedef enum {
    STEPPER_IDLE = 0,       /* No motion in progress */
    STEPPER_RUNNING,        /* Executing a motion block */
    STEPPER_HOLD,           /* Motion paused (feed hold) */
    STEPPER_STOPPING,       /* Decelerating to stop */
} stepper_state_t;

/* Stepper configuration */
typedef struct {
    /* Timing parameters */
    uint32_t step_pulse_us;       /* Step pulse duration in microseconds */
    uint32_t step_idle_delay_us;  /* Delay between steps in microseconds */
    
    /* Direction setup time */
    uint32_t dir_setup_us;        /* Time to wait after setting direction */
    
    /* Enable/disable control */
    bool motors_enabled;          /* Motors are enabled */
    bool idle_disable;            /* Disable motors when idle */
    uint32_t idle_timeout_ms;     /* Time before disabling motors when idle */
} stepper_config_t;

/* Current stepper execution context */
typedef struct {
    /* Current state */
    stepper_state_t state;
    
    /* Configuration */
    stepper_config_t config;
    
    /* Current block being executed */
    planner_block_t *current_block;
    
    /* Step counters for current block */
    uint32_t step_count[HAL_AXIS_MAX];  /* Steps taken per axis */
    uint32_t target_steps[HAL_AXIS_MAX]; /* Target steps per axis */
    
    /* Current position in steps */
    kin_steps_t position;
    
    /* Timing */
    uint32_t last_step_time_us;   /* Time of last step pulse */
    uint32_t step_interval_us;    /* Current interval between steps */
    
    /* Speed tracking */
    float current_speed;          /* Current speed in mm/min */
    
    /* Idle tracking */
    uint32_t idle_start_time_ms;  /* Time when idle state started */
} stepper_context_t;

/* ----------------------------- Public API ----------------------------- */

/* Initialize the stepper subsystem */
void stepper_init(stepper_context_t *ctx, const stepper_config_t *config);

/* Reset stepper to safe state */
void stepper_reset(stepper_context_t *ctx);

/* Start executing a new block from the planner */
bool stepper_load_block(stepper_context_t *ctx, planner_block_t *block);

/* Update stepper state - call frequently from main loop or timer ISR */
void stepper_update(stepper_context_t *ctx);

/* ----------------------------- Motion control ----------------------------- */

/* Enable/disable stepper motors */
void stepper_enable_motors(stepper_context_t *ctx, bool enable);

/* Check if motors are enabled */
bool stepper_motors_enabled(const stepper_context_t *ctx);

/* Pause motion (feed hold) */
void stepper_hold(stepper_context_t *ctx);

/* Resume motion from hold */
void stepper_resume(stepper_context_t *ctx);

/* Stop motion immediately */
void stepper_stop(stepper_context_t *ctx);

/* ----------------------------- Status queries ----------------------------- */

/* Get current stepper state */
stepper_state_t stepper_get_state(const stepper_context_t *ctx);

/* Check if stepper is idle */
bool stepper_is_idle(const stepper_context_t *ctx);

/* Check if a block is currently being executed */
bool stepper_is_executing(const stepper_context_t *ctx);

/* Get current position in steps */
void stepper_get_position(const stepper_context_t *ctx, kin_steps_t *out_pos);

/* Get current position in Cartesian coordinates */
void stepper_get_cart_position(const stepper_context_t *ctx, kin_cart_t *out_cart);

/* ----------------------------- Configuration ----------------------------- */

/* Update stepper configuration */
void stepper_set_config(stepper_context_t *ctx, const stepper_config_t *config);

/* Get current configuration */
void stepper_get_config(const stepper_context_t *ctx, stepper_config_t *out_config);

#ifdef __cplusplus
}
#endif
