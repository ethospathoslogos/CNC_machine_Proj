/* stepper.c - Stepper motor control implementation */

#include "stepper.h"
#include "system_state.h"
#include <string.h>
#include <math.h>

/* Constants */
#define DEFAULT_STEP_INTERVAL_US 1000  /* Default step interval: 1ms */

/* Internal helper functions */

/* Delay for a specified number of microseconds 
 * Note: This implementation uses hal_delay_ms with rounding up to 1ms minimum
 * For sub-millisecond precision, a platform-specific implementation would be needed
 */
static void delay_us(uint32_t us) {
    if (us == 0) return;
    
    /* Convert to milliseconds, rounding up */
    uint32_t ms = (us + 999) / 1000;
    if (ms == 0) ms = 1;  /* Minimum 1ms delay */
    
    hal_delay_ms(ms);
}

/* Convert planner block to step counts using kinematics */
static bool block_to_steps(const planner_block_t *block, 
                          uint32_t *out_steps,
                          uint8_t *out_dir_bits) {
    if (!block || !out_steps || !out_dir_bits) {
        return false;
    }
    
    /* For now, we'll use the step_event_count from the planner block */
    /* In a real implementation, this would use kinematics to convert
     * the target position to steps for each axis */
    
    /* Initialize all step counts to zero */
    for (uint8_t i = 0; i < HAL_AXIS_MAX; i++) {
        out_steps[i] = 0;
    }
    
    /* Use the step_event_count and direction_bits from the block */
    /* This is simplified - real implementation would use kinematics */
    *out_dir_bits = block->direction_bits;
    
    /* Distribute steps across axes based on direction bits */
    /* This is a simplified approach */
    if (block->step_event_count > 0) {
        /* For simplicity, put all steps on X axis if direction bit 0 is set */
        /* Real implementation would calculate per-axis steps properly */
        out_steps[HAL_AXIS_X] = block->step_event_count;
    }
    
    return true;
}

/* Set direction pins for all axes */
static void set_directions(uint8_t dir_bits) {
    for (hal_axis_t axis = HAL_AXIS_X; axis < HAL_AXIS_MAX; axis++) {
        bool dir_positive = (dir_bits & (1 << axis)) != 0;
        hal_stepper_set_dir(axis, dir_positive);
    }
}

/* Clear step pulses for all axes */
static void clear_step_pulses(void) {
    for (hal_axis_t axis = HAL_AXIS_X; axis < HAL_AXIS_MAX; axis++) {
        hal_stepper_step_clear(axis);
    }
}

/* ----------------------------- Public API Implementation ----------------------------- */

void stepper_init(stepper_context_t *ctx, const stepper_config_t *config) {
    if (!ctx) {
        return;
    }
    
    /* Clear context */
    memset(ctx, 0, sizeof(stepper_context_t));
    
    /* Set initial state */
    ctx->state = STEPPER_IDLE;
    
    /* Copy configuration */
    if (config) {
        ctx->config = *config;
    } else {
        /* Set default configuration */
        ctx->config.step_pulse_us = 10;        /* 10 microsecond pulse */
        ctx->config.step_idle_delay_us = 100;  /* 100 us between steps */
        ctx->config.dir_setup_us = 5;          /* 5 us direction setup */
        ctx->config.motors_enabled = false;
        ctx->config.idle_disable = true;
        ctx->config.idle_timeout_ms = 30000;   /* 30 second timeout */
    }
    
    /* Initialize position to zero */
    memset(&ctx->position, 0, sizeof(kin_steps_t));
    
    /* Disable motors initially */
    hal_stepper_enable(false);
}

void stepper_reset(stepper_context_t *ctx) {
    if (!ctx) {
        return;
    }
    
    /* Stop any current motion */
    ctx->state = STEPPER_IDLE;
    ctx->current_block = NULL;
    
    /* Clear step counters */
    memset(ctx->step_count, 0, sizeof(ctx->step_count));
    memset(ctx->target_steps, 0, sizeof(ctx->target_steps));
    
    /* Reset speed */
    ctx->current_speed = 0.0f;
    
    /* Clear all step pulses */
    clear_step_pulses();
    
    /* Disable motors if configured */
    if (ctx->config.idle_disable) {
        hal_stepper_enable(false);
        ctx->config.motors_enabled = false;
    }
}

bool stepper_load_block(stepper_context_t *ctx, planner_block_t *block) {
    if (!ctx || !block) {
        return false;
    }
    
    /* Can't load a new block if not idle */
    if (ctx->state != STEPPER_IDLE) {
        return false;
    }
    
    /* Validate the block */
    if (!planner_block_validate(block)) {
        return false;
    }
    
    /* Convert block to step counts */
    uint8_t dir_bits = 0;
    if (!block_to_steps(block, ctx->target_steps, &dir_bits)) {
        return false;
    }
    
    /* Set the block */
    ctx->current_block = block;
    
    /* Reset step counters */
    memset(ctx->step_count, 0, sizeof(ctx->step_count));
    
    /* Set direction pins */
    set_directions(dir_bits);
    
    /* Wait for direction setup time */
    delay_us(ctx->config.dir_setup_us);
    
    /* Calculate initial step interval from entry speed */
    /* Convert speed from mm/min to steps/s, then to interval in us */
    if (block->entry_speed > 0.0f) {
        /* Simplified calculation - assumes 1:1 mm to steps */
        float steps_per_sec = block->entry_speed / 60.0f;
        if (steps_per_sec > 0.0f) {
            ctx->step_interval_us = (uint32_t)(1000000.0f / steps_per_sec);
        } else {
            ctx->step_interval_us = DEFAULT_STEP_INTERVAL_US;
        }
    } else {
        ctx->step_interval_us = DEFAULT_STEP_INTERVAL_US;
    }
    
    ctx->current_speed = block->entry_speed;
    
    /* Enable motors if not already enabled */
    if (!ctx->config.motors_enabled) {
        hal_stepper_enable(true);
        ctx->config.motors_enabled = true;
    }
    
    /* Start executing */
    ctx->state = STEPPER_RUNNING;
    ctx->last_step_time_us = hal_micros();
    
    return true;
}

void stepper_update(stepper_context_t *ctx) {
    if (!ctx) {
        return;
    }
    
    uint32_t now_us = hal_micros();
    
    switch (ctx->state) {
        case STEPPER_IDLE:
            /* Check idle timeout for motor disable */
            if (ctx->config.idle_disable && ctx->config.motors_enabled) {
                uint32_t now_ms = hal_millis();
                if (ctx->idle_start_time_ms > 0) {
                    uint32_t idle_time = now_ms - ctx->idle_start_time_ms;
                    if (idle_time >= ctx->config.idle_timeout_ms) {
                        hal_stepper_enable(false);
                        ctx->config.motors_enabled = false;
                    }
                }
            }
            break;
            
        case STEPPER_RUNNING:
            /* Check if it's time for the next step */
            if (now_us - ctx->last_step_time_us >= ctx->step_interval_us) {
                bool steps_remaining = false;
                
                /* Generate step pulses for axes that need them */
                for (hal_axis_t axis = HAL_AXIS_X; axis < HAL_AXIS_MAX; axis++) {
                    if (ctx->step_count[axis] < ctx->target_steps[axis]) {
                        hal_stepper_step_pulse(axis);
                        ctx->step_count[axis]++;
                        steps_remaining = true;
                        
                        /* Update position */
                        if (ctx->current_block && 
                            (ctx->current_block->direction_bits & (1 << axis))) {
                            ctx->position.v[axis]++;
                        } else {
                            ctx->position.v[axis]--;
                        }
                    }
                }
                
                /* Wait for pulse duration */
                delay_us(ctx->config.step_pulse_us);
                
                /* Clear step pulses */
                clear_step_pulses();
                
                /* Update last step time */
                ctx->last_step_time_us = now_us;
                
                /* Check if block is complete */
                if (!steps_remaining) {
                    /* Block finished */
                    ctx->current_block = NULL;
                    ctx->state = STEPPER_IDLE;
                    ctx->current_speed = 0.0f;
                    ctx->idle_start_time_ms = hal_millis();
                }
            }
            break;
            
        case STEPPER_HOLD:
            /* Motion paused - do nothing */
            break;
            
        case STEPPER_STOPPING:
            /* Decelerate and stop */
            ctx->state = STEPPER_IDLE;
            ctx->current_block = NULL;
            ctx->current_speed = 0.0f;
            clear_step_pulses();
            ctx->idle_start_time_ms = hal_millis();
            break;
    }
}

/* ----------------------------- Motion control ----------------------------- */

void stepper_enable_motors(stepper_context_t *ctx, bool enable) {
    if (!ctx) {
        return;
    }
    
    hal_stepper_enable(enable);
    ctx->config.motors_enabled = enable;
    
    if (!enable) {
        /* Motors disabled - ensure we're not running */
        if (ctx->state != STEPPER_IDLE) {
            stepper_stop(ctx);
        }
    }
}

bool stepper_motors_enabled(const stepper_context_t *ctx) {
    if (!ctx) {
        return false;
    }
    return ctx->config.motors_enabled;
}

void stepper_hold(stepper_context_t *ctx) {
    if (!ctx) {
        return;
    }
    
    if (ctx->state == STEPPER_RUNNING) {
        ctx->state = STEPPER_HOLD;
    }
}

void stepper_resume(stepper_context_t *ctx) {
    if (!ctx) {
        return;
    }
    
    if (ctx->state == STEPPER_HOLD) {
        ctx->state = STEPPER_RUNNING;
        ctx->last_step_time_us = hal_micros();
    }
}

void stepper_stop(stepper_context_t *ctx) {
    if (!ctx) {
        return;
    }
    
    ctx->state = STEPPER_STOPPING;
}

/* ----------------------------- Status queries ----------------------------- */

stepper_state_t stepper_get_state(const stepper_context_t *ctx) {
    if (!ctx) {
        return STEPPER_IDLE;
    }
    return ctx->state;
}

bool stepper_is_idle(const stepper_context_t *ctx) {
    if (!ctx) {
        return true;
    }
    return ctx->state == STEPPER_IDLE;
}

bool stepper_is_executing(const stepper_context_t *ctx) {
    if (!ctx) {
        return false;
    }
    return ctx->current_block != NULL;
}

void stepper_get_position(const stepper_context_t *ctx, kin_steps_t *out_pos) {
    if (!ctx || !out_pos) {
        return;
    }
    *out_pos = ctx->position;
}

void stepper_get_cart_position(const stepper_context_t *ctx, kin_cart_t *out_cart) {
    if (!ctx || !out_cart) {
        return;
    }
    
    /* Use kinematics to convert steps to Cartesian coordinates */
    if (g_kin.steps_to_cart) {
        g_kin.steps_to_cart(&ctx->position, out_cart);
    } else {
        /* Fallback: zero position */
        memset(out_cart, 0, sizeof(kin_cart_t));
    }
}

/* ----------------------------- Configuration ----------------------------- */

void stepper_set_config(stepper_context_t *ctx, const stepper_config_t *config) {
    if (!ctx || !config) {
        return;
    }
    
    ctx->config = *config;
}

void stepper_get_config(const stepper_context_t *ctx, stepper_config_t *out_config) {
    if (!ctx || !out_config) {
        return;
    }
    
    *out_config = ctx->config;
}
