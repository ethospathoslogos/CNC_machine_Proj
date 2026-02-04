/* system_state.c - System state management implementation */

#include "system_state.h"
#include <string.h>
#include <stdio.h>

/* Default constants if not defined */
#ifndef GRBL_LINE_QUEUE_DEPTH
#define GRBL_LINE_QUEUE_DEPTH 8u
#endif

#ifndef GRBL_RX_CHUNK
#define GRBL_RX_CHUNK 64u
#endif

/* ----------------------------- Private helpers ----------------------------- */

/* Protocol line callback - can be used with protocol layer */
static void on_line_received(const char *line, void *user) {
    system_context_t *sys = (system_context_t *)user;
    
    /* Process G-code line if system is ready */
    if (sys->state == SYS_STATE_IDLE || sys->state == SYS_STATE_RUNNING) {
        gcode_status_t gcode_st = gcode_process_line(&sys->gcode, line);
        
        if (gcode_st == GCODE_OK) {
            sys->total_lines_processed++;
            if (sys->state == SYS_STATE_IDLE) {
                sys->state = SYS_STATE_RUNNING;
            }
        } else {
            sys->total_errors++;
        }
    } else if (sys->state == SYS_STATE_CHECK) {
        /* In check mode, parse but don't execute */
        gcode_block_t block;
        gcode_parse_line(line, &block);
        sys->total_lines_processed++;
    } else {
        sys->total_errors++;
    }
}

/* Process a G-code line (public function for external use) */
void system_process_line(system_context_t *sys, const char *line) {
    if (!sys || !line) return;
    on_line_received(line, sys);
}

/* ----------------------------- Initialization ----------------------------- */

void system_init(system_context_t *sys) {
    if (!sys) return;
    
    memset(sys, 0, sizeof(*sys));
    
    /* Initialize subsystems */
    gcode_init(&sys->gcode);
    
    /* Initialize planner queue with default depth */
    planner_queue_init(&sys->planner, GRBL_LINE_QUEUE_DEPTH);
    
    /* Set initial state */
    sys->state = SYS_STATE_IDLE;
    sys->alarm = SYS_ALARM_NONE;
    
    /* Default flags */
    sys->homed = false;
    sys->limits_enabled = true;
    sys->soft_limits_enabled = false;
    sys->spindle_enabled = true;
    
    /* Initialize positions */
    sys->machine_x = 0.0f;
    sys->machine_y = 0.0f;
    sys->machine_z = 0.0f;
    
    sys->work_offset_x = 0.0f;
    sys->work_offset_y = 0.0f;
    sys->work_offset_z = 0.0f;
    
    /* Initialize statistics */
    sys->total_lines_processed = 0;
    sys->total_errors = 0;
    sys->uptime_ms = 0;
}

void system_reset(system_context_t *sys) {
    if (!sys) return;
    
    /* Reset subsystems */
    gcode_reset(&sys->gcode);
    planner_queue_clear(&sys->planner);
    
    /* Clear alarm and return to idle */
    sys->state = SYS_STATE_IDLE;
    sys->alarm = SYS_ALARM_NONE;
    
    /* Keep homing state and position (don't clear on soft reset) */
}

void system_poll(system_context_t *sys) {
    if (!sys) return;
    
    /* Update uptime */
    sys->uptime_ms = hal_millis();
    
    /* Poll HAL */
    hal_poll();
    
    /* Check for limit switches if enabled */
    if (sys->limits_enabled && sys->state == SYS_STATE_RUNNING) {
        hal_inputs_t inputs;
        hal_read_inputs(&inputs);
        
        if (inputs.limit_x || inputs.limit_y || inputs.limit_z) {
            system_trigger_alarm(sys, SYS_ALARM_HARD_LIMIT);
        }
        
        if (inputs.estop) {
            system_trigger_alarm(sys, SYS_ALARM_ESTOP);
        }
    }
    
    /* Update machine position from G-code state */
    gcode_get_position(&sys->gcode, &sys->machine_x, &sys->machine_y);
    /* Z axis handling would go here if supported */
}

/* ----------------------------- State management ----------------------------- */

system_state_t system_get_state(const system_context_t *sys) {
    return sys ? sys->state : SYS_STATE_ALARM;
}

bool system_set_state(system_context_t *sys, system_state_t new_state) {
    if (!sys) return false;
    
    /* Validate state transition */
    system_state_t old_state = sys->state;
    
    /* Can't transition out of alarm without explicit clear */
    if (old_state == SYS_STATE_ALARM && new_state != SYS_STATE_IDLE) {
        return false;
    }
    
    /* Some states require specific preconditions */
    switch (new_state) {
        case SYS_STATE_HOMING:
            /* Can only start homing from idle */
            if (old_state != SYS_STATE_IDLE) return false;
            break;
            
        case SYS_STATE_RUNNING:
            /* Can only run from idle or hold */
            if (old_state != SYS_STATE_IDLE && old_state != SYS_STATE_HOLD) {
                return false;
            }
            break;
            
        case SYS_STATE_IDLE:
            /* Most states can transition to idle */
            break;
            
        default:
            break;
    }
    
    sys->state = new_state;
    return true;
}

void system_trigger_alarm(system_context_t *sys, system_alarm_t alarm) {
    if (!sys) return;
    
    sys->state = SYS_STATE_ALARM;
    sys->alarm = alarm;
    
    /* Disable motion immediately */
    hal_stepper_enable(false);
    
    /* Turn off spindle for safety */
    hal_spindle_set(HAL_SPINDLE_OFF, 0.0f);
    
    /* Clear planner queue */
    planner_queue_clear(&sys->planner);
}

bool system_clear_alarm(system_context_t *sys) {
    if (!sys) return false;
    
    if (sys->state != SYS_STATE_ALARM) {
        return false;
    }
    
    sys->alarm = SYS_ALARM_NONE;
    sys->state = SYS_STATE_IDLE;
    
    return true;
}

bool system_is_idle(const system_context_t *sys) {
    return sys && (sys->state == SYS_STATE_IDLE);
}

bool system_is_alarmed(const system_context_t *sys) {
    return sys && (sys->state == SYS_STATE_ALARM);
}

/* ----------------------------- Realtime command handlers ----------------------------- */

void system_feed_hold(system_context_t *sys) {
    if (!sys) return;
    
    if (sys->state == SYS_STATE_RUNNING || sys->state == SYS_STATE_JOG) {
        sys->state = SYS_STATE_HOLD;
        /* In a real implementation, this would decelerate motion smoothly */
    }
}

void system_cycle_start(system_context_t *sys) {
    if (!sys) return;
    
    if (sys->state == SYS_STATE_HOLD) {
        sys->state = SYS_STATE_RUNNING;
        /* In a real implementation, this would resume motion */
    }
}

void system_soft_reset(system_context_t *sys) {
    if (!sys) return;
    
    system_reset(sys);
}

/* ----------------------------- Status reporting ----------------------------- */

size_t system_get_status_report(const system_context_t *sys, char *buf, size_t buf_size) {
    if (!sys || !buf || buf_size == 0) return 0;
    
    /* Generate grbl-style status report: <state|MPos:x,y,z|WPos:x,y,z|F:feed> */
    
    float wpos_x = sys->machine_x - sys->work_offset_x;
    float wpos_y = sys->machine_y - sys->work_offset_y;
    float wpos_z = sys->machine_z - sys->work_offset_z;
    
    float feed = gcode_get_feedrate(&sys->gcode);
    float spindle = gcode_get_spindle_speed(&sys->gcode);
    
    int written = snprintf(buf, buf_size,
        "<%s|MPos:%.3f,%.3f,%.3f|WPos:%.3f,%.3f,%.3f|F:%.1f|S:%.0f",
        system_state_string(sys->state),
        sys->machine_x, sys->machine_y, sys->machine_z,
        wpos_x, wpos_y, wpos_z,
        feed, spindle);
    
    /* Add alarm code if in alarm state */
    if (sys->state == SYS_STATE_ALARM && written > 0 && (size_t)written < buf_size) {
        written += snprintf(buf + written, buf_size - written,
            "|A:%d", sys->alarm);
    }
    
    /* Close status report */
    if (written > 0 && (size_t)written < buf_size) {
        strncat(buf, ">", buf_size - written - 1);
        written++;
    }
    
    return (written > 0) ? (size_t)written : 0;
}

const char *system_state_string(system_state_t state) {
    switch (state) {
        case SYS_STATE_IDLE:    return "Idle";
        case SYS_STATE_RUNNING: return "Run";
        case SYS_STATE_HOLD:    return "Hold";
        case SYS_STATE_JOG:     return "Jog";
        case SYS_STATE_ALARM:   return "Alarm";
        case SYS_STATE_HOMING:  return "Home";
        case SYS_STATE_CHECK:   return "Check";
        case SYS_STATE_SLEEP:   return "Sleep";
        case SYS_STATE_DOOR:    return "Door";
        default:                return "Unknown";
    }
}

const char *system_alarm_string(system_alarm_t alarm) {
    switch (alarm) {
        case SYS_ALARM_NONE:            return "None";
        case SYS_ALARM_HARD_LIMIT:      return "Hard limit triggered";
        case SYS_ALARM_SOFT_LIMIT:      return "Soft limit exceeded";
        case SYS_ALARM_ESTOP:           return "Emergency stop";
        case SYS_ALARM_PROBE_FAIL:      return "Probe cycle failed";
        case SYS_ALARM_HOMING_FAIL:     return "Homing cycle failed";
        case SYS_ALARM_OVERFLOW:        return "Buffer overflow";
        case SYS_ALARM_SPINDLE_STALL:   return "Spindle stall detected";
        default:                        return "Unknown alarm";
    }
}

/* ----------------------------- Position management ----------------------------- */

void system_get_machine_position(const system_context_t *sys, float *x, float *y, float *z) {
    if (!sys) return;
    if (x) *x = sys->machine_x;
    if (y) *y = sys->machine_y;
    if (z) *z = sys->machine_z;
}

void system_get_work_position(const system_context_t *sys, float *x, float *y, float *z) {
    if (!sys) return;
    if (x) *x = sys->machine_x - sys->work_offset_x;
    if (y) *y = sys->machine_y - sys->work_offset_y;
    if (z) *z = sys->machine_z - sys->work_offset_z;
}

void system_set_work_offset(system_context_t *sys, float x, float y, float z) {
    if (!sys) return;
    sys->work_offset_x = x;
    sys->work_offset_y = y;
    sys->work_offset_z = z;
}

/* ----------------------------- Homing ----------------------------- */

bool system_start_homing(system_context_t *sys, uint8_t axis_mask) {
    if (!sys) return false;
    
    /* Can only home from idle state */
    if (sys->state != SYS_STATE_IDLE) {
        return false;
    }
    
    /* Validate homing axes with kinematics if available */
    if (g_kin.validate_homing_axes) {
        if (!g_kin.validate_homing_axes(axis_mask)) {
            return false;
        }
    }
    
    sys->state = SYS_STATE_HOMING;
    
    /* In a real implementation, this would:
     * 1. Move axes to limit switches
     * 2. Back off and re-approach slowly
     * 3. Set machine position to home position
     * 4. Set homed flag
     * For now, just simulate immediate completion
     */
    
    sys->machine_x = 0.0f;
    sys->machine_y = 0.0f;
    sys->machine_z = 0.0f;
    
    sys->homed = true;
    sys->state = SYS_STATE_IDLE;
    
    return true;
}

bool system_is_homed(const system_context_t *sys) {
    return sys && sys->homed;
}

/* ----------------------------- Limits ----------------------------- */

void system_set_limits_enabled(system_context_t *sys, bool enabled) {
    if (!sys) return;
    sys->limits_enabled = enabled;
}

void system_set_soft_limits_enabled(system_context_t *sys, bool enabled) {
    if (!sys) return;
    sys->soft_limits_enabled = enabled;
}

bool system_check_soft_limits(const system_context_t *sys, float x, float y, float z) {
    if (!sys || !sys->soft_limits_enabled) return true;
    
    /* Define soft limits (these would typically come from settings) */
    const float X_MIN = 0.0f, X_MAX = 200.0f;
    const float Y_MIN = 0.0f, Y_MAX = 200.0f;
    const float Z_MIN = -50.0f, Z_MAX = 0.0f;
    
    if (x < X_MIN || x > X_MAX) return false;
    if (y < Y_MIN || y > Y_MAX) return false;
    if (z < Z_MIN || z > Z_MAX) return false;
    
    return true;
}
