/* system_state.h - System state management for CNC machine
 *
 * Purpose:
 *  - Track overall system state (idle, running, alarm, etc.)
 *  - Coordinate between protocol, gcode, planner, and HAL layers
 *  - Generate status reports
 *  - Handle state transitions and safety interlocks
 *
 * Design goals: single source of truth for machine state, safety-critical
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "gcode.h"
#include "planner.h"
#include "kinematics.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- System state enumeration ----------------------------- */

/* Main system states (based on grbl state machine) */
typedef enum {
    SYS_STATE_IDLE = 0,         /* Machine is ready for new commands */
    SYS_STATE_RUNNING,          /* Executing G-code motion */
    SYS_STATE_HOLD,             /* Feed hold - motion paused */
    SYS_STATE_JOG,              /* Jogging mode */
    SYS_STATE_ALARM,            /* Alarm condition - requires reset */
    SYS_STATE_HOMING,           /* Homing cycle in progress */
    SYS_STATE_CHECK,            /* Check mode - parse but don't execute */
    SYS_STATE_SLEEP,            /* Low power state */
    SYS_STATE_DOOR,             /* Safety door open */
} system_state_t;

/* Alarm codes */
typedef enum {
    SYS_ALARM_NONE = 0,
    SYS_ALARM_HARD_LIMIT,       /* Hard limit triggered */
    SYS_ALARM_SOFT_LIMIT,       /* Soft limit exceeded */
    SYS_ALARM_ESTOP,            /* Emergency stop */
    SYS_ALARM_PROBE_FAIL,       /* Probe cycle failed */
    SYS_ALARM_HOMING_FAIL,      /* Homing cycle failed */
    SYS_ALARM_OVERFLOW,         /* Buffer overflow */
    SYS_ALARM_SPINDLE_STALL,    /* Spindle stall detected */
} system_alarm_t;

/* ----------------------------- System context structure ----------------------------- */

/* ----------------------------- System context structure ----------------------------- */

/* Global system context - contains all subsystem states */
typedef struct {
    /* Current system state */
    system_state_t state;
    system_alarm_t alarm;
    
    /* Subsystem states */
    gcode_state_t gcode;        /* G-code parser/executor state */
    planner_queue_t planner;    /* Motion planner queue */
    
    /* System flags */
    bool homed;                 /* Machine has been homed */
    bool limits_enabled;        /* Limit switches enabled */
    bool soft_limits_enabled;   /* Software limits enabled */
    bool spindle_enabled;       /* Spindle control enabled */
    
    /* Machine position (in mm) */
    float machine_x;
    float machine_y;
    float machine_z;
    
    /* Work position offset (G54/G55/etc) */
    float work_offset_x;
    float work_offset_y;
    float work_offset_z;
    
    /* Statistics */
    uint32_t total_lines_processed;
    uint32_t total_errors;
    uint32_t uptime_ms;         /* System uptime in milliseconds */
    
} system_context_t;

/* ----------------------------- Public API ----------------------------- */

/* Initialize the system state machine and all subsystems */
void system_init(system_context_t *sys);

/* Reset system to safe state (call after alarm) */
void system_reset(system_context_t *sys);

/* Main system poll - call frequently from main loop */
void system_poll(system_context_t *sys);

/* Process a G-code line (called by protocol layer or directly) */
void system_process_line(system_context_t *sys, const char *line);

/* ----------------------------- State management ----------------------------- */

/* Get current system state */
system_state_t system_get_state(const system_context_t *sys);

/* Request state transition */
bool system_set_state(system_context_t *sys, system_state_t new_state);

/* Trigger an alarm condition */
void system_trigger_alarm(system_context_t *sys, system_alarm_t alarm);

/* Clear alarm and return to idle (requires user acknowledgment) */
bool system_clear_alarm(system_context_t *sys);

/* Check if system can accept new commands */
bool system_is_idle(const system_context_t *sys);

/* Check if system is in an alarm state */
bool system_is_alarmed(const system_context_t *sys);

/* ----------------------------- Realtime command handlers ----------------------------- */

/* Handle feed hold request (!) */
void system_feed_hold(system_context_t *sys);

/* Handle cycle start request (~) */
void system_cycle_start(system_context_t *sys);

/* Handle soft reset request (Ctrl-X) */
void system_soft_reset(system_context_t *sys);

/* ----------------------------- Status reporting ----------------------------- */

/* Generate status report string (for '?' command) */
size_t system_get_status_report(const system_context_t *sys, char *buf, size_t buf_size);

/* Get state name as string */
const char *system_state_string(system_state_t state);

/* Get alarm description as string */
const char *system_alarm_string(system_alarm_t alarm);

/* ----------------------------- Position management ----------------------------- */

/* Get machine position (absolute machine coordinates) */
void system_get_machine_position(const system_context_t *sys, float *x, float *y, float *z);

/* Get work position (relative to work coordinate system) */
void system_get_work_position(const system_context_t *sys, float *x, float *y, float *z);

/* Set work offset (G92 or G10 L2) */
void system_set_work_offset(system_context_t *sys, float x, float y, float z);

/* ----------------------------- Homing ----------------------------- */

/* Start homing cycle for specified axes */
bool system_start_homing(system_context_t *sys, uint8_t axis_mask);

/* Check if machine is homed */
bool system_is_homed(const system_context_t *sys);

/* ----------------------------- Limits ----------------------------- */

/* Enable/disable limit switches */
void system_set_limits_enabled(system_context_t *sys, bool enabled);

/* Enable/disable soft limits */
void system_set_soft_limits_enabled(system_context_t *sys, bool enabled);

/* Check if a position is within soft limits */
bool system_check_soft_limits(const system_context_t *sys, float x, float y, float z);

#ifdef __cplusplus
}
#endif
