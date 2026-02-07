/* gcode.h - G-code parser and executor for 2D CNC engraver
 *
 * Purpose:
 *  - Parse G-code commands from protocol layer
 *  - Maintain modal state (position, feedrate, spindle speed, etc.)
 *  - Interface with planner for motion execution
 *  - Support 2D engraving (X, Y axes only)
 *
 * Supported commands:
 *  Motion: G00 (rapid), G01 (linear), G02 (arc CW), G03 (arc CCW), G04 (dwell)
 *  Feed modes: G94 (units per minute)
 *  Spindle: M03 (on CW), M04 (on CCW), M05 (off)
 *  Program: M02 (program end), M30 (program end and rewind)
 *  Spindle speed: S parameter (RPM)
 *  Feedrate: F parameter (mm/min)
 *  Coordinates: G90 (absolute), G91 (relative)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------- G-code state ----------------------------- */

/* Modal groups (simplified for 2D engraver) */
typedef enum {
    GCODE_MOTION_RAPID = 0,      /* G00 - rapid positioning */
    GCODE_MOTION_LINEAR = 1,     /* G01 - linear interpolation */
    GCODE_MOTION_ARC_CW = 2,    /* G02 - clockwise arc */
    GCODE_MOTION_ARC_CCW = 3,   /* G03 - counter-clockwise arc */
    GCODE_MOTION_DWELL = 4,      /* G04 - dwell */
} gcode_motion_mode_t;

typedef enum {
    GCODE_COORD_ABSOLUTE = 0,    /* G90 - absolute positioning */
    GCODE_COORD_RELATIVE = 1,    /* G91 - relative positioning */
} gcode_coord_mode_t;

typedef enum {
    GCODE_FEED_UNITS_PER_MIN = 0, /* G94 - feed in units per minute */
    GCODE_FEED_INVERSE_TIME = 1,  /* G93 - inverse time feed mode */
} gcode_feed_mode_t;

typedef enum {
    GCODE_SPINDLE_OFF = 0,
    GCODE_SPINDLE_CW = 1,         /* M03 - clockwise */
    GCODE_SPINDLE_CCW = 2,        /* M04 - counter-clockwise */
} gcode_spindle_state_t;

/* Parser result codes */
typedef enum {
    GCODE_OK = 0,
    GCODE_ERR_MISSING_PARAM,
    GCODE_ERR_INVALID_PARAM,
    GCODE_ERR_UNKNOWN_CMD,
    GCODE_ERR_UNSUPPORTED_CMD,
    GCODE_ERR_INVALID_TARGET,
    GCODE_ERR_OVERFLOW,
} gcode_status_t;

/* Modal state machine */
typedef struct {
    /* Current position (in machine coordinates, mm) */
    float position_x;
    float position_y;
    
    /* Modal states */
    gcode_motion_mode_t motion_mode;
    gcode_coord_mode_t coord_mode;
    gcode_feed_mode_t feed_mode;
    gcode_spindle_state_t spindle_state;
    
    /* Feed and speed parameters */
    float feedrate;        /* mm/min */
    float spindle_speed;   /* RPM or 0-100% depending on implementation */
    
    /* Flags */
    bool feedrate_set;     /* true if F was ever specified */
    bool absolute_mode;    /* derived from coord_mode for convenience */
    bool program_complete; /* true after M02/M30 - program has ended */
    
} gcode_state_t;

/* Parsed G-code block */
typedef struct {
    /* Commanded values (NAN if not specified) */
    float x, y;            /* Target position or offset */
    float i, j;            /* Arc center offsets (relative to current position) */
    float r;               /* Arc radius (alternative to I/J) */
    float f;               /* Feedrate */
    float s;               /* Spindle speed */
    float p;               /* Dwell time (seconds) */
    
    /* Word flags (which words were present) */
    bool has_x, has_y;
    bool has_i, has_j, has_r;
    bool has_f, has_s, has_p;
    
    /* Modal commands */
    int g_code;            /* G-code number (0, 1, 2, 3, 4, 90, 91, 94, etc.) */
    int m_code;            /* M-code number (2, 3, 4, 5, 30, etc.) */
    
    bool has_g, has_m;
    
} gcode_block_t;

/* ----------------------------- Public API ----------------------------- */

/* Initialize the G-code parser/executor state */
void gcode_init(gcode_state_t *gc);

/* Reset to safe startup state */
void gcode_reset(gcode_state_t *gc);

/* Parse a single G-code line (already normalized by protocol layer) */
gcode_status_t gcode_parse_line(const char *line, gcode_block_t *block);

/* Execute a parsed G-code block (updates state, sends to planner) */
gcode_status_t gcode_execute_block(gcode_state_t *gc, const gcode_block_t *block);

/* Convenience: parse + execute in one call */
gcode_status_t gcode_process_line(gcode_state_t *gc, const char *line);

/* Query current state */
void gcode_get_position(const gcode_state_t *gc, float *out_x, float *out_y);
float gcode_get_feedrate(const gcode_state_t *gc);
float gcode_get_spindle_speed(const gcode_state_t *gc);
gcode_spindle_state_t gcode_get_spindle_state(const gcode_state_t *gc);
bool gcode_is_program_complete(const gcode_state_t *gc);

/* Get error message for a status code */
const char *gcode_status_string(gcode_status_t status);

#ifdef __cplusplus
}
#endif
