/* gcode.c - G-code parser and executor for 2D CNC engraver */

#include "gcode.h"
#include "arc.h"
#include "kinematics.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* NAN helper for optional parameters */
#ifndef NAN
#define NAN (0.0f/0.0f)
#endif

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

/* ----------------------------- Initialization ----------------------------- */

void gcode_init(gcode_state_t *gc) {
    if (!gc) return;
    memset(gc, 0, sizeof(*gc));
    
    /* Default modal states (typical G-code startup) */
    gc->motion_mode = GCODE_MOTION_LINEAR;
    gc->coord_mode = GCODE_COORD_ABSOLUTE;
    gc->feed_mode = GCODE_FEED_UNITS_PER_MIN;
    gc->spindle_state = GCODE_SPINDLE_OFF;
    
    gc->absolute_mode = true;
    gc->feedrate = 100.0f;  /* default feedrate mm/min */
    gc->feedrate_set = false;
    gc->spindle_speed = 0.0f;
    gc->program_complete = false;
    
    gc->position_x = 0.0f;
    gc->position_y = 0.0f;
}

void gcode_reset(gcode_state_t *gc) {
    gcode_init(gc);
}

/* ----------------------------- Parsing helpers ----------------------------- */

/* Skip whitespace */
static const char *skip_ws(const char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    return s;
}

/* Parse a floating point number after a letter code */
static bool parse_float(const char **ptr, float *out) {
    char *end;
    float val = strtof(*ptr, &end);
    if (end == *ptr) return false;  /* no conversion */
    *out = val;
    *ptr = end;
    return true;
}

/* Parse an integer number after a letter code */
static bool parse_int(const char **ptr, int *out) {
    char *end;
    long val = strtol(*ptr, &end, 10);
    if (end == *ptr) return false;  /* no conversion */
    *out = (int)val;
    *ptr = end;
    return true;
}

/* ----------------------------- Line parsing ----------------------------- */

gcode_status_t gcode_parse_line(const char *line, gcode_block_t *block) {
    if (!line || !block) return GCODE_ERR_INVALID_PARAM;
    
    /* Initialize block */
    memset(block, 0, sizeof(*block));
    block->x = block->y = block->f = block->s = block->p = NAN;
    block->i = block->j = block->r = NAN;
    
    const char *ptr = skip_ws(line);
    
    /* Empty line */
    if (*ptr == '\0') return GCODE_OK;
    
    /* Parse word by word */
    while (*ptr) {
        ptr = skip_ws(ptr);
        if (*ptr == '\0') break;
        
        char letter = toupper((unsigned char)*ptr);
        ptr++;
        
        switch (letter) {
            case 'G': {
                int gnum;
                if (!parse_int(&ptr, &gnum)) return GCODE_ERR_INVALID_PARAM;
                block->g_code = gnum;
                block->has_g = true;
                break;
            }
            
            case 'M': {
                int mnum;
                if (!parse_int(&ptr, &mnum)) return GCODE_ERR_INVALID_PARAM;
                block->m_code = mnum;
                block->has_m = true;
                break;
            }
            
            case 'X': {
                if (!parse_float(&ptr, &block->x)) return GCODE_ERR_INVALID_PARAM;
                block->has_x = true;
                break;
            }
            
            case 'Y': {
                if (!parse_float(&ptr, &block->y)) return GCODE_ERR_INVALID_PARAM;
                block->has_y = true;
                break;
            }
            
            case 'F': {
                if (!parse_float(&ptr, &block->f)) return GCODE_ERR_INVALID_PARAM;
                block->has_f = true;
                break;
            }
            
            case 'S': {
                if (!parse_float(&ptr, &block->s)) return GCODE_ERR_INVALID_PARAM;
                block->has_s = true;
                break;
            }
            
            case 'P': {
                if (!parse_float(&ptr, &block->p)) return GCODE_ERR_INVALID_PARAM;
                block->has_p = true;
                break;
            }
            
            case 'I': {
                if (!parse_float(&ptr, &block->i)) return GCODE_ERR_INVALID_PARAM;
                block->has_i = true;
                break;
            }
            
            case 'J': {
                if (!parse_float(&ptr, &block->j)) return GCODE_ERR_INVALID_PARAM;
                block->has_j = true;
                break;
            }
            
            case 'R': {
                if (!parse_float(&ptr, &block->r)) return GCODE_ERR_INVALID_PARAM;
                block->has_r = true;
                break;
            }
            
            /* Ignore other letters for now */
            default:
                /* Skip to next space or end */
                while (*ptr && !isspace((unsigned char)*ptr)) ptr++;
                break;
        }
    }
    
    return GCODE_OK;
}

/* ----------------------------- Execution ----------------------------- */

/* Execute motion command - integrates with kinematics for segmentation */
static gcode_status_t execute_motion(gcode_state_t *gc, const gcode_block_t *block) {
    float target_x = gc->position_x;
    float target_y = gc->position_y;
    
    /* Update feedrate if F word is present */
    if (block->has_f) {
        if (block->f <= 0.0f) return GCODE_ERR_INVALID_PARAM;
        gc->feedrate = block->f;
        gc->feedrate_set = true;
    }
    
    /* Calculate target position */
    if (gc->absolute_mode) {
        /* Absolute mode: X/Y specify absolute coordinates */
        if (block->has_x) target_x = block->x;
        if (block->has_y) target_y = block->y;
    } else {
        /* Relative mode: X/Y specify offsets */
        if (block->has_x) target_x += block->x;
        if (block->has_y) target_y += block->y;
    }
    
    /* Validate that feedrate was set for non-rapid moves */
    if (gc->motion_mode == GCODE_MOTION_LINEAR && !gc->feedrate_set) {
        return GCODE_ERR_MISSING_PARAM;
    }
    
    /* Use kinematics segment_move for path segmentation when available.
     * This subdivides long moves into shorter segments as needed by the
     * machine geometry (e.g., CoreXY with max_segment_len set).
     */
    if (g_kin.segment_move) {
        kin_cart_t cart_current = {{ gc->position_x, gc->position_y, 0.0f }};
        kin_cart_t cart_target  = {{ target_x, target_y, 0.0f }};
        kin_motion_hint_t hint = {
            .feed_mm_min = (gc->motion_mode == GCODE_MOTION_RAPID) ? 0.0f : gc->feedrate,
            .accel_mm_s2 = 0.0f,
            .junction_dev_mm = 0.0f
        };
        kin_cart_t cart_next;
        bool init = true;
        
        while (g_kin.segment_move(&cart_target, &cart_current, &hint, init, &cart_next)) {
            init = false;
            /* Each segment endpoint could be converted to joint space and queued:
             *   kin_joint_t joint;
             *   g_kin.cart_to_joint(&cart_next, &joint);
             *   planner_enqueue_joint(&joint, feedrate);
             */
            cart_current = cart_next;
        }
    }
    
    /* Update position */
    gc->position_x = target_x;
    gc->position_y = target_y;
    
    return GCODE_OK;
}

/* Execute dwell command */
static gcode_status_t execute_dwell(gcode_state_t *gc, const gcode_block_t *block) {
    (void)gc;  /* unused in this simple implementation */
    
    if (!block->has_p) return GCODE_ERR_MISSING_PARAM;
    if (block->p < 0.0f) return GCODE_ERR_INVALID_PARAM;
    
    /* In a real system, this would schedule a dwell/delay */
    /* Example: hal_delay_ms((uint32_t)(block->p * 1000.0f)); */
    
    return GCODE_OK;
}

/* Arc segment callback context */
typedef struct {
    gcode_state_t *gc;
    gcode_status_t status;
} arc_cb_ctx_t;

/* Callback for each arc segment - updates position */
static bool arc_segment_handler(float x, float y, void *user) {
    arc_cb_ctx_t *ctx = (arc_cb_ctx_t *)user;
    
    /* Update position for each segment endpoint.
     * In a real system, each segment would be queued to the planner:
     *   planner_line_to(x, y, ctx->gc->feedrate);
     */
    ctx->gc->position_x = x;
    ctx->gc->position_y = y;
    
    return true;
}

/* Execute arc command (G02/G03) */
static gcode_status_t execute_arc(gcode_state_t *gc, const gcode_block_t *block,
                                  bool clockwise) {
    /* Update feedrate if F word is present */
    if (block->has_f) {
        if (block->f <= 0.0f) return GCODE_ERR_INVALID_PARAM;
        gc->feedrate = block->f;
        gc->feedrate_set = true;
    }
    
    /* Arc moves require a feedrate */
    if (!gc->feedrate_set) return GCODE_ERR_MISSING_PARAM;
    
    /* Calculate target position */
    float target_x = gc->position_x;
    float target_y = gc->position_y;
    
    if (gc->absolute_mode) {
        if (block->has_x) target_x = block->x;
        if (block->has_y) target_y = block->y;
    } else {
        if (block->has_x) target_x += block->x;
        if (block->has_y) target_y += block->y;
    }
    
    /* Set up callback context */
    arc_cb_ctx_t ctx = { .gc = gc, .status = GCODE_OK };
    
    bool ok;
    if (block->has_r) {
        /* R-form arc */
        ok = arc_generate_r(gc->position_x, gc->position_y,
                            target_x, target_y,
                            block->r, clockwise,
                            arc_segment_handler, &ctx);
    } else if (block->has_i || block->has_j) {
        /* I/J center-offset form */
        float i_off = block->has_i ? block->i : 0.0f;
        float j_off = block->has_j ? block->j : 0.0f;
        
        ok = arc_generate_ij(gc->position_x, gc->position_y,
                             target_x, target_y,
                             i_off, j_off, clockwise,
                             arc_segment_handler, &ctx);
    } else {
        /* No arc center specified */
        return GCODE_ERR_MISSING_PARAM;
    }
    
    if (!ok) return GCODE_ERR_INVALID_TARGET;
    
    return ctx.status;
}

/* Execute program end command (M02/M30) */
static gcode_status_t execute_program_end(gcode_state_t *gc, int m_code) {
    /* Turn off spindle for safety */
    gc->spindle_state = GCODE_SPINDLE_OFF;
    /* Interface with HAL: hal_spindle_set(HAL_SPINDLE_OFF, 0.0f); */
    
    /* Mark program as complete */
    gc->program_complete = true;
    
    if (m_code == 30) {
        /* M30 additionally resets position to origin (program rewind) */
        gc->position_x = 0.0f;
        gc->position_y = 0.0f;
    }
    
    return GCODE_OK;
}

/* Execute spindle control command */
static gcode_status_t execute_spindle(gcode_state_t *gc, int m_code, const gcode_block_t *block) {
    switch (m_code) {
        case 3:  /* M03 - spindle on, CW */
            gc->spindle_state = GCODE_SPINDLE_CW;
            if (block->has_s) {
                gc->spindle_speed = block->s;
            }
            /* Interface with HAL: hal_spindle_set(HAL_SPINDLE_CW, speed_to_pwm(gc->spindle_speed)); */
            break;
            
        case 4:  /* M04 - spindle on, CCW */
            gc->spindle_state = GCODE_SPINDLE_CCW;
            if (block->has_s) {
                gc->spindle_speed = block->s;
            }
            /* Interface with HAL: hal_spindle_set(HAL_SPINDLE_CCW, speed_to_pwm(gc->spindle_speed)); */
            break;
            
        case 5:  /* M05 - spindle off */
            gc->spindle_state = GCODE_SPINDLE_OFF;
            /* Interface with HAL: hal_spindle_set(HAL_SPINDLE_OFF, 0.0f); */
            break;
            
        default:
            return GCODE_ERR_UNKNOWN_CMD;
    }
    
    return GCODE_OK;
}

gcode_status_t gcode_execute_block(gcode_state_t *gc, const gcode_block_t *block) {
    if (!gc || !block) return GCODE_ERR_INVALID_PARAM;
    
    gcode_status_t status = GCODE_OK;
    
    /* Process G-code commands */
    if (block->has_g) {
        switch (block->g_code) {
            case 0:  /* G00 - rapid positioning */
                gc->motion_mode = GCODE_MOTION_RAPID;
                status = execute_motion(gc, block);
                break;
                
            case 1:  /* G01 - linear interpolation */
                gc->motion_mode = GCODE_MOTION_LINEAR;
                status = execute_motion(gc, block);
                break;
                
            case 2:  /* G02 - clockwise arc */
                gc->motion_mode = GCODE_MOTION_ARC_CW;
                status = execute_arc(gc, block, true);
                break;
                
            case 3:  /* G03 - counter-clockwise arc */
                gc->motion_mode = GCODE_MOTION_ARC_CCW;
                status = execute_arc(gc, block, false);
                break;
                
            case 4:  /* G04 - dwell */
                status = execute_dwell(gc, block);
                break;
                
            case 90: /* G90 - absolute positioning */
                gc->coord_mode = GCODE_COORD_ABSOLUTE;
                gc->absolute_mode = true;
                break;
                
            case 91: /* G91 - relative positioning */
                gc->coord_mode = GCODE_COORD_RELATIVE;
                gc->absolute_mode = false;
                break;
                
            case 94: /* G94 - feed rate mode (units per minute) */
                gc->feed_mode = GCODE_FEED_UNITS_PER_MIN;
                break;
                
            case 93: /* G93 - inverse time feed mode */
                gc->feed_mode = GCODE_FEED_INVERSE_TIME;
                break;
                
            default:
                return GCODE_ERR_UNSUPPORTED_CMD;
        }
        
        if (status != GCODE_OK) return status;
    }
    
    /* Process M-code commands */
    if (block->has_m) {
        switch (block->m_code) {
            case 2:  /* M02 - program end */
            case 30: /* M30 - program end and rewind */
                status = execute_program_end(gc, block->m_code);
                break;
            default:
                status = execute_spindle(gc, block->m_code, block);
                break;
        }
        if (status != GCODE_OK) return status;
    }
    
    /* Process standalone S word (spindle speed change without M code) */
    if (block->has_s && !block->has_m) {
        gc->spindle_speed = block->s;
        /* Update spindle if it's already running */
        if (gc->spindle_state != GCODE_SPINDLE_OFF) {
            /* Interface with HAL to update speed */
        }
    }
    
    return GCODE_OK;
}

gcode_status_t gcode_process_line(gcode_state_t *gc, const char *line) {
    gcode_block_t block;
    gcode_status_t status;
    
    status = gcode_parse_line(line, &block);
    if (status != GCODE_OK) return status;
    
    status = gcode_execute_block(gc, &block);
    return status;
}

/* ----------------------------- Query functions ----------------------------- */

void gcode_get_position(const gcode_state_t *gc, float *out_x, float *out_y) {
    if (!gc) return;
    if (out_x) *out_x = gc->position_x;
    if (out_y) *out_y = gc->position_y;
}

float gcode_get_feedrate(const gcode_state_t *gc) {
    return gc ? gc->feedrate : 0.0f;
}

float gcode_get_spindle_speed(const gcode_state_t *gc) {
    return gc ? gc->spindle_speed : 0.0f;
}

gcode_spindle_state_t gcode_get_spindle_state(const gcode_state_t *gc) {
    return gc ? gc->spindle_state : GCODE_SPINDLE_OFF;
}

bool gcode_is_program_complete(const gcode_state_t *gc) {
    return gc ? gc->program_complete : false;
}

const char *gcode_status_string(gcode_status_t status) {
    switch (status) {
        case GCODE_OK:                  return "OK";
        case GCODE_ERR_MISSING_PARAM:   return "Missing parameter";
        case GCODE_ERR_INVALID_PARAM:   return "Invalid parameter";
        case GCODE_ERR_UNKNOWN_CMD:     return "Unknown command";
        case GCODE_ERR_UNSUPPORTED_CMD: return "Unsupported command";
        case GCODE_ERR_INVALID_TARGET:  return "Invalid target";
        case GCODE_ERR_OVERFLOW:        return "Overflow";
        default:                        return "Unknown error";
    }
}
