/* gcode_test.c - Unit tests for gcode parser and executor */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include "../src/gcode.h"

/* Helper to check if two floats are approximately equal */
static int float_equal(float a, float b) {
    return fabsf(a - b) < 0.001f;
}

void test_init_and_reset() {
    printf("Testing gcode_init and gcode_reset...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    assert(gc.position_x == 0.0f);
    assert(gc.position_y == 0.0f);
    assert(gc.feedrate == 100.0f);
    assert(gc.spindle_speed == 0.0f);
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    assert(gc.absolute_mode == true);
    
    printf("  [PASSED]\n");
}

void test_parse_simple_commands() {
    printf("Testing gcode_parse_line with simple commands...\n");
    
    gcode_block_t block;
    gcode_status_t status;
    
    /* Test G00 command */
    status = gcode_parse_line("G00 X10 Y20", &block);
    assert(status == GCODE_OK);
    assert(block.has_g == true);
    assert(block.g_code == 0);
    assert(block.has_x == true);
    assert(float_equal(block.x, 10.0f));
    assert(block.has_y == true);
    assert(float_equal(block.y, 20.0f));
    
    /* Test G01 command with feedrate */
    status = gcode_parse_line("G01 X50 Y75 F300", &block);
    assert(status == GCODE_OK);
    assert(block.has_g == true);
    assert(block.g_code == 1);
    assert(block.has_x == true);
    assert(float_equal(block.x, 50.0f));
    assert(block.has_y == true);
    assert(float_equal(block.y, 75.0f));
    assert(block.has_f == true);
    assert(float_equal(block.f, 300.0f));
    
    /* Test M03 with spindle speed */
    status = gcode_parse_line("M03 S1000", &block);
    assert(status == GCODE_OK);
    assert(block.has_m == true);
    assert(block.m_code == 3);
    assert(block.has_s == true);
    assert(float_equal(block.s, 1000.0f));
    
    printf("  [PASSED]\n");
}

void test_motion_execution() {
    printf("Testing motion command execution...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Test rapid positioning G00 */
    gcode_status_t status = gcode_process_line(&gc, "G00 X50 Y30");
    assert(status == GCODE_OK);
    assert(float_equal(gc.position_x, 50.0f));
    assert(float_equal(gc.position_y, 30.0f));
    
    /* Test linear interpolation G01 with feedrate */
    status = gcode_process_line(&gc, "G01 X75 Y50 F300");
    assert(status == GCODE_OK);
    assert(float_equal(gc.position_x, 75.0f));
    assert(float_equal(gc.position_y, 50.0f));
    assert(float_equal(gc.feedrate, 300.0f));
    
    printf("  [PASSED]\n");
}

void test_spindle_control() {
    printf("Testing spindle control commands...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Test M03 - spindle on CW */
    gcode_status_t status = gcode_process_line(&gc, "M03 S2000");
    assert(status == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_CW);
    assert(float_equal(gc.spindle_speed, 2000.0f));
    
    /* Test M04 - spindle on CCW */
    status = gcode_process_line(&gc, "M04 S1500");
    assert(status == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_CCW);
    assert(float_equal(gc.spindle_speed, 1500.0f));
    
    /* Test M05 - spindle off */
    status = gcode_process_line(&gc, "M05");
    assert(status == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    
    printf("  [PASSED]\n");
}

void test_absolute_relative_modes() {
    printf("Testing absolute and relative positioning modes...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Start in absolute mode */
    assert(gc.absolute_mode == true);
    
    /* Move to absolute position */
    gcode_process_line(&gc, "G00 X10 Y20");
    assert(float_equal(gc.position_x, 10.0f));
    assert(float_equal(gc.position_y, 20.0f));
    
    /* Switch to relative mode */
    gcode_process_line(&gc, "G91");
    assert(gc.absolute_mode == false);
    
    /* Move relative */
    gcode_process_line(&gc, "G00 X5 Y10");
    assert(float_equal(gc.position_x, 15.0f));
    assert(float_equal(gc.position_y, 30.0f));
    
    /* Switch back to absolute mode */
    gcode_process_line(&gc, "G90");
    assert(gc.absolute_mode == true);
    
    /* Move to absolute position */
    gcode_process_line(&gc, "G00 X100 Y50");
    assert(float_equal(gc.position_x, 100.0f));
    assert(float_equal(gc.position_y, 50.0f));
    
    printf("  [PASSED]\n");
}

void test_feedrate_modes() {
    printf("Testing feedrate modes...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Default should be units per minute (G94) */
    assert(gc.feed_mode == GCODE_FEED_UNITS_PER_MIN);
    
    /* Set feedrate */
    gcode_process_line(&gc, "G01 X10 Y10 F500");
    assert(float_equal(gc.feedrate, 500.0f));
    
    /* Switch to inverse time mode */
    gcode_process_line(&gc, "G93");
    assert(gc.feed_mode == GCODE_FEED_INVERSE_TIME);
    
    /* Switch back to units per minute */
    gcode_process_line(&gc, "G94");
    assert(gc.feed_mode == GCODE_FEED_UNITS_PER_MIN);
    
    printf("  [PASSED]\n");
}

void test_dwell_command() {
    printf("Testing dwell (G04) command...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Test dwell with P parameter */
    gcode_status_t status = gcode_process_line(&gc, "G04 P2.5");
    assert(status == GCODE_OK);
    
    /* Test dwell without P parameter (should fail) */
    status = gcode_process_line(&gc, "G04");
    assert(status == GCODE_ERR_MISSING_PARAM);
    
    printf("  [PASSED]\n");
}

void test_query_functions() {
    printf("Testing query functions...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Set up some state */
    gcode_process_line(&gc, "G01 X100 Y200 F250");
    gcode_process_line(&gc, "M03 S3000");
    
    /* Test query functions */
    float x, y;
    gcode_get_position(&gc, &x, &y);
    assert(float_equal(x, 100.0f));
    assert(float_equal(y, 200.0f));
    
    assert(float_equal(gcode_get_feedrate(&gc), 250.0f));
    assert(float_equal(gcode_get_spindle_speed(&gc), 3000.0f));
    assert(gcode_get_spindle_state(&gc) == GCODE_SPINDLE_CW);
    
    printf("  [PASSED]\n");
}

void test_error_handling() {
    printf("Testing error handling...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Test unsupported G-code */
    gcode_status_t status = gcode_process_line(&gc, "G99");
    assert(status == GCODE_ERR_UNSUPPORTED_CMD);
    
    /* Test unknown M-code */
    status = gcode_process_line(&gc, "M99");
    assert(status == GCODE_ERR_UNKNOWN_CMD);
    
    /* Test invalid parameter (negative feedrate) */
    status = gcode_process_line(&gc, "G01 X10 Y10 F-100");
    assert(status == GCODE_ERR_INVALID_PARAM);
    
    /* Test error messages */
    const char *msg = gcode_status_string(GCODE_OK);
    assert(strcmp(msg, "OK") == 0);
    
    msg = gcode_status_string(GCODE_ERR_MISSING_PARAM);
    assert(strcmp(msg, "Missing parameter") == 0);
    
    printf("  [PASSED]\n");
}

void test_program_end_m02() {
    printf("Testing M02 program end...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Set up running state */
    assert(gcode_process_line(&gc, "M03 S1000") == GCODE_OK);
    assert(gcode_process_line(&gc, "G01 X50 Y50 F200") == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_CW);
    assert(!gcode_is_program_complete(&gc));
    
    /* Execute M02 */
    gcode_status_t status = gcode_process_line(&gc, "M02");
    assert(status == GCODE_OK);
    
    /* Verify: spindle off, program complete, position unchanged */
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    assert(gcode_is_program_complete(&gc));
    assert(float_equal(gc.position_x, 50.0f));
    assert(float_equal(gc.position_y, 50.0f));
    
    printf("  [PASSED]\n");
}

void test_program_end_m30() {
    printf("Testing M30 program end with rewind...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Set up running state */
    assert(gcode_process_line(&gc, "M03 S1000") == GCODE_OK);
    assert(gcode_process_line(&gc, "G01 X50 Y50 F200") == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_CW);
    
    /* Execute M30 */
    gcode_status_t status = gcode_process_line(&gc, "M30");
    assert(status == GCODE_OK);
    
    /* Verify: spindle off, program complete, position reset to origin */
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    assert(gcode_is_program_complete(&gc));
    assert(float_equal(gc.position_x, 0.0f));
    assert(float_equal(gc.position_y, 0.0f));
    
    printf("  [PASSED]\n");
}

void test_arc_cw_ij() {
    printf("Testing G02 clockwise arc with I/J...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Move to start position */
    assert(gcode_process_line(&gc, "G00 X10 Y0") == GCODE_OK);
    
    /* CW arc from (10,0) to (0,10) with center at (0,0)
     * This traverses 270 degrees clockwise (3/4 circle).
     * I = 0 - 10 = -10, J = 0 - 0 = 0 */
    gcode_status_t status = gcode_process_line(&gc, "G02 X0 Y10 I-10 J0 F300");
    assert(status == GCODE_OK);
    
    /* Final position should be the arc endpoint */
    assert(float_equal(gc.position_x, 0.0f));
    assert(float_equal(gc.position_y, 10.0f));
    
    /* Motion mode should be arc CW */
    assert(gc.motion_mode == GCODE_MOTION_ARC_CW);
    
    printf("  [PASSED]\n");
}

void test_arc_ccw_ij() {
    printf("Testing G03 counter-clockwise arc with I/J...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Move to start position */
    assert(gcode_process_line(&gc, "G00 X10 Y0") == GCODE_OK);
    
    /* Quarter-circle CCW arc from (10,0) to (0,-10) with center at (0,0)
     * I = -10, J = 0 */
    gcode_status_t status = gcode_process_line(&gc, "G03 X0 Y-10 I-10 J0 F300");
    assert(status == GCODE_OK);
    
    /* Final position should be the arc endpoint */
    assert(float_equal(gc.position_x, 0.0f));
    assert(float_equal(gc.position_y, -10.0f));
    
    /* Motion mode should be arc CCW */
    assert(gc.motion_mode == GCODE_MOTION_ARC_CCW);
    
    printf("  [PASSED]\n");
}

void test_arc_r_form() {
    printf("Testing G02 arc with R parameter...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Move to start position */
    assert(gcode_process_line(&gc, "G00 X10 Y0") == GCODE_OK);
    
    /* Semicircle CW from (10,0) to (-10,0) with R=10 */
    gcode_status_t status = gcode_process_line(&gc, "G02 X-10 Y0 R10 F300");
    assert(status == GCODE_OK);
    
    /* Final position should be the arc endpoint */
    assert(float_equal(gc.position_x, -10.0f));
    assert(float_equal(gc.position_y, 0.0f));
    
    printf("  [PASSED]\n");
}

void test_arc_missing_params() {
    printf("Testing arc with missing parameters...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Arc without I, J, or R should fail */
    gcode_status_t status = gcode_process_line(&gc, "G02 X10 Y10 F300");
    assert(status == GCODE_ERR_MISSING_PARAM);
    
    /* Arc without feedrate should fail (when feedrate was never set) */
    gc.feedrate_set = false;
    status = gcode_process_line(&gc, "G02 X10 Y10 I5 J0");
    assert(status == GCODE_ERR_MISSING_PARAM);
    
    printf("  [PASSED]\n");
}

void test_arc_parse_ij_params() {
    printf("Testing parsing of I, J, R parameters...\n");
    
    gcode_block_t block;
    gcode_status_t status;
    
    /* Test G02 with I and J */
    status = gcode_parse_line("G02 X10 Y20 I5 J-3 F100", &block);
    assert(status == GCODE_OK);
    assert(block.has_g == true);
    assert(block.g_code == 2);
    assert(block.has_i == true);
    assert(float_equal(block.i, 5.0f));
    assert(block.has_j == true);
    assert(float_equal(block.j, -3.0f));
    
    /* Test G03 with R */
    status = gcode_parse_line("G03 X10 Y20 R15 F100", &block);
    assert(status == GCODE_OK);
    assert(block.g_code == 3);
    assert(block.has_r == true);
    assert(float_equal(block.r, 15.0f));
    
    printf("  [PASSED]\n");
}

void test_engraver_workflow_with_arcs() {
    printf("Testing complete engraver workflow with arcs and program end...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* 1. Home / start position */
    assert(gcode_process_line(&gc, "G00 X0 Y0") == GCODE_OK);
    
    /* 2. Turn on spindle */
    assert(gcode_process_line(&gc, "M03 S1500") == GCODE_OK);
    
    /* 3. Dwell for spindle start */
    assert(gcode_process_line(&gc, "G04 P0.5") == GCODE_OK);
    
    /* 4. Linear move to start of curve */
    assert(gcode_process_line(&gc, "G01 X10 Y0 F200") == GCODE_OK);
    assert(float_equal(gc.position_x, 10.0f));
    
    /* 5. CW arc */
    assert(gcode_process_line(&gc, "G02 X0 Y10 I-10 J0 F200") == GCODE_OK);
    assert(float_equal(gc.position_x, 0.0f));
    assert(float_equal(gc.position_y, 10.0f));
    
    /* 6. Another linear move */
    assert(gcode_process_line(&gc, "G01 X0 Y20") == GCODE_OK);
    
    /* 7. CCW arc */
    assert(gcode_process_line(&gc, "G03 X10 Y30 I10 J0 F200") == GCODE_OK);
    assert(float_equal(gc.position_x, 10.0f));
    assert(float_equal(gc.position_y, 30.0f));
    
    /* 8. Turn off spindle and end program */
    assert(gcode_process_line(&gc, "M05") == GCODE_OK);
    assert(gcode_process_line(&gc, "M30") == GCODE_OK);
    
    /* Verify final state */
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    assert(gcode_is_program_complete(&gc));
    assert(float_equal(gc.position_x, 0.0f)); /* M30 rewinds */
    assert(float_equal(gc.position_y, 0.0f));
    
    printf("  [PASSED]\n");
}

void test_2d_engraver_workflow() {
    printf("Testing complete 2D engraver workflow...\n");
    
    gcode_state_t gc;
    gcode_init(&gc);
    
    /* Typical engraving sequence */
    
    /* 1. Move to start position (rapid) */
    assert(gcode_process_line(&gc, "G00 X0 Y0") == GCODE_OK);
    assert(float_equal(gc.position_x, 0.0f));
    assert(float_equal(gc.position_y, 0.0f));
    
    /* 2. Turn on spindle */
    assert(gcode_process_line(&gc, "M03 S1500") == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_CW);
    assert(float_equal(gc.spindle_speed, 1500.0f));
    
    /* 3. Engrave a line with feedrate */
    assert(gcode_process_line(&gc, "G01 X50 Y0 F200") == GCODE_OK);
    assert(float_equal(gc.position_x, 50.0f));
    assert(float_equal(gc.feedrate, 200.0f));
    
    /* 4. Engrave another line */
    assert(gcode_process_line(&gc, "G01 X50 Y50") == GCODE_OK);
    assert(float_equal(gc.position_y, 50.0f));
    
    /* 5. Rapid move to next position */
    assert(gcode_process_line(&gc, "G00 X100 Y100") == GCODE_OK);
    assert(float_equal(gc.position_x, 100.0f));
    assert(float_equal(gc.position_y, 100.0f));
    
    /* 6. Turn off spindle */
    assert(gcode_process_line(&gc, "M05") == GCODE_OK);
    assert(gc.spindle_state == GCODE_SPINDLE_OFF);
    
    printf("  [PASSED]\n");
}

int main() {
    printf("\n=== G-code Parser and Executor Tests ===\n\n");
    
    test_init_and_reset();
    test_parse_simple_commands();
    test_motion_execution();
    test_spindle_control();
    test_absolute_relative_modes();
    test_feedrate_modes();
    test_dwell_command();
    test_query_functions();
    test_error_handling();
    test_program_end_m02();
    test_program_end_m30();
    test_arc_parse_ij_params();
    test_arc_cw_ij();
    test_arc_ccw_ij();
    test_arc_r_form();
    test_arc_missing_params();
    test_2d_engraver_workflow();
    test_engraver_workflow_with_arcs();
    
    printf("\n=== All G-code tests passed! ===\n\n");
    return 0;
}
