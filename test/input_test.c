#include <assert.h>
#include <stdio.h>
#include <string.h>

// Mocked functions for the 2D engraving machine to simulate the input command layer
// These will be part of your input/command layer implementation.
int parse_gcode_command(const char *command);         // Parses and executes a G-code command
const char *get_last_error();                         // Retrieves the last error description
int machine_state_spindle_on();                       // Returns 1 if the spindle (engraver) is ON
float machine_state_last_feed_rate();                 // Returns the last set feed rate
float machine_state_position_x();                     // Returns the current X position
float machine_state_position_y();                     // Returns the current Y position

// Test for movement commands (G00, G01)
void test_movement_commands() {
    printf("Testing movement commands...\n");

    // Test rapid positioning (G00)
    assert(parse_gcode_command("G00 X50 Y30") == 0);
    assert(machine_state_position_x() == 50.0);
    assert(machine_state_position_y() == 30.0);

    // Test linear movement with feed rate (G01)
    assert(parse_gcode_command("G01 X75 Y50 F300") == 0);
    assert(machine_state_position_x() == 75.0);
    assert(machine_state_position_y() == 50.0);
    assert(machine_state_last_feed_rate() == 300.0);

    printf("[passed].\n");
}

// Test for engraving control commands (M03, M05)
void test_engraving_control_commands() {
    printf("Testing engraving control commands...\n");

    // Test turning on the spindle with M03
    assert(parse_gcode_command("M03") == 0);
    assert(machine_state_spindle_on() == 1);

    // Test turning off the spindle with M05
    assert(parse_gcode_command("M05") == 0);
    assert(machine_state_spindle_on() == 0);

    printf("[passed].\n");
}

// Test for invalid inputs and error states
void test_invalid_commands() {
    printf("Testing invalid commands...\n");

    // Test unknown command
    assert(parse_gcode_command("G999") != 0);
    assert(strcmp(get_last_error(), "Unknown command") == 0);

    // Test missing parameters
    assert(parse_gcode_command("G01") != 0);  // Missing X and Y
    assert(strcmp(get_last_error(), "Missing parameters") == 0);

    // Test invalid parameter format
    assert(parse_gcode_command("G01 Xabc Y20") != 0);
    assert(strcmp(get_last_error(), "Invalid parameter format") == 0);

    printf("Invalid commands passed.\n");
}

// Main function to execute all test cases
int main() {
    // Run individual test cases
    test_movement_commands();
    test_engraving_control_commands();
    test_invalid_commands();

    printf("All tests passed for the 2D engraving machine.\n");
    return 0;
}
