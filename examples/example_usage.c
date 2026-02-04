/* example_usage.c - Example of how to use the system_state module
 *
 * This example demonstrates basic usage of the system state management module
 * for a CNC machine controller.
 */

#include "system_state.h"
#include <stdio.h>

/* Example: Simple main loop for a CNC controller */
int main(void) {
    /* Initialize system context */
    system_context_t sys;
    system_init(&sys);
    
    printf("CNC System State Manager - Example\n");
    printf("===================================\n\n");
    
    /* Check initial state */
    printf("Initial state: %s\n", system_state_string(system_get_state(&sys)));
    printf("System is idle: %s\n\n", system_is_idle(&sys) ? "Yes" : "No");
    
    /* Process some G-code commands */
    printf("Processing G-code commands:\n");
    system_process_line(&sys, "G90");           /* Absolute positioning */
    system_process_line(&sys, "G00 X10 Y20");   /* Rapid move to X10, Y20 */
    system_process_line(&sys, "G01 X15 Y25 F300"); /* Linear move at feedrate 300 */
    
    /* Get current position */
    float x, y, z;
    system_get_machine_position(&sys, &x, &y, &z);
    printf("Current machine position: X=%.2f, Y=%.2f, Z=%.2f\n\n", x, y, z);
    
    /* Demonstrate feed hold */
    printf("Demonstrating feed hold:\n");
    system_set_state(&sys, SYS_STATE_RUNNING);
    printf("State: %s\n", system_state_string(system_get_state(&sys)));
    
    system_feed_hold(&sys);
    printf("After feed hold: %s\n", system_state_string(system_get_state(&sys)));
    
    system_cycle_start(&sys);
    printf("After cycle start: %s\n\n", system_state_string(system_get_state(&sys)));
    
    /* Demonstrate alarm handling */
    printf("Demonstrating alarm handling:\n");
    system_trigger_alarm(&sys, SYS_ALARM_HARD_LIMIT);
    printf("After triggering hard limit alarm:\n");
    printf("  State: %s\n", system_state_string(system_get_state(&sys)));
    printf("  Alarm: %s\n", system_alarm_string(sys.alarm));
    printf("  Is alarmed: %s\n", system_is_alarmed(&sys) ? "Yes" : "No");
    
    /* Clear alarm */
    system_clear_alarm(&sys);
    printf("After clearing alarm:\n");
    printf("  State: %s\n", system_state_string(system_get_state(&sys)));
    printf("  Is alarmed: %s\n\n", system_is_alarmed(&sys) ? "Yes" : "No");
    
    /* Generate status report */
    printf("Generating status report:\n");
    char status_buf[256];
    system_get_status_report(&sys, status_buf, sizeof(status_buf));
    printf("  %s\n\n", status_buf);
    
    /* Demonstrate homing */
    printf("Demonstrating homing:\n");
    printf("Is homed: %s\n", system_is_homed(&sys) ? "Yes" : "No");
    system_start_homing(&sys, 0x03);  /* Home X and Y axes */
    printf("After homing:\n");
    printf("  Is homed: %s\n", system_is_homed(&sys) ? "Yes" : "No");
    printf("  State: %s\n\n", system_state_string(system_get_state(&sys)));
    
    /* Demonstrate soft limits */
    printf("Demonstrating soft limits:\n");
    system_set_soft_limits_enabled(&sys, true);
    printf("Valid position (100, 100, -10): %s\n",
           system_check_soft_limits(&sys, 100.0f, 100.0f, -10.0f) ? "Yes" : "No");
    printf("Invalid position (300, 100, -10): %s\n",
           system_check_soft_limits(&sys, 300.0f, 100.0f, -10.0f) ? "Yes" : "No");
    
    printf("\n=== Example completed successfully ===\n");
    
    return 0;
}
