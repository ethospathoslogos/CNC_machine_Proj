#ifndef PLANNER_H
#define PLANNER_H

#include <stdint.h>

// Planner block structure
// This structure contains all the information needed for motion planning
typedef struct {
    // Speed parameters
    float entry_speed;        // Entry speed for this block (mm/min)
    float nominal_speed;      // Maximum speed this block can achieve (mm/min)
    float exit_speed;         // Exit speed for this block (mm/min)
    
    // Acceleration parameters
    float acceleration;       // Maximum acceleration for this block (mm/min^2)
    float max_entry_speed;    // Maximum allowable entry speed (mm/min)
    
    // Distance and time
    float millimeters;        // Total distance to travel in this block (mm)
    
    // Direction and step counts
    uint8_t direction_bits;   // Direction bits for each axis
    uint32_t step_event_count; // Number of step events for this block
    
    // Status flags
    uint8_t recalculate_flag; // Flag to indicate block needs recalculation
    uint8_t nominal_length_flag; // Flag to indicate block is running at nominal speed
    
    // Pointer to next block (for linked list)
    void *next;               // Pointer to next planner block
    
} planner_block_t;

// Function declarations
void planner_block_init(planner_block_t *block);
int planner_block_validate(const planner_block_t *block);

#endif // PLANNER_H
