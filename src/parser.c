#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// --- Constants ---
#define MAX_ERROR_MESSAGE 256
#define TRUE 1
#define FALSE 0

// --- Enums ---
typedef enum {
    CMD_MOVE_LINEAR,   // G01
    CMD_MOVE_RAPID,    // G00
    CMD_DWELL,         // G04
    CMD_SPINDLE_ON,    // M03
    CMD_SPINDLE_OFF,   // M05
    CMD_UNKNOWN
} CommandType;

// --- State Representation ---
typedef struct {
    float positionX;   // Current X-axis position
    float positionY;   // Current Y-axis position
    float feedRate;    // Current feed rate
    int spindleOn;     // Spindle (tool) state: 1=ON, 0=OFF
} MachineState;

// --- Global State and Error Handling ---
static MachineState machineState = {0, 0, 0, 0};
static char lastErrorMessage[MAX_ERROR_MESSAGE];

// --- Utility Functions ---
// Trim leading and trailing spaces from a string
static void trim(char *str) {
    char *end;
    while (isspace((unsigned char)*str)) str++;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;
}

// Store an error message (for testing purposes)
static void set_error_message(const char *message) {
    strncpy(lastErrorMessage, message, MAX_ERROR_MESSAGE - 1);
    lastErrorMessage[MAX_ERROR_MESSAGE - 1] = '\0';
}

// Retrieve the last error for debugging
const char *get_last_error() {
    return lastErrorMessage;
}

// --- Command Parsing ---
CommandType parse_command_type(const char *command) {
    if (strncmp(command, "G00", 3) == 0) return CMD_MOVE_RAPID;
    if (strncmp(command, "G01", 3) == 0) return CMD_MOVE_LINEAR;
    if (strncmp(command, "G04", 3) == 0) return CMD_DWELL;
    if (strncmp(command, "M03", 3) == 0) return CMD_SPINDLE_ON;
    if (strncmp(command, "M05", 3) == 0) return CMD_SPINDLE_OFF;
    return CMD_UNKNOWN;
}

// Parse G-code parameters (X, Y, F, etc.)
int parse_parameters(const char *command, float *x, float *y, float *f) {
    *x = *y = *f = -1; // Initialize values to -1 (unspecified)
    const char *ptr = command;

    while (*ptr) {
        if (*ptr == 'X') {
            *x = atof(ptr + 1);
        } else if (*ptr == 'Y') {
            *y = atof(ptr + 1);
        } else if (*ptr == 'F') {
            *f = atof(ptr + 1);
        }
        ptr++;
    }
    return 0;
}

// --- Command Execution ---
int execute_move(float x, float y, float f) {
    if (x == -1 || y == -1) {
        set_error_message("Missing parameters");
        return -1;
    }
    if (f <= 0) {
        set_error_message("Invalid parameter format");
        return -1;
    }

    machineState.positionX = x;
    machineState.positionY = y;
    machineState.feedRate = f;
    return 0;
}

int execute_dwell(float seconds) {
    if (seconds <= 0) {
        set_error_message("Dwell time must be greater than zero.");
        return -1;
    }
    // Simulate a dwell (no actual hardware interaction here)
    return 0;
}

int execute_command(CommandType type, float x, float y, float f) {
    switch (type) {
        case CMD_MOVE_RAPID:
            return execute_move(x, y, 1000);  // Rapid movement uses a default high feed rate (e.g., 1000).
        case CMD_MOVE_LINEAR:
            return execute_move(x, y, f);
        case CMD_DWELL:
            return execute_dwell(f);
        case CMD_SPINDLE_ON:
            machineState.spindleOn = 1;
            return 0;
        case CMD_SPINDLE_OFF:
            machineState.spindleOn = 0;
            return 0;
        default:
            set_error_message("Unknown command");
            return -1;
    }
}

// --- Main Parsing Function ---
int parse_gcode_command(const char *command) {
    CommandType type = parse_command_type(command);  // Determine the command type

    float x = -1, y = -1, f = -1;  // Default parameters (-1 means unspecified, optional)
    parse_parameters(command, &x, &y, &f);

    switch (type) {
        case CMD_MOVE_LINEAR:
            return execute_move(x, y, f);
        case CMD_MOVE_RAPID:
            return execute_move(x, y, 1000);  // Rapid move feed rate is defaulted
        case CMD_SPINDLE_ON:
            machineState.spindleOn = 1;
            return 0;
        case CMD_SPINDLE_OFF:
            machineState.spindleOn = 0;
            return 0;
        case CMD_UNKNOWN:
        default:
            // Corrected error message
            set_error_message("Unknown command");
            return -1;
    }
}

// --- Machine State Query Functions ---
float machine_state_position_x() {
    return machineState.positionX;
}

float machine_state_position_y() {
    return machineState.positionY;
}

float machine_state_last_feed_rate() {
    return machineState.feedRate;
}

int machine_state_spindle_on() {
    return machineState.spindleOn;
}
