# G-code Module for 2D Engraver

## Overview

The G-code module (`src/gcode.c` and `src/gcode.h`) provides a complete parser and executor for G-code commands tailored for a 2D CNC engraver. This implementation focuses on X and Y axis motion control with feedrate and spindle speed management.

## Supported G-code Commands

### Motion Commands
- **G00** - Rapid positioning (no feedrate required)
  ```gcode
  G00 X50 Y30
  ```

- **G01** - Linear interpolation with feedrate
  ```gcode
  G01 X100 Y50 F250
  ```

- **G04** - Dwell (pause for specified seconds)
  ```gcode
  G04 P2.5
  ```

### Coordinate Modes
- **G90** - Absolute positioning mode (default)
  ```gcode
  G90
  G00 X100 Y100  ; Move to absolute position (100, 100)
  ```

- **G91** - Relative positioning mode
  ```gcode
  G91
  G00 X10 Y10    ; Move 10mm in X and 10mm in Y from current position
  ```

### Feed Rate Modes
- **G94** - Units per minute mode (default)
  ```gcode
  G94
  ```

- **G93** - Inverse time mode
  ```gcode
  G93
  ```

### Spindle Control
- **M03** - Spindle on, clockwise
  ```gcode
  M03 S1500      ; Turn spindle on at 1500 RPM/speed
  ```

- **M04** - Spindle on, counter-clockwise
  ```gcode
  M04 S1200      ; Turn spindle on CCW at 1200 RPM/speed
  ```

- **M05** - Spindle off
  ```gcode
  M05
  ```

### Parameters
- **X** - X-axis position (mm)
- **Y** - Y-axis position (mm)
- **F** - Feedrate (mm/min)
- **S** - Spindle speed (RPM or percentage)
- **P** - Dwell time (seconds)

## Usage Example

```c
#include "gcode.h"

int main() {
    gcode_state_t gc;
    
    // Initialize the G-code state
    gcode_init(&gc);
    
    // Process G-code commands
    gcode_process_line(&gc, "G00 X0 Y0");          // Move to origin
    gcode_process_line(&gc, "M03 S1500");          // Start spindle at 1500
    gcode_process_line(&gc, "G01 X50 Y0 F200");    // Engrave to X=50
    gcode_process_line(&gc, "G01 X50 Y50");        // Engrave to Y=50
    gcode_process_line(&gc, "M05");                // Stop spindle
    
    // Query current state
    float x, y;
    gcode_get_position(&gc, &x, &y);
    printf("Current position: X=%.2f Y=%.2f\n", x, y);
    printf("Feedrate: %.2f mm/min\n", gcode_get_feedrate(&gc));
    
    return 0;
}
```

## Typical 2D Engraving Workflow

```gcode
G90              ; Absolute positioning mode
G00 X0 Y0        ; Move to start position (rapid)
M03 S1500        ; Turn on spindle at 1500
G01 X50 Y0 F200  ; Engrave line 1 (feedrate 200 mm/min)
G01 X50 Y50      ; Engrave line 2
G01 X0 Y50       ; Engrave line 3
G01 X0 Y0        ; Engrave line 4
M05              ; Turn off spindle
G00 X0 Y0        ; Return to home
```

## Integration with Existing Modules

The G-code module is designed to integrate with:

- **Protocol Layer** (`protocol.h/c`) - Receives normalized command lines
- **Planner** (`planner.h/c`) - Sends motion commands for execution
- **Kinematics** (`kinematics.h/c`) - Coordinate transformation (e.g., CoreXY)
- **HAL** (`hal.h`) - Hardware control for spindle and motion

## API Reference

### Initialization
- `void gcode_init(gcode_state_t *gc)` - Initialize G-code state
- `void gcode_reset(gcode_state_t *gc)` - Reset to default state

### Parsing and Execution
- `gcode_status_t gcode_parse_line(const char *line, gcode_block_t *block)` - Parse a line
- `gcode_status_t gcode_execute_block(gcode_state_t *gc, const gcode_block_t *block)` - Execute a block
- `gcode_status_t gcode_process_line(gcode_state_t *gc, const char *line)` - Parse and execute

### State Query
- `void gcode_get_position(const gcode_state_t *gc, float *out_x, float *out_y)` - Get position
- `float gcode_get_feedrate(const gcode_state_t *gc)` - Get current feedrate
- `float gcode_get_spindle_speed(const gcode_state_t *gc)` - Get spindle speed
- `gcode_spindle_state_t gcode_get_spindle_state(const gcode_state_t *gc)` - Get spindle state

### Error Handling
- `const char *gcode_status_string(gcode_status_t status)` - Get error message

## Error Codes

- `GCODE_OK` - Success
- `GCODE_ERR_MISSING_PARAM` - Required parameter missing
- `GCODE_ERR_INVALID_PARAM` - Invalid parameter value
- `GCODE_ERR_UNKNOWN_CMD` - Unknown M-code
- `GCODE_ERR_UNSUPPORTED_CMD` - Unsupported G-code
- `GCODE_ERR_INVALID_TARGET` - Invalid target position
- `GCODE_ERR_OVERFLOW` - Buffer overflow

## Testing

Run the comprehensive test suite:

```bash
cd test
make clean
make run
```

The test suite includes:
- Initialization tests
- Parsing tests
- Motion execution tests
- Spindle control tests
- Coordinate mode tests
- Feed rate mode tests
- Error handling tests
- Complete workflow tests

## Notes

- This implementation is optimized for 2D engraving (X, Y axes only)
- Z-axis commands are not supported as per design requirements
- The module uses modal state management (commands retain their effect)
- Feedrate must be set before G01 commands (or use default 100 mm/min)
- Position tracking is maintained internally but actual motion requires planner integration
