# System State Management Module

This module provides comprehensive state management for a CNC machine controller, integrating with the existing G-code parser, planner, kinematics, and HAL layers.

## Features

### State Management
- **System States**: IDLE, RUNNING, HOLD, ALARM, HOMING, CHECK, SLEEP, DOOR
- **Alarm Types**: Hard limit, soft limit, e-stop, probe fail, homing fail, overflow, spindle stall
- Safe state transitions with validation
- Comprehensive state query functions

### Position Tracking
- Machine coordinate system (absolute positions)
- Work coordinate system (relative to work offset)
- Support for G92 and G10 L2 work offset commands
- Real-time position reporting

### Safety Features
- Hard limit switch monitoring
- Soft limit checking
- Emergency stop handling
- Automatic motion disable on alarm
- Spindle safety shutdown

### Status Reporting
- grbl-compatible status report format
- Machine and work position reporting
- Feed rate and spindle speed reporting
- Alarm status indication

### Motion Control
- Feed hold (pause motion)
- Cycle start (resume motion)
- Soft reset
- Homing cycle support

## API Overview

### Initialization
```c
system_context_t sys;
system_init(&sys);  // Initialize all subsystems
```

### G-code Processing
```c
system_process_line(&sys, "G00 X10 Y20");  // Process a G-code line
```

### State Management
```c
system_state_t state = system_get_state(&sys);
bool is_idle = system_is_idle(&sys);
system_set_state(&sys, SYS_STATE_RUNNING);
```

### Alarm Handling
```c
system_trigger_alarm(&sys, SYS_ALARM_HARD_LIMIT);
bool is_alarmed = system_is_alarmed(&sys);
system_clear_alarm(&sys);
```

### Realtime Commands
```c
system_feed_hold(&sys);     // Pause motion
system_cycle_start(&sys);   // Resume motion
system_soft_reset(&sys);    // Reset system
```

### Position Management
```c
float x, y, z;
system_get_machine_position(&sys, &x, &y, &z);
system_get_work_position(&sys, &x, &y, &z);
system_set_work_offset(&sys, 5.0f, 10.0f, 0.0f);
```

### Status Reporting
```c
char buf[256];
system_get_status_report(&sys, buf, sizeof(buf));
// Output: <Idle|MPos:0.000,0.000,0.000|WPos:0.000,0.000,0.000|F:100.0|S:0>
```

### Homing
```c
system_start_homing(&sys, 0x03);  // Home X and Y axes
bool is_homed = system_is_homed(&sys);
```

### Soft Limits
```c
system_set_soft_limits_enabled(&sys, true);
bool valid = system_check_soft_limits(&sys, x, y, z);
```

## Integration

The system state module integrates with:

1. **G-code Parser** (`gcode.h`): Processes G-code commands and maintains modal state
2. **Motion Planner** (`planner.h`): Manages motion queue for trajectory planning
3. **Kinematics** (`kinematics.h`): Handles coordinate transformations
4. **HAL** (`hal.h`): Interfaces with hardware (steppers, spindle, limits)

## Example Usage

See `examples/example_usage.c` for a complete working example demonstrating:
- System initialization
- G-code processing
- State transitions
- Alarm handling
- Position management
- Status reporting
- Homing
- Soft limits

Build and run the example:
```bash
gcc -Isrc -o examples/example_usage examples/example_usage.c examples/hal_mock.c lib/libgrblcore.a -lm
./examples/example_usage
```

## Testing

The module includes comprehensive unit tests in `test/system_state_test.c`:

```bash
# Build the library
make clean && make all

# Build and run tests
gcc -Isrc -o test/system_state_test test/system_state_test.c lib/libgrblcore.a -lm
./test/system_state_test
```

All 11 tests validate:
- ✓ System initialization
- ✓ State transitions
- ✓ Alarm handling
- ✓ Feed hold and resume
- ✓ Soft reset
- ✓ Position management
- ✓ Status reporting
- ✓ State string conversion
- ✓ Homing
- ✓ Soft limits
- ✓ Idle state detection

## Design Decisions

### No Protocol Layer Embedding
The system_state module does not directly embed the protocol layer to avoid circular dependencies. Instead:
- Protocol layer calls `system_process_line()` when a complete G-code line is ready
- Higher-level code manages the protocol instance separately
- This maintains clean separation of concerns

### Minimal Dynamic Allocation
- Uses static allocation for all critical data structures
- No malloc/free in main execution paths
- Suitable for embedded systems with limited heap

### Safety-Critical Design
- All state transitions are validated
- Alarms immediately disable motion and spindle
- Limit switches are checked during motion
- E-stop has highest priority

## Future Enhancements

Potential improvements for future versions:
- [ ] Settings/EEPROM integration
- [ ] Probe cycle support
- [ ] Tool length offset (TLO)
- [ ] Multiple work coordinate systems (G54-G59)
- [ ] Feed rate override
- [ ] Spindle speed override
- [ ] Jog mode implementation
- [ ] Sleep mode with auto-wake
- [ ] Door safety interlock implementation

## License

This module is part of the CNC_machine_Proj repository.
