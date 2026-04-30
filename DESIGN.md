# ToolIndexer Firmware — Design Description

## Overview

The ToolIndexer is a repurposed Prusa MK3S+ 3D printer (Einsy Rambo board, ATmega2560) used as a motorised tool changer for a Kinova robotic arm. The X-axis stepper drives a rotary carousel that presents one of three tool holders to the robot. All 3D-printing logic has been stripped; the firmware retains only stepper control, TMC2130 driver management, the LCD panel, sensor reading, and a line-oriented serial command interface.

The firmware is built on the Prusa MK3S firmware codebase (tag 3.14.1) with a custom application layer replacing `Marlin_main.cpp`.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ATmega2560 @ 16 MHz |
| Board | Einsy Rambo 1.0a |
| Stepper drivers | TMC2130 × 4 (X, Y, Z, E) |
| X motor | NEMA17 1.8°/step — drives tool carousel via ~33:1 gearbox |
| PINDA sensor | SuperPINDA on Z_MIN_PIN (pin 10), active-low |
| IR sensor | Filament sensor on IR_SENSOR_PIN, active-low |
| LCD | 20×4 character, standard Prusa wiring |
| Serial | UART0 (MYSERIAL), 115200 baud |

---

## Axis Convention

Only the X axis is used for tool indexing. Y and Z motors are present but unused in normal operation.

| Axis | Unit | Notes |
|------|------|-------|
| X | degrees | `current_pos[X_AXIS]` tracks degrees, not mm. `cs.axis_steps_per_mm[X_AXIS]` is steps/degree. |
| Y, Z | mm | Standard linear axes, not used in tool indexing. |
| E | mm | Extruder — never homed, rarely used. |

### X Calibration

`DEFAULT_AXIS_STEPS_PER_UNIT[X]` (currently 200 in `Firmware/variants/ToolIndexer.h`) sets steps per degree and accounts for the full drive chain (motor microsteps × gear ratio / 360).

Calibration formula:
```
steps_per_deg = commanded_D_value × (200 × 16 / 360) / measured_physical_degrees
```

The value is stored in EEPROM and overrides the compile-time default on every boot. To push a new compile-time default to the device:
```
M92 X<new_value>
M500
```
Or clear EEPROM entirely and reflash.

Live value is visible via the `STATUS` command (`CONFIG X_STEPS_PER_DEG=...`).

---

## Firmware Architecture

```
custom_main.cpp       — setup() / loop()
├── custom_motion.cpp — stepper motion API
├── custom_sensor.cpp — PINDA, IR, endstop reads
├── custom_display.cpp— LCD status screen
└── custom_command.cpp— serial command parser / dispatcher
```

All custom files sit alongside the upstream Prusa sources. The build system selects the `ToolIndexer` variant, which compiles out printing, temperature, bed leveling, MMU2, SD card, and power panic subsystems.

### custom_main.cpp

Initialisation order (important — dependencies are strict):
1. Disable watchdog
2. Timer2 (`millis()`)
3. SPI (required by TMC2130)
4. LCD hardware
5. ADC
6. `motion_init()` — `plan_init()` + `update_mode_profile()`
7. `tmc2130_init()` — configure driver registers
8. `st_init()` — arm Timer1 COMPA stepper ISR
9. `sensor_init()` — configure GPIO inputs
10. `display_init()` / `command_init()`

`update_mode_profile()` must be called after `plan_init()` to populate `max_acceleration_steps_per_s2[]`; without it the planner uses zero acceleration and motion is extremely slow.

### custom_motion.cpp

Two motion modes co-exist:

**Planner mode** (`motion_move`, `motion_rotate_x`):
- Calls `plan_buffer_line()` → stepper ISR drains the buffer
- Feedrate in mm/min is divided by 60 before passing to the planner (which expects mm/s)
- `motion_wait()` blocks via `st_synchronize()` until the buffer is empty

**Direct step mode** (`motion_rotate_x_deg`, `motion_home_pinda_x`):
- Calls `DISABLE_STEPPER_DRIVER_INTERRUPT()` then bit-bangs `X_STEP_PIN` / `X_DIR_PIN`
- Used because the planner does not generate adequate step rates for this rotary axis at normal operating speeds
- `WRITE(X_ENABLE_PIN, X_ENABLE_ON)` must be called before stepping — the planner normally does this via `st_wake_up()`; direct mode bypasses that path
- After stepping, `st_set_position()` and `plan_set_position()` re-sync the planner and stepper counter to `current_pos[]`
- `ENABLE_STEPPER_DRIVER_INTERRUPT()` restores normal ISR control

`current_pos[X_AXIS]` is in degrees (1 unit = 1 degree). Steps = `current_pos[X] × cs.axis_steps_per_mm[X]`.

### X Homing (PINDA center-finding)

Home does **not** use the stallGuard endstop. Instead it finds the angular center of the non-triggered gap at each tool holder:

1. If PINDA is not triggered (already in a gap), reverse until PINDA triggers (move onto metal)
2. Scan forward slowly (`PINDA_SCAN_FEEDRATE = 300 deg/min`)
3. Record `enter_step` when PINDA drops (entering non-metal gap)
4. Record `exit_step` when PINDA rises (exiting non-metal gap)
5. Back up `(exit_step − enter_step) / 2` steps to the gap center
6. Declare position 0°

If no gap is found within `PINDA_MAX_SCAN_DEG = 400°`, return failure → serial reply `ERROR PINDA home failed`.

Y and Z still use stallGuard homing (`home_single_axis()`).

### custom_sensor.cpp

| Function | Pin | Active state |
|----------|-----|-------------|
| `sensor_read_pinda()` | Z_MIN_PIN (10) | `true` when pin LOW (`Z_MIN_ENDSTOP_INVERTING = 0`) |
| `sensor_read_ir()` | IR_SENSOR_PIN | `true` when pin LOW (active-low) |
| `sensor_read_endstop_x/y/z()` | X/Y/Z_MIN_PIN | `true` when triggered |

`sensor_read_endstop_z()` is an alias for `sensor_read_pinda()`.

### custom_command.cpp

Line-buffered ASCII over MYSERIAL. Lines are echoed character-by-character; a complete line dispatches a command and replies `OK\r\n` or `ERROR <reason>\r\n`.

All motion commands block until the move completes before sending `OK` (either via `motion_wait()` for planner moves, or because direct step loops are inherently blocking).

---

## Serial Command Reference

| Command | Arguments | Description |
|---------|-----------|-------------|
| `HOME` | `[X] [Y] [Z]` | Home axes. X uses PINDA center-finding; Y/Z use stallGuard. No args = all three. |
| `ROTATE` | `D<degrees> [F<deg/min>]` | Rotate X by relative degrees. Direct step mode. |
| `MOVE` | `[X<mm>] [Y<mm>] [Z<mm>] [E<mm>] [F<mm/min>]` | Absolute move via planner. NAN axis = unchanged. |
| `WAIT` | — | Block until motion buffer empty. |
| `ESTOP` | — | Immediately disable stepper ISR and all motor enables. |
| `ENABLE` | — | Re-enable motors and stepper ISR after ESTOP. |
| `READ` | `IR` \| `PINDA` \| `ENDSTOP [X\|Y\|Z]` | Read sensor value. Reply: `VALUE <0\|1>`. |
| `STATUS` | — | Print position, sensor states, and calibration values. |
| `LCD` | `<text>` | Write text to LCD row 3. |
| `TOOL` | `A` \| `B` \| `C` | Rotate to absolute tool angle then verify position via PINDA (must be in gap). Home X first for reliable results. LCD row 0 shows `Tool: <X>` on success. |

### STATUS response format
```
POS X<deg> Y<mm> Z<mm> E<mm>
SENSOR IR=<0|1> PINDA=<0|1> ENDSTOP_X=<0|1> ENDSTOP_Y=<0|1> ENDSTOP_Z=<0|1>
CONFIG X_STEPS_PER_DEG=<value>
TOOL <A|B|C|NONE>
OK
```

---

## Tool Positions

The carousel has three tool holders (A, B, C) spaced 120° apart. After PINDA homing sets position 0°, the firmware rotates to the following angles:

| Tool | Angle (deg) | Constant |
|------|-------------|----------|
| A | 0 | `TOOL_A_DEG` |
| B | 120 | `TOOL_B_DEG` |
| C | -120 | `TOOL_C_DEG` |

Constants are defined in `Firmware/variants/ToolIndexer.h`. Adjust them if the physical carousel spacing differs, then rebuild and reflash.

---

## Build System

```bash
./build.sh
```

On first run, `utils/bootstrap.py` downloads:
- `avr-gcc 7.3.0` (locked toolchain) into `.dependencies/`
- `prusa3dboards 1.0.6` (board support package) into `.dependencies/`

Subsequent runs skip the download (idempotent). Falls back to system `avr-gcc` if the locked toolchain is absent.

Output: `build/ToolIndexer_ENGLISH.hex`

### Flashing

The board uses the Prusa MK3S+ bootloader (`stk500v2`):
```bash
avrdude -p m2560 -c stk500v2 -P /dev/ttyACM0 -b 115200 -D \
  -U flash:w:build/ToolIndexer_ENGLISH.hex:i
```

### EEPROM

Compile-time defaults (e.g. `DEFAULT_AXIS_STEPS_PER_UNIT`) are written to EEPROM on first boot only. If the EEPROM already has values, those take precedence. To force new defaults: send `M502` (load defaults) then `M500` (save to EEPROM), or use `M92 X<value>` to set a specific axis value then `M500`.

---

## Key Files

| File | Role |
|------|------|
| `Firmware/variants/ToolIndexer.h` | Variant config — steps/unit, feedrates, TMC2130 settings |
| `Firmware/custom_main.cpp` | Entry point — setup() / loop() |
| `Firmware/custom_motion.cpp` | All motion logic |
| `Firmware/custom_motion.h` | Motion API |
| `Firmware/custom_command.cpp` | Serial command parser |
| `Firmware/custom_sensor.cpp` | PINDA, IR, endstop reads |
| `Firmware/custom_display.cpp/h` | LCD abstraction |
| `build.sh` | Build entry point |
| `cmake/AvrGcc.cmake` | Locked toolchain (bootstrap-managed) |
| `cmake/AnyAvrGcc.cmake` | System avr-gcc fallback |
