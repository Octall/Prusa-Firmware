#ifndef CONFIGURATION_PRUSA_H
#define CONFIGURATION_PRUSA_H

/*
 * ToolIndexer variant — Prusa MK3S Einsy Rambo repurposed as a multi-tool indexer
 * for autonomous attachment to a Kinova robotic arm.
 *
 * Retains: X/Y/Z/E stepper motors, LCD panel, IR sensor, SuperPINDA,
 *          endstops (for homing), serial/USB command interface.
 * Removed: all 3D-printing logic, temperature control, bed leveling, MMU2,
 *          SD card, power panic, calibration, fan control, thermal safety.
 */

#include <limits.h>
#include "printers.h"

/* ---------------------------------------------------
 * Hardware identity
 * --------------------------------------------------- */
#define MOTHERBOARD  BOARD_EINSY_1_0a
#define PRINTER_TYPE PRINTER_MK3S
#define PRINTER_NAME PRINTER_MK3S_NAME
#define PRINTER_NAME_ALTERNATE PRINTER_MK3_NAME
#define PRINTER_MMU_TYPE PRINTER_MK3S_MMU3
#define PRINTER_MMU_NAME PRINTER_MK3S_MMU3_NAME
#define CUSTOM_MENDEL_NAME "ToolIndexer"

/* ---------------------------------------------------
 * Axis settings — keep MK3S defaults so existing
 * mechanical calibration remains valid.
 * --------------------------------------------------- */
#define NUM_AXIS 4  // X, Y, Z, E

// X axis: steps per DEGREE (motor steps × microsteps × gear-ratio / 360).
// Calibrate by running ROTATE D<large> and measuring physical angle:
//   steps_per_deg = command_value × (200 × 16 / 360) / measured_degrees
// Empirical value from D2000 ≈ 60°: 2000 × 8.889 / 60 ≈ 296
#define DEFAULT_AXIS_STEPS_PER_UNIT   {143, 100, 3200/8, 280}

// Direction inverting (MK3S stock)
#define INVERT_X_DIR  1
#define INVERT_Y_DIR  0
#define INVERT_Z_DIR  1
#define INVERT_E0_DIR 0

// Endstop logic (0 = active-low, 1 = active-high)
#define X_MIN_ENDSTOP_INVERTING 0
#define Y_MIN_ENDSTOP_INVERTING 0
#define Z_MIN_ENDSTOP_INVERTING 0

// Home direction (all min)
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// Home positions
#define MANUAL_X_HOME_POS  0
#define MANUAL_Y_HOME_POS  0
#define MANUAL_Z_HOME_POS  0

// Axis travel limits
#define X_MIN_POS   0
#define X_MAX_POS 255
#define Y_MIN_POS   0
#define Y_MAX_POS 210
#define Z_MIN_POS   0
#define Z_MAX_POS 210

// Feedrates (mm/s)
#define DEFAULT_MAX_FEEDRATE         {200, 200, 12, 120}
#define DEFAULT_MAX_FEEDRATE_SILENT  {100, 100, 12, 120}

// Accelerations (mm/s²)
#define DEFAULT_MAX_ACCELERATION         {1000, 1000, 200, 5000}
#define DEFAULT_MAX_ACCELERATION_SILENT  { 960,  960, 200, 5000}
#define DEFAULT_ACCELERATION       1250
#define DEFAULT_RETRACT_ACCELERATION 1250
#define DEFAULT_TRAVEL_ACCELERATION  1250

// Jerk limits (mm/s)
#define DEFAULT_XJERK  10.0
#define DEFAULT_YJERK  10.0
#define DEFAULT_ZJERK   0.4
#define DEFAULT_EJERK   4.5

// Homing feedrate (mm/min)
#define HOMING_FEEDRATE {3000, 3000, 800, 0}

// Motion planner minimums
#define DEFAULT_MINIMUMFEEDRATE       0.0
#define DEFAULT_MINTRAVELFEEDRATE     0.0
#define DEFAULT_MINSEGMENTTIME     20000  // µs

// Manual move speeds (mm/min)
#define MANUAL_FEEDRATE {2700, 2700, 1000, 100}

/* ---------------------------------------------------
 * Silent mode limits (used by TMC2130 profiles)
 * --------------------------------------------------- */
#define SILENT_MAX_ACCEL_XY    960ul
#define SILENT_MAX_FEEDRATE_XY 100
#define NORMAL_MAX_ACCEL_XY   2500ul
#define NORMAL_MAX_FEEDRATE_XY 200

/* ---------------------------------------------------
 * Arc interpolation (M214 / G2 G3)
 * --------------------------------------------------- */
#define DEFAULT_MM_PER_ARC_SEGMENT    1.0f
#define DEFAULT_MIN_MM_PER_ARC_SEGMENT 0.1f
#define DEFAULT_N_ARC_CORRECTION       25
#define DEFAULT_MIN_ARC_SEGMENTS        0
#define DEFAULT_ARC_SEGMENTS_PER_SEC    0

/* ---------------------------------------------------
 * End-of-file section check (lang tooling)
 * --------------------------------------------------- */
#define END_FILE_SECTION 0

/* ---------------------------------------------------
 * Z-axis: keep motor always powered to prevent sag
 * --------------------------------------------------- */
#define Z_AXIS_ALWAYS_ON 1

/* ---------------------------------------------------
 * LCD button timing (required by lcd.cpp)
 * --------------------------------------------------- */
#define LONG_PRESS_TIME      1000  // ms for long-press detection
#define BUTTON_BLANKING_TIME  200  // ms debounce after button release

/* ---------------------------------------------------
 * Tool carousel angular positions
 * Degrees from PINDA home (gap centre) to each tool holder.
 * Adjust if the physical carousel spacing differs from 120°.
 * --------------------------------------------------- */
#define TOOL_A_DEG    0.0f
#define TOOL_B_DEG  120.0f
#define TOOL_C_DEG  -120.0f

/* ---------------------------------------------------
 * TMC2130 stepper driver settings (copied from MK3S.h)
 * --------------------------------------------------- */
#define TMC2130_USTEPS_XY   16
#define TMC2130_USTEPS_Z    16
#define TMC2130_USTEPS_E    32

#define TMC2130_PWM_GRAD_X  2
#define TMC2130_PWM_AMPL_X  230
#define TMC2130_PWM_AUTO_X  1
#define TMC2130_PWM_FREQ_X  2

#define TMC2130_PWM_GRAD_Y  2
#define TMC2130_PWM_AMPL_Y  235
#define TMC2130_PWM_AUTO_Y  1
#define TMC2130_PWM_FREQ_Y  2

#define TMC2130_PWM_GRAD_Z  4
#define TMC2130_PWM_AMPL_Z  200
#define TMC2130_PWM_AUTO_Z  1
#define TMC2130_PWM_FREQ_Z  2

#define TMC2130_PWM_GRAD_E  4
#define TMC2130_PWM_AMPL_E  240
#define TMC2130_PWM_AUTO_E  1
#define TMC2130_PWM_FREQ_E  2

#define TMC2130_PWM_GRAD_Ecool  84
#define TMC2130_PWM_AMPL_Ecool  43
#define TMC2130_PWM_AUTO_Ecool  0

#define TMC2130_TOFF_XYZ    3
#define TMC2130_TOFF_E      3

#define TMC2130_FCLK        12000000ul
#define TMC2130_PWM_DIV     512
#define TMC2130_PWM_CLK     (2 * TMC2130_FCLK / TMC2130_PWM_DIV)
#define TMC2130_THIGH       0
#define TMC2130_TPWMTHRS    0
#define TMC2130_TPWMTHRS_E  403

#define TMC2130_TCOOLTHRS_X 430
#define TMC2130_TCOOLTHRS_Y 430
#define TMC2130_TCOOLTHRS_Z 500
#define TMC2130_TCOOLTHRS_E 500

#define TMC2130_SG_THRS_X        3
#define TMC2130_SG_THRS_Y        3
#define TMC2130_SG_THRS_Z        4
#define TMC2130_SG_THRS_E        3
#define TMC2130_SG_THRS_HOME     {3, 3, TMC2130_SG_THRS_Z, TMC2130_SG_THRS_E}

#define TMC2130_CURRENTS_H       {16, 20, 35, 30}
#define TMC2130_CURRENTS_R       {16, 20, 35, 30}
#define TMC2130_CURRENTS_R_HOME  {8, 10, 20, 18}
#define TMC2130_CURRENTS_FARM    36

/* ---------------------------------------------------
 * Fan check — mirror Einsy hardware pin so fancheck.h
 * exposes fan_measuring (needed by planner.cpp).
 * Actual fan control is stubbed in custom_stubs.cpp.
 * --------------------------------------------------- */
#define FANCHECK
#define EXTRUDER_0_AUTO_FAN_PIN 8   // Einsy autofan pin (hardware present)

/* ---------------------------------------------------
 * Extruder temperature guard — set to 0 so the E axis
 * motor is always usable without heater active.
 * --------------------------------------------------- */
#define EXTRUDE_MINTEMP 0

/* ---------------------------------------------------
 * Mesh bed leveling constants — required by
 * mesh_bed_calibration.h (included by planner.cpp)
 * even though MBL is disabled. Actual MBL code is
 * compiled out because MESH_BED_LEVELING is not defined.
 * --------------------------------------------------- */
#define MESH_NUM_X_POINTS 3
#define MESH_NUM_Y_POINTS 3
#define X_PROBE_OFFSET_FROM_EXTRUDER 0
#define Y_PROBE_OFFSET_FROM_EXTRUDER 0

#endif // CONFIGURATION_PRUSA_H
