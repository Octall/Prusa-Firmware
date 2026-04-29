#pragma once
/**
 * custom_motion.h
 *
 * High-level motion API for the ToolIndexer firmware.
 * Wraps stepper.cpp / planner.cpp with simple mm-based calls.
 *
 * Axis indices match Marlin convention: X=0, Y=1, Z=2, E=3.
 */

#include <stdint.h>

/**
 * Bitmask for axis selection.
 * Usage: AXIS_X | AXIS_Z  etc.
 */
#define AXIS_X  (1u << 0)
#define AXIS_Y  (1u << 1)
#define AXIS_Z  (1u << 2)
#define AXIS_E  (1u << 3)
#define AXIS_ALL (AXIS_X | AXIS_Y | AXIS_Z | AXIS_E)

/**
 * Initialise the motion subsystem (called once from setup()).
 * Sets current position to 0 on all axes; does NOT home.
 */
void motion_init();

/**
 * Home one or more axes using their min endstops.
 * @param axes_mask  bitmask of AXIS_X / AXIS_Y / AXIS_Z (E cannot be homed)
 *
 * Drives each selected axis toward its min endstop at HOMING_FEEDRATE,
 * then backs off 2 mm and sets the planner position to 0.
 */
void motion_home(uint8_t axes_mask);

/**
 * Queue a linear move.
 * Pass NAN for any axis to leave it at its current position.
 * @param x_mm, y_mm, z_mm, e_mm  target positions in mm
 * @param feedrate_mm_min          travel speed in mm/min
 */
void motion_move(float x_mm, float y_mm, float z_mm, float e_mm,
                 float feedrate_mm_min);

/**
 * Rotate the X axis by a relative amount at the specified feedrate.
 * Calls with the same delta_mm accumulate correctly (relative, not absolute).
 * @param delta_mm        signed distance in mm (positive = away from home)
 * @param feedrate_mm_min travel speed in mm/min; 0 uses the X homing feedrate
 */
void motion_rotate_x(float delta_mm, float feedrate_mm_min = 0.0f);

/**
 * Rotate the X axis by a relative angle using motor-degree units.
 * Converts degrees to mm via the NEMA17 motor parameters and axis_steps_per_mm.
 * Accumulates correctly across repeated calls (relative, not absolute).
 * @param degrees         signed angle in degrees (positive = away from home)
 * @param feedrate_mm_min travel speed in mm/min; 0 uses the X homing feedrate
 */
void motion_rotate_x_deg(float degrees, float feedrate_mm_min = 0.0f);

/**
 * Block until the motion buffer is empty (all queued moves complete).
 */
void motion_wait();

/**
 * Emergency stop: immediately disable the stepper ISR and all motor drivers.
 * After ESTOP, call motion_enable() before any further moves.
 */
void motion_estop();

/**
 * Re-enable stepper drivers and ISR after an ESTOP.
 * Position state is preserved; re-home before relying on position accuracy.
 */
void motion_enable();

/**
 * Return current position in mm for one axis.
 * @param axis  0=X, 1=Y, 2=Z, 3=E
 */
float motion_get_position_mm(uint8_t axis);
