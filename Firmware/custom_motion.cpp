/**
 * custom_motion.cpp
 *
 * Motion API implementation for the ToolIndexer firmware.
 *
 * Design notes:
 *  - plan_buffer_line() takes positions in mm and internally converts to steps.
 *  - st_synchronize() blocks until the stepper ISR drains the planner buffer.
 *  - Homing drives the axis at slow feedrate until the endstop triggers
 *    (detected via stepper.cpp's endstop_hit bitmask), then backs off.
 *  - ESTOP disables the stepper ISR timer via DISABLE_STEPPER_DRIVER_INTERRUPT
 *    and disables the motor enable pins.
 */

#include "custom_motion.h"
#include "custom_sensor.h"
#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "tmc2130.h"
#include "ConfigurationStore.h"
#include <math.h>

// Current logical position in mm (maintained locally; mirrors planner state)
static float current_pos[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};

// Currently selected tool: 0=none/unknown, 1=A, 2=B, 3=C
static uint8_t current_tool = 0;

// Feedrate used internally when not specified (mm/min)
#define DEFAULT_MOVE_FEEDRATE  3000.0f

// NEMA17 1.8°/step motor: 200 full steps per revolution
#define X_MOTOR_FULL_STEPS_PER_REV  200

// Back-off distance after hitting an endstop during homing (mm)
#define HOMING_BACKOFF_MM  2.0f

// PINDA center-finding scan parameters
#define PINDA_SCAN_FEEDRATE  300.0f   // deg/min (slow for edge accuracy)
#define PINDA_MAX_SCAN_DEG   400.0f   // abort after one full rev + margin

// ---------------------------------------------------------------------------
// homing_feedrate[] is defined as constexpr in Marlin.h — use it directly

void motion_init()
{
    plan_init();

    // Compute max_acceleration_steps_per_s2[] from cs.max_acceleration_mm_per_s2
    // and cs.axis_steps_per_mm. plan_init() does NOT do this — without it the
    // planner uses zero acceleration, resulting in extremely slow motion.
    // update_mode_profile() also selects normal vs silent feedrate/accel arrays.
    update_mode_profile();

    // Set planner position to 0 on all axes
    long zero[NUM_AXIS] = {0, 0, 0, 0};
    st_set_position(zero);

    for (uint8_t i = 0; i < NUM_AXIS; i++)
        current_pos[i] = 0.0f;
}

// ---------------------------------------------------------------------------

static void home_single_axis(uint8_t axis)
{
    if (axis >= NUM_AXIS) return;

    const float fast_rate = homing_feedrate[axis];  // mm/min
    if (fast_rate <= 0.0f) return;                  // E axis skipped

    // Step 1: configure TMC2130 stallGuard for this axis and enable endstop
    // checking in the stepper ISR. With TMC2130_SG_HOMING defined, the ISR
    // substitutes the TMC2130 DIAG pin (stall output) for the physical endstop
    // pin on axes whose bit is set in tmc2130_sg_homing_axes_mask.
    const uint8_t axis_mask = (1 << axis);
    tmc2130_home_enter(axis_mask);
    enable_endstops(true);

    // Step 2: drive a distance larger than the axis travel toward the stop.
    // The stallGuard stall triggers the DIAG pin, the ISR treats it as an
    // endstop hit and aborts the block — st_synchronize() then returns.
    float target[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++) target[i] = current_pos[i];

    const float travel[NUM_AXIS] = {
        (float)(X_MAX_POS - X_MIN_POS + 10),
        (float)(Y_MAX_POS - Y_MIN_POS + 10),
        (float)(Z_MAX_POS - Z_MIN_POS + 10),
        0.0f
    };
    target[axis] = current_pos[axis] - travel[axis];

    plan_buffer_line(target[X_AXIS], target[Y_AXIS],
                     target[Z_AXIS], target[E_AXIS],
                     fast_rate / 60.0f,   // planner wants mm/s
                     0 /*extruder*/);
    st_synchronize();

    // Step 3: restore normal TMC2130 operation and disable endstop checking.
    tmc2130_home_exit();
    enable_endstops(false);

    // Step 4: back off so the motor is no longer stalled.
    target[axis] = current_pos[axis] + HOMING_BACKOFF_MM;
    plan_buffer_line(target[X_AXIS], target[Y_AXIS],
                     target[Z_AXIS], target[E_AXIS],
                     fast_rate / 60.0f,
                     0);
    st_synchronize();

    // Step 5: declare home — axis is at position 0.
    current_pos[axis] = 0.0f;
    long steps[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        steps[i] = lround(current_pos[i] * cs.axis_steps_per_mm[i]);
    st_set_position(steps);
    plan_set_position(current_pos[X_AXIS], current_pos[Y_AXIS],
                     current_pos[Z_AXIS], current_pos[E_AXIS]);
}

static bool motion_home_pinda_x()
{
    float spm = cs.axis_steps_per_mm[X_AXIS];
    if (spm < 1.0f) spm = 200.0f;

    const float steps_per_sec = (PINDA_SCAN_FEEDRATE / 60.0f) * spm;
    const uint16_t half_us = (uint16_t)constrain(500000.0f / steps_per_sec, 2.0f, 65535.0f);
    const long max_steps = (long)(spm * PINDA_MAX_SCAN_DEG);

    st_synchronize();
    WRITE(X_ENABLE_PIN, X_ENABLE_ON);  // ensure motor is driven before bypassing ISR
    DISABLE_STEPPER_DRIVER_INTERRUPT();

    // If already in the non-triggered gap, back off until PINDA triggers
    if (!sensor_read_pinda()) {
        WRITE(X_DIR_PIN, INVERT_X_DIR);
        delayMicroseconds(2);
        for (long i = 0; i < max_steps; i++) {
            WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
            delayMicroseconds(half_us);
            WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
            delayMicroseconds(half_us);
            if (sensor_read_pinda()) break;
        }
    }

    // Scan forward, find falling edge (enter gap) then rising edge (exit gap)
    WRITE(X_DIR_PIN, !INVERT_X_DIR);
    delayMicroseconds(2);

    long enter_step = -1, exit_step = -1;
    for (long i = 0; i < max_steps; i++) {
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        delayMicroseconds(half_us);
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        delayMicroseconds(half_us);

        if (enter_step < 0 && !sensor_read_pinda()) { enter_step = i; continue; }
        if (enter_step >= 0 && sensor_read_pinda())  { exit_step  = i; break; }
    }

    ENABLE_STEPPER_DRIVER_INTERRUPT();

    if (enter_step < 0 || exit_step < 0) return false;

    // We are now at exit_step; back up to the midpoint
    const long back_steps = (exit_step - enter_step) / 2;
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    WRITE(X_DIR_PIN, INVERT_X_DIR);
    delayMicroseconds(2);
    for (long i = 0; i < back_steps; i++) {
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
        delayMicroseconds(half_us);
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        delayMicroseconds(half_us);
    }
    ENABLE_STEPPER_DRIVER_INTERRUPT();

    // Declare center as position 0°
    current_pos[X_AXIS] = 0.0f;
    long sp[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        sp[i] = lround(current_pos[i] * cs.axis_steps_per_mm[i]);
    st_set_position(sp);
    plan_set_position(current_pos[X_AXIS], current_pos[Y_AXIS],
                      current_pos[Z_AXIS], current_pos[E_AXIS]);
    return true;
}

bool motion_home(uint8_t axes_mask)
{
    bool ok = true;
    if (axes_mask & AXIS_X) { if (!motion_home_pinda_x()) ok = false; }
    if (axes_mask & AXIS_Y) home_single_axis(Y_AXIS);
    if (axes_mask & AXIS_Z) home_single_axis(Z_AXIS);
    // E cannot be homed
    return ok;
}

// ---------------------------------------------------------------------------

void motion_move(float x_mm, float y_mm, float z_mm, float e_mm,
                 float feedrate_mm_min)
{
    // NAN means "stay at current position"
    if (!isnan(x_mm)) current_pos[X_AXIS] = x_mm;
    if (!isnan(y_mm)) current_pos[Y_AXIS] = y_mm;
    if (!isnan(z_mm)) current_pos[Z_AXIS] = z_mm;
    if (!isnan(e_mm)) current_pos[E_AXIS] = e_mm;

    if (feedrate_mm_min <= 0.0f) feedrate_mm_min = DEFAULT_MOVE_FEEDRATE;

    plan_buffer_line(current_pos[X_AXIS], current_pos[Y_AXIS],
                     current_pos[Z_AXIS], current_pos[E_AXIS],
                     feedrate_mm_min / 60.0f,  // planner wants mm/s
                     0 /*extruder*/);
}

// ---------------------------------------------------------------------------

void motion_rotate_x(float delta_mm, float feedrate_mm_min)
{
    if (feedrate_mm_min <= 0.0f) feedrate_mm_min = homing_feedrate[X_AXIS];
    current_pos[X_AXIS] += delta_mm;
    plan_buffer_line(current_pos[X_AXIS], current_pos[Y_AXIS],
                     current_pos[Z_AXIS], current_pos[E_AXIS],
                     feedrate_mm_min / 60.0f,
                     0 /*extruder*/);
}

void motion_rotate_x_deg(float degrees, float feedrate_mm_min)
{
    if (feedrate_mm_min <= 0.0f) feedrate_mm_min = homing_feedrate[X_AXIS];

    // Use the calibrated axis_steps_per_mm value as steps-per-degree for X.
    // This accounts for the full drive chain (motor + gearbox) and is set via
    // DEFAULT_AXIS_STEPS_PER_UNIT[X] in ToolIndexer.h.
    float spm = cs.axis_steps_per_mm[X_AXIS];
    if (spm < 1.0f) spm = 200.0f; // guard against blank EEPROM; matches DEFAULT_AXIS_STEPS_PER_UNIT[X]

    const long steps = lround(fabsf(degrees) * spm);
    if (steps == 0) return;

    const float steps_per_sec = (feedrate_mm_min / 60.0f) * spm;
    const uint16_t half_us = (uint16_t)constrain(500000.0f / steps_per_sec, 2.0f, 65535.0f);

    // Drain any queued planner moves, then take direct control of the step/dir pins.
    st_synchronize();
    DISABLE_STEPPER_DRIVER_INTERRUPT();

    // Positive degrees = away from home endstop (INVERT_X_DIR = 1 on ToolIndexer)
    WRITE(X_DIR_PIN, degrees > 0.0f ? !INVERT_X_DIR : INVERT_X_DIR);
    delayMicroseconds(2); // TMC2130 minimum DIR setup time before first step

    for (long i = 0; i < steps; i++) {
        WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN); // rising edge → TMC2130 advances one microstep
        delayMicroseconds(half_us);
        WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
        delayMicroseconds(half_us);
    }

    // current_pos[X_AXIS] tracks position in degrees (1 unit = 1 degree).
    // st_set_position converts back to steps via axis_steps_per_mm.
    current_pos[X_AXIS] += degrees;
    long sp[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        sp[i] = lround(current_pos[i] * cs.axis_steps_per_mm[i]);
    st_set_position(sp);
    plan_set_position(current_pos[X_AXIS], current_pos[Y_AXIS],
                      current_pos[Z_AXIS], current_pos[E_AXIS]);

    ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// ---------------------------------------------------------------------------

void motion_wait()
{
    st_synchronize();
}

// ---------------------------------------------------------------------------

void motion_estop()
{
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    quickStop();
    // Disable all motor drivers (active-low enable pins)
    WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
    WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
    WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
    WRITE(E0_ENABLE_PIN, !E_ENABLE_ON);
}

void motion_enable()
{
    // Re-enable motor drivers
    WRITE(X_ENABLE_PIN, X_ENABLE_ON);
    WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
    WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
    WRITE(E0_ENABLE_PIN, E_ENABLE_ON);

    // Reset the stepper timer before re-enabling to prevent a spurious overflow
    st_reset_timer();
    ENABLE_STEPPER_DRIVER_INTERRUPT();
}

// ---------------------------------------------------------------------------

uint8_t motion_get_current_tool() { return current_tool; }

bool motion_go_to_tool(uint8_t tool)
{
    const float tool_angles[4] = { 0.0f, TOOL_A_DEG, TOOL_B_DEG, TOOL_C_DEG };
    if (tool < 1 || tool > 3) return false;

    current_tool = 0;  // unknown until motion succeeds

    // Move to the absolute target angle from current tracked position.
    // Caller is responsible for homing X before the first TOOL command so
    // current_pos[X_AXIS] is a reliable reference.
    float delta = tool_angles[tool] - current_pos[X_AXIS];
    if (delta != 0.0f)
        motion_rotate_x_deg(delta, DEFAULT_MOVE_FEEDRATE);

    // Verify position: at each tool holder centre the PINDA sits in the
    // non-metal gap, so it must NOT be triggered.
    if (sensor_read_pinda()) return false;

    current_tool = tool;
    return true;
}

float motion_get_position_mm(uint8_t axis)
{
    if (axis >= NUM_AXIS) return 0.0f;
    return current_pos[axis];
}

float motion_get_steps_per_unit(uint8_t axis)
{
    if (axis >= NUM_AXIS) return 0.0f;
    return cs.axis_steps_per_mm[axis];
}

bool motion_set_steps_per_unit(uint8_t axis, float steps_per_mm)
{
    if (axis >= NUM_AXIS || steps_per_mm <= 0.0f) return false;
    cs.axis_steps_per_mm[axis] = steps_per_mm;
    return true;
}
