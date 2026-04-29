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
#include "ConfigurationStore.h"
#include <math.h>

// Current logical position in mm (maintained locally; mirrors planner state)
static float current_pos[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};

// Feedrate used internally when not specified (mm/min)
#define DEFAULT_MOVE_FEEDRATE  3000.0f

// Back-off distance after hitting an endstop during homing (mm)
#define HOMING_BACKOFF_MM  2.0f

// ---------------------------------------------------------------------------
// homing_feedrate[] is defined as constexpr in Marlin.h — use it directly

void motion_init()
{
    plan_init();

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

    // Step 1: drive toward min endstop
    enable_endstops(true);

    // Move a large negative distance — the endstop will stop us
    float target[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++) target[i] = current_pos[i];

    // Choose a distance guaranteed to exceed axis travel
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

    // Step 2: back off
    enable_endstops(false);
    target[axis] = current_pos[axis] + HOMING_BACKOFF_MM;
    plan_buffer_line(target[X_AXIS], target[Y_AXIS],
                     target[Z_AXIS], target[E_AXIS],
                     fast_rate / 60.0f,
                     0);
    st_synchronize();

    // Step 3: declare home
    current_pos[axis] = 0.0f;
    long steps[NUM_AXIS];
    for (uint8_t i = 0; i < NUM_AXIS; i++)
        steps[i] = lround(current_pos[i] * cs.axis_steps_per_mm[i]);
    st_set_position(steps);
    plan_set_position_curposXYZE();
}

void motion_home(uint8_t axes_mask)
{
    if (axes_mask & AXIS_X) home_single_axis(X_AXIS);
    if (axes_mask & AXIS_Y) home_single_axis(Y_AXIS);
    if (axes_mask & AXIS_Z) home_single_axis(Z_AXIS);
    // E cannot be homed
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

float motion_get_position_mm(uint8_t axis)
{
    if (axis >= NUM_AXIS) return 0.0f;
    return current_pos[axis];
}
