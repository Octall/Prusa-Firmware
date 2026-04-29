/**
 * custom_sensor.cpp
 *
 * Sensor abstraction for the ToolIndexer firmware.
 * All reads use fastio.h READ() which handles AVR port atomicity.
 */

#include "custom_sensor.h"
#include "Marlin.h"      // fastio.h, pins.h pulled in via Marlin.h
#include "fastio.h"

// Endstop inversion flags from the variant header
// 0 = active-low (triggered = pin LOW), 1 = active-high (triggered = pin HIGH)
#ifndef X_MIN_ENDSTOP_INVERTING
#  define X_MIN_ENDSTOP_INVERTING 0
#endif
#ifndef Y_MIN_ENDSTOP_INVERTING
#  define Y_MIN_ENDSTOP_INVERTING 0
#endif
#ifndef Z_MIN_ENDSTOP_INVERTING
#  define Z_MIN_ENDSTOP_INVERTING 0
#endif

// The IR sensor is active-low on the Einsy (pin pulled high, goes low when object present)
#define IR_SENSOR_ACTIVE_LOW 1

void sensor_init()
{
    SET_INPUT(X_MIN_PIN);
    SET_INPUT(Y_MIN_PIN);
    SET_INPUT(Z_MIN_PIN);
    SET_INPUT(IR_SENSOR_PIN);

    // Enable pull-ups (the Einsy hardware already has external pull-ups on
    // endstop lines, but setting PULLUP doesn't hurt)
#ifdef ENDSTOPPULLUPS
    WRITE(X_MIN_PIN, HIGH);
    WRITE(Y_MIN_PIN, HIGH);
    WRITE(Z_MIN_PIN, HIGH);
#endif
    WRITE(IR_SENSOR_PIN, HIGH);
}

bool sensor_read_ir()
{
    // IR sensor: signal goes LOW when filament/tool is present
    // Return true when object detected
#if IR_SENSOR_ACTIVE_LOW
    return !READ(IR_SENSOR_PIN);
#else
    return  READ(IR_SENSOR_PIN);
#endif
}

bool sensor_read_pinda()
{
    // SuperPINDA on Z_MIN_PIN — triggered state depends on inverting flag
#if Z_MIN_ENDSTOP_INVERTING
    return READ(Z_MIN_PIN);   // active-high
#else
    return !READ(Z_MIN_PIN);  // active-low
#endif
}

bool sensor_read_endstop_x()
{
#if X_MIN_ENDSTOP_INVERTING
    return READ(X_MIN_PIN);
#else
    return !READ(X_MIN_PIN);
#endif
}

bool sensor_read_endstop_y()
{
#if Y_MIN_ENDSTOP_INVERTING
    return READ(Y_MIN_PIN);
#else
    return !READ(Y_MIN_PIN);
#endif
}

bool sensor_read_endstop_z()
{
    return sensor_read_pinda();
}

uint8_t sensor_read_endstops_all()
{
    return (sensor_read_endstop_x() ? 1u : 0u)
         | (sensor_read_endstop_y() ? 2u : 0u)
         | (sensor_read_endstop_z() ? 4u : 0u);
}
