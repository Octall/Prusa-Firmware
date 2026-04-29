#pragma once
/**
 * custom_sensor.h
 *
 * Sensor abstraction for the ToolIndexer firmware.
 * Provides read access to:
 *   - IR filament/tool-present sensor  (IR_SENSOR_PIN, digital pin 62)
 *   - SuperPINDA probe trigger          (Z_MIN_PIN, digital pin 10)
 *   - X, Y, Z min endstops             (X_MIN_PIN 12, Y_MIN_PIN 11, Z_MIN_PIN 10)
 *
 * All reads are direct GPIO using fastio.h macros — no ADC needed.
 *
 * Note: SuperPINDA and Z endstop share the same physical pin (Z_MIN_PIN).
 *       sensor_read_pinda() and sensor_read_endstop_z() are aliases.
 */

#include <stdint.h>

/** Initialise sensor pins as inputs. */
void sensor_init();

/**
 * Read the IR filament/tool-present sensor.
 * @return true  when the sensor detects an object (logic depends on inverting flag).
 */
bool sensor_read_ir();

/**
 * Read the SuperPINDA probe.
 * @return true  when the probe is triggered (object detected below).
 * Same physical pin as Z_MIN endstop.
 */
bool sensor_read_pinda();

/** Read X-axis minimum endstop. Returns true when triggered. */
bool sensor_read_endstop_x();

/** Read Y-axis minimum endstop. Returns true when triggered. */
bool sensor_read_endstop_y();

/**
 * Read Z-axis minimum endstop / SuperPINDA.
 * Identical to sensor_read_pinda() — two names for the same pin.
 */
bool sensor_read_endstop_z();

/**
 * Bitmask of all endstop states packed as:
 *   bit 0 = X_MIN, bit 1 = Y_MIN, bit 2 = Z_MIN
 */
uint8_t sensor_read_endstops_all();
