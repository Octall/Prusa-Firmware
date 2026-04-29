#pragma once
/**
 * custom_command.h
 *
 * Line-based ASCII serial command protocol for the ToolIndexer firmware.
 *
 * Commands (case-insensitive, one per line, terminated with '\n'):
 *
 *   HOME [X] [Y] [Z] [ALL]
 *       Home the specified axes using their min endstops.
 *       Returns: OK
 *
 *   MOVE [X<mm>] [Y<mm>] [Z<mm>] [E<mm>] [F<mm/min>]
 *       Queue a linear move. Omitted axes stay at current position.
 *       Returns: OK
 *
 *   WAIT
 *       Block until the motion buffer is empty.
 *       Returns: OK
 *
 *   ESTOP
 *       Emergency stop — disables stepper drivers immediately.
 *       Returns: OK
 *
 *   ENABLE
 *       Re-enable stepper drivers after ESTOP.
 *       Returns: OK
 *
 *   READ IR
 *       Read the IR filament/tool-present sensor.
 *       Returns: VALUE 0  or  VALUE 1
 *
 *   READ PINDA
 *       Read the SuperPINDA probe (Z_MIN_PIN).
 *       Returns: VALUE 0  or  VALUE 1
 *
 *   READ ENDSTOP X|Y|Z|ALL
 *       Read endstop state(s). ALL returns a bitmask (bit0=X, bit1=Y, bit2=Z).
 *       Returns: VALUE <n>
 *
 *   STATUS
 *       Report current axis positions and sensor states.
 *       Returns: multi-line block terminated by OK
 *
 *   LCD <message>
 *       Display a message (up to 20 chars) on LCD row 3.
 *       Returns: OK
 *
 * All commands return "OK\n" on success or "ERROR <reason>\n" on failure.
 */

/** Initialise the command subsystem (call once from setup()). */
void command_init();

/**
 * Process pending serial input. Non-blocking.
 * Call every loop() iteration.
 */
void command_process();
