#pragma once
/**
 * custom_display.h
 *
 * Thin LCD abstraction for the ToolIndexer firmware.
 * Wraps lcd.cpp's hardware driver to provide a simple status screen:
 *
 *   Row 0: [status/alert message, up to 20 chars]
 *   Row 1: X:<pos>  Y:<pos>
 *   Row 2: Z:<pos>  E:<pos>
 *   Row 3: [user-settable message via LCD command]
 *
 * Updates are throttled to LCD_UPDATE_INTERVAL (100 ms) by lcd.cpp.
 */

/**
 * Initialise the display. Call once from setup(), after lcd_init().
 * Sets the lcd_lcdupdate_func callback so lcd_update() drives our screen.
 */
void display_init();

/**
 * Drive periodic display refresh. Call every loop() iteration.
 * Non-blocking; internally throttled by lcd.cpp timing.
 */
void display_update();

/**
 * Set the top status line (row 0). Truncated to 20 characters.
 * Use this for transient messages like "Homing..." or "ESTOP".
 */
void display_set_status(const char *msg);

/**
 * Set the bottom user row (row 3). Truncated to 20 characters.
 * Written by the LCD serial command.
 */
void display_set_user_row(const char *msg);
