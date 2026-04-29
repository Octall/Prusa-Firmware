/**
 * custom_display.cpp
 *
 * LCD status screen for the ToolIndexer firmware.
 * Reads live step positions from stepper.h count_position[] and converts
 * them to mm using cs.axis_steps_per_mm[] from ConfigurationStore.h.
 */

#include "custom_display.h"
#include "lcd.h"
#include "stepper.h"         // count_position[], st_get_position_mm()
#include "ConfigurationStore.h" // cs
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>

#define DISPLAY_ROW_STATUS   0
#define DISPLAY_ROW_XY       1
#define DISPLAY_ROW_ZE       2
#define DISPLAY_ROW_USER     3

static char status_msg[LCD_WIDTH + 1]   = "ToolIndexer Ready   ";
static char user_row[LCD_WIDTH + 1]     = "                    ";

// Forward declaration — this is the function we register as lcd_lcdupdate_func
static void display_update_screen();

void display_init()
{
    lcd_update_enabled = 1;
    lcd_draw_update    = 2;  // force full redraw on first call
    lcd_lcdupdate_func = display_update_screen;
}

void display_update()
{
    lcd_update(0);
}

void display_set_status(const char *msg)
{
    strncpy(status_msg, msg, LCD_WIDTH);
    status_msg[LCD_WIDTH] = '\0';
    // Pad with spaces so old content is overwritten
    for (uint8_t i = strlen(status_msg); i < LCD_WIDTH; i++)
        status_msg[i] = ' ';
    status_msg[LCD_WIDTH] = '\0';
    lcd_draw_update = 1;
}

void display_set_user_row(const char *msg)
{
    strncpy(user_row, msg, LCD_WIDTH);
    user_row[LCD_WIDTH] = '\0';
    for (uint8_t i = strlen(user_row); i < LCD_WIDTH; i++)
        user_row[i] = ' ';
    user_row[LCD_WIDTH] = '\0';
    lcd_draw_update = 1;
}

// -------------------------------------------------------------------
// Internal: called by lcd_update() via the function pointer
// -------------------------------------------------------------------
static void display_update_screen()
{
    if (!lcd_draw_update) return;
    lcd_draw_update = 0;

    // Row 0: status message
    lcd_set_cursor(0, DISPLAY_ROW_STATUS);
    lcd_print(status_msg);

    // Row 1: X and Y positions in mm
    {
        float x_mm = (float)count_position[X_AXIS] / cs.axis_steps_per_mm[X_AXIS];
        float y_mm = (float)count_position[Y_AXIS] / cs.axis_steps_per_mm[Y_AXIS];
        char buf[LCD_WIDTH + 1];
        snprintf_P(buf, sizeof(buf), PSTR("X%7.2f  Y%7.2f"), (double)x_mm, (double)y_mm);
        lcd_set_cursor(0, DISPLAY_ROW_XY);
        lcd_print(buf);
    }

    // Row 2: Z and E positions in mm
    {
        float z_mm = (float)count_position[Z_AXIS] / cs.axis_steps_per_mm[Z_AXIS];
        float e_mm = (float)count_position[E_AXIS] / cs.axis_steps_per_mm[E_AXIS];
        char buf[LCD_WIDTH + 1];
        snprintf_P(buf, sizeof(buf), PSTR("Z%7.2f  E%7.2f"), (double)z_mm, (double)e_mm);
        lcd_set_cursor(0, DISPLAY_ROW_ZE);
        lcd_print(buf);
    }

    // Row 3: user-settable message
    lcd_set_cursor(0, DISPLAY_ROW_USER);
    lcd_print(user_row);
}
