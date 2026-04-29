/**
 * custom_command.cpp
 *
 * Line-buffered ASCII command parser for the ToolIndexer firmware.
 * Reads from MYSERIAL (UART0, 115200 baud), executes each command,
 * and replies with "OK\n" or "ERROR <reason>\n".
 */

#include "custom_command.h"
#include "custom_motion.h"
#include "custom_sensor.h"
#include "custom_display.h"
#include "Marlin.h"        // MYSERIAL, NAN, etc.
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

// Maximum line length (bytes, including '\0')
#define CMD_BUF_LEN 80

static char  cmd_buf[CMD_BUF_LEN];
static uint8_t cmd_len = 0;

// ---------------------------------------------------------------------------
// Helper: send OK or ERROR over serial
// ---------------------------------------------------------------------------
static void reply_ok()    { MYSERIAL.print("OK\r\n"); }
static void reply_error(const char *reason)
{
    MYSERIAL.print("ERROR ");
    MYSERIAL.print(reason);
    MYSERIAL.print("\r\n");
}

// ---------------------------------------------------------------------------
// Helper: case-insensitive comparison of first `n` bytes
// ---------------------------------------------------------------------------
static bool starts_with_ci(const char *s, const char *prefix)
{
    while (*prefix)
    {
        if (toupper((uint8_t)*s) != toupper((uint8_t)*prefix)) return false;
        s++; prefix++;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Parse a named float argument from the command line.
// e.g. parse_arg("X", "MOVE X10.5 F3000") returns 10.5
// Returns NAN if the argument is not present.
// ---------------------------------------------------------------------------
static float parse_arg(char letter, const char *line)
{
    letter = toupper((uint8_t)letter);
    const char *p = line;
    while (*p)
    {
        if (toupper((uint8_t)*p) == letter && (p == line || p[-1] == ' '))
        {
            char *end;
            float val = (float)strtod(p + 1, &end);
            if (end != p + 1) return val;
        }
        p++;
    }
    return NAN;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static void cmd_home(const char *args)
{
    // If no axis specified, default to ALL
    bool do_x = strstr(args, "X") || strstr(args, "ALL") || (strlen(args) == 0);
    bool do_y = strstr(args, "Y") || strstr(args, "ALL") || (strlen(args) == 0);
    bool do_z = strstr(args, "Z") || strstr(args, "ALL") || (strlen(args) == 0);

    // Case-insensitive search
    for (const char *p = args; *p; p++) {
        char c = toupper((uint8_t)*p);
        if (c == 'X') do_x = true;
        if (c == 'Y') do_y = true;
        if (c == 'Z') do_z = true;
        if (c == 'A') { do_x = true; do_y = true; do_z = true; } // ALL
    }

    uint8_t mask = 0;
    if (do_x) mask |= AXIS_X;
    if (do_y) mask |= AXIS_Y;
    if (do_z) mask |= AXIS_Z;

    display_set_status("Homing...");
    motion_home(mask);
    display_set_status("Ready");
    reply_ok();
}

static void cmd_move(const char *args)
{
    float x = parse_arg('X', args);
    float y = parse_arg('Y', args);
    float z = parse_arg('Z', args);
    float e = parse_arg('E', args);
    float f = parse_arg('F', args);

    // At least one axis or feedrate must be specified
    if (isnan(x) && isnan(y) && isnan(z) && isnan(e))
    {
        reply_error("no axis specified");
        return;
    }

    motion_move(x, y, z, e, isnan(f) ? 0.0f : f);
    reply_ok();
}

static void cmd_wait(const char *)
{
    motion_wait();
    reply_ok();
}

static void cmd_estop(const char *)
{
    motion_estop();
    display_set_status("ESTOP");
    reply_ok();
}

static void cmd_enable(const char *)
{
    motion_enable();
    display_set_status("Ready");
    reply_ok();
}

static void cmd_read(const char *args)
{
    // Skip leading spaces
    while (*args == ' ') args++;

    if (starts_with_ci(args, "IR"))
    {
        MYSERIAL.print("VALUE ");
        MYSERIAL.print(sensor_read_ir() ? 1 : 0);
        MYSERIAL.print("\r\n");
    }
    else if (starts_with_ci(args, "PINDA"))
    {
        MYSERIAL.print("VALUE ");
        MYSERIAL.print(sensor_read_pinda() ? 1 : 0);
        MYSERIAL.print("\r\n");
    }
    else if (starts_with_ci(args, "ENDSTOP"))
    {
        const char *axis = args + 7;
        while (*axis == ' ') axis++;
        if (toupper((uint8_t)*axis) == 'X')
        {
            MYSERIAL.print("VALUE ");
            MYSERIAL.print(sensor_read_endstop_x() ? 1 : 0);
            MYSERIAL.print("\r\n");
        }
        else if (toupper((uint8_t)*axis) == 'Y')
        {
            MYSERIAL.print("VALUE ");
            MYSERIAL.print(sensor_read_endstop_y() ? 1 : 0);
            MYSERIAL.print("\r\n");
        }
        else if (toupper((uint8_t)*axis) == 'Z')
        {
            MYSERIAL.print("VALUE ");
            MYSERIAL.print(sensor_read_endstop_z() ? 1 : 0);
            MYSERIAL.print("\r\n");
        }
        else // ALL or unspecified
        {
            MYSERIAL.print("VALUE ");
            MYSERIAL.print(sensor_read_endstops_all());
            MYSERIAL.print("\r\n");
        }
    }
    else
    {
        reply_error("unknown sensor");
        return;
    }
    // Sensors don't print OK — keep the protocol consistent
    reply_ok();
}

static void cmd_status(const char *)
{
    MYSERIAL.print("POS");
    MYSERIAL.print(" X"); MYSERIAL.print(motion_get_position_mm(0), 2);
    MYSERIAL.print(" Y"); MYSERIAL.print(motion_get_position_mm(1), 2);
    MYSERIAL.print(" Z"); MYSERIAL.print(motion_get_position_mm(2), 2);
    MYSERIAL.print(" E"); MYSERIAL.print(motion_get_position_mm(3), 2); MYSERIAL.print("\r\n");

    MYSERIAL.print("SENSOR IR=");
    MYSERIAL.print(sensor_read_ir() ? 1 : 0);
    MYSERIAL.print(" PINDA=");
    MYSERIAL.print(sensor_read_pinda() ? 1 : 0);
    MYSERIAL.print(" ENDSTOP_X=");
    MYSERIAL.print(sensor_read_endstop_x() ? 1 : 0);
    MYSERIAL.print(" ENDSTOP_Y=");
    MYSERIAL.print(sensor_read_endstop_y() ? 1 : 0);
    MYSERIAL.print(" ENDSTOP_Z=");
    MYSERIAL.print(sensor_read_endstop_z() ? 1 : 0);
    MYSERIAL.print("\r\n");

    reply_ok();
}

static void cmd_rotate(const char *args)
{
    float deg = parse_arg('D', args);
    float f   = parse_arg('F', args);

    if (isnan(deg))
    {
        reply_error("missing D<degrees>");
        return;
    }

    motion_rotate_x_deg(deg, isnan(f) ? 0.0f : f);
    reply_ok();
}

static void cmd_lcd(const char *args)
{
    // Skip leading space
    if (*args == ' ') args++;
    display_set_user_row(args);
    reply_ok();
}

// ---------------------------------------------------------------------------
// Dispatch a complete line
// ---------------------------------------------------------------------------
static void dispatch(char *line)
{
    // Trim trailing whitespace
    int len = strlen(line);
    while (len > 0 && (line[len-1] == '\r' || line[len-1] == '\n' || line[len-1] == ' '))
        line[--len] = '\0';

    if (len == 0) return; // blank line — ignore silently

    // Command word is everything up to the first space
    char *args = strchr(line, ' ');
    if (args) {
        *args = '\0';
        args++;
    } else {
        args = line + len; // empty string
    }

    // Convert command word to upper case in-place
    for (char *p = line; *p; p++) *p = toupper((uint8_t)*p);

    if      (strcmp(line, "HOME")   == 0) cmd_home(args);
    else if (strcmp(line, "MOVE")   == 0) cmd_move(args);
    else if (strcmp(line, "ROTATE") == 0) cmd_rotate(args);
    else if (strcmp(line, "WAIT")   == 0) cmd_wait(args);
    else if (strcmp(line, "ESTOP")  == 0) cmd_estop(args);
    else if (strcmp(line, "ENABLE") == 0) cmd_enable(args);
    else if (strcmp(line, "READ")   == 0) cmd_read(args);
    else if (strcmp(line, "STATUS") == 0) cmd_status(args);
    else if (strcmp(line, "LCD")    == 0) cmd_lcd(args);
    else                                   reply_error("unknown command");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void command_init()
{
    cmd_len = 0;
    cmd_buf[0] = '\0';
}

void command_process()
{
    while (MYSERIAL.available())
    {
        char c = (char)MYSERIAL.read();

        if (c == '\n' || c == '\r')
        {
            if (cmd_len > 0)
            {
                MYSERIAL.print("\r\n"); // echo newline so cursor moves to next line
                cmd_buf[cmd_len] = '\0';
                dispatch(cmd_buf);
                cmd_len = 0;
            }
        }
        else if (c == '\b' || c == 0x7F) // backspace or DEL
        {
            if (cmd_len > 0)
            {
                cmd_len--;
                MYSERIAL.print("\b \b"); // move back, erase, move back again
            }
        }
        else if (cmd_len < CMD_BUF_LEN - 1)
        {
            cmd_buf[cmd_len++] = c;
            MYSERIAL.write(c); // echo character so it appears in the terminal
        }
        else
        {
            // Line too long — flush and report error
            cmd_len = 0;
            reply_error("line too long");
        }
    }
}
