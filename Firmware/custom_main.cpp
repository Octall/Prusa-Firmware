/**
 * custom_main.cpp
 *
 * ToolIndexer firmware entry point.
 * Replaces Marlin_main.cpp — a lean setup()/loop() that initialises only
 * the subsystems needed for stepper motor control, LCD display, and sensor
 * reading via a serial ASCII command interface.
 *
 * Initialization order (mirrors the critical dependencies from Marlin):
 *   1. Watchdog disable       — must be first (bootloader may leave WDT running)
 *   2. System timer (Timer2)  — enables millis(); required by everything below
 *   3. SPI                    — required by TMC2130 stepper drivers
 *   4. LCD hardware           — show startup message early
 *   5. Serial                 — host command interface
 *   6. ADC                    — for sensor reads (even if only digital,
 *                                adc_init sets up the ADC peripheral)
 *   7. Motion planner         — clear ring buffer
 *   8. Stepper + TMC2130      — enable step ISR; this enables interrupts
 *   9. Sensor pins            — configure GPIO inputs
 *  10. Display + Command      — register LCD callback, ready for commands
 */

#include "Marlin.h"          // MYSERIAL, fastio, pins, etc.
#include "lcd.h"
#include "adc.h"
#include "stepper.h"
#include "planner.h"
#include "tmc2130.h"
#include "sound.h"
#include "spi.h"
#include "system_timer.h"
#include "custom_motion.h"
#include "custom_sensor.h"
#include "custom_display.h"
#include "custom_command.h"

#include <avr/wdt.h>
#include <util/atomic.h>

// ---------------------------------------------------------------------------
// Watchdog — disable early to prevent spurious resets from old bootloaders
// ---------------------------------------------------------------------------
static void watchdog_early_disable()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        wdt_reset();
        MCUSR &= ~_BV(WDRF);
        wdt_disable();
    }
}

// ---------------------------------------------------------------------------
// setup() — called once by the AVR core's main()
// ---------------------------------------------------------------------------
void setup()
{
    // Disable Timer0 and Timer1 interrupts immediately.
    //
    // Timer0: The Arduino core's init() enables TIMER0_OVF interrupt and calls
    // sei() BEFORE setup() runs. heatbed_pwm.cpp (removed from this build) owns
    // the TIMER0_OVF ISR; without it, overflow fires __bad_interrupt → soft
    // reset. A no-op ISR stub in custom_stubs.cpp covers the race window
    // between init() and here; we then kill Timer0 interrupts entirely since
    // we don't need them.
    //
    // Timer1: used for the stepper ISR; may still be armed from the bootloader.
    // st_init() re-enables it after proper initialisation.
    TIMSK0 = 0;
    TIMSK1 = 0;

    // Serial first — maximises chance of output even if a reset follows immediately
    MYSERIAL.begin(BAUDRATE);
    // MYSERIAL.println("DBG:0");      // confirm serial works at all

    // 1. Disable watchdog (must be very early)
    watchdog_early_disable();
    // MYSERIAL.println("DBG:1");      // WDT disabled

    // 2. System timer — enables millis() / _millis()
    timer2_init();
    // MYSERIAL.println("DBG:2");      // Timer2 OK

    // 3. SPI — needed by TMC2130
    spi_init();
    // MYSERIAL.println("DBG:3");      // SPI OK

    // 4. LCD hardware — display a boot message immediately
    lcd_init();
    // MYSERIAL.println("DBG:4");      // LCD hw OK

    lcd_refresh();
    lcd_set_cursor(0, 0);
    lcd_puts_P(PSTR("ToolIndexer v1.0    "));
    lcd_set_cursor(0, 1);
    lcd_puts_P(PSTR("Booting...          "));
    // MYSERIAL.println("DBG:5");      // LCD text written

    // 5. ADC — sets up multiplexer; raw values used by adc.h consumers
    adc_init();
    adc_start_cycle();
    // MYSERIAL.println("DBG:6");      // ADC OK

    // 6. Sound
    Sound_Init();
    // MYSERIAL.println("DBG:7");      // Sound OK

    // 7. Motion planner — clear ring buffer and reset position state
    motion_init();   // calls plan_init() internally + sets position to 0
    // MYSERIAL.println("DBG:8");      // Planner OK

    // 8. TMC2130 stepper drivers
    //    Mode: NORMAL (spread cycle), no farm mode, no E-cool
    tmc2130_mode = TMC2130_MODE_NORMAL;
    tmc2130_init(TMCInitParams(false /*bSuppressFlag*/, false /*enableECool*/));
    // MYSERIAL.println("DBG:9");      // TMC2130 OK

    // 9. Stepper ISR — enables Timer1 COMPA interrupt
    //    Must come after TMC2130 so drivers are configured before first step
    st_init();
    // MYSERIAL.println("DBG:10");     // Stepper ISR OK

    // 10. Sensors — configure GPIO inputs
    sensor_init();
    // MYSERIAL.println("DBG:11");     // Sensors OK

    // 11. Display — register update callback with lcd.cpp
    display_init();
    // MYSERIAL.println("DBG:12");     // Display callback OK

    // 12. Command parser
    command_init();
    // MYSERIAL.println("DBG:13");     // Command parser OK

    // Ready
    display_set_status("ToolIndexer Ready");
    MYSERIAL.println("READY");
}

// ---------------------------------------------------------------------------
// loop() — called repeatedly by the AVR core's main()
// ---------------------------------------------------------------------------
void loop()
{
    // Process incoming serial commands (non-blocking)
    command_process();

    // Update LCD status screen (throttled internally by lcd.cpp)
    display_update();
}
