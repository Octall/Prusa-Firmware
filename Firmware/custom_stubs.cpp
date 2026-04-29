/**
 * custom_stubs.cpp
 *
 * Provides stub implementations and global variable definitions for subsystems
 * that are removed in the ToolIndexer build but are still referenced by the
 * retained driver files (stepper.cpp, planner.cpp, tmc2130.cpp).
 *
 * Rules:
 *  - No actual thermal management, heater control, or SD card logic.
 *  - manage_heater() and manage_inactivity() are no-ops.
 *  - Temperature globals are zero-initialised so that FORCE_INLINE accessors
 *    in temperature.h compile and link without crashing at runtime.
 *  - M500_conf cs is initialised with the ToolIndexer default motion parameters.
 */

#include "Marlin.h"
#include "temperature.h"
#include "ConfigurationStore.h"
#include "fancheck.h"
#include <avr/interrupt.h>

// ---------------------------------------------------------------------------
// Timer0 OVF stub
//
// heatbed_pwm.cpp owns ISR(TIMER0_OVF_vect) in the full Marlin build.
// We removed that file, but the Arduino core's init() unconditionally enables
// the Timer0 OVF interrupt (TIMSK0 |= TOIE0) and calls sei() before setup()
// runs. Without an ISR, the first Timer0 overflow calls __bad_interrupt which
// soft-resets the MCU. Provide a no-op ISR so overflow is harmless until
// setup() can clear TIMSK0.
// ---------------------------------------------------------------------------
ISR(TIMER0_OVF_vect) {}

/* -----------------------------------------------------------------------
 * Temperature globals (required by FORCE_INLINE functions in temperature.h
 * and referenced symbolically from planner.cpp / stepper.cpp).
 * ----------------------------------------------------------------------- */
int   target_temperature[EXTRUDERS]   = {0};
float current_temperature[EXTRUDERS]  = {0.0f};
int   target_temperature_bed          = 0;
float current_temperature_bed         = 0.0f;

#ifdef PINDA_THERMISTOR
uint16_t current_temperature_raw_pinda = 0;
float    current_temperature_pinda     = 0.0f;
#endif

#ifdef AMBIENT_THERMISTOR
int   current_temperature_raw_ambient = 0;
float current_temperature_ambient     = 0.0f;
#endif

#ifdef VOLT_PWR_PIN
int current_voltage_raw_pwr = 0;
#endif

#ifdef VOLT_BED_PIN
int current_voltage_raw_bed = 0;
#endif

bool bedPWMDisabled = false;

/* Fan control stubs (FAN_SOFT_PWM is defined in Configuration.h) */
#ifdef FAN_SOFT_PWM
unsigned char fanSpeedSoftPwm = 0;
uint8_t       fanSpeedBckp    = 0;
#endif

/* fan_measuring declared by fancheck.h when EXTRUDER_0_AUTO_FAN_PIN > -1 && FAN_SOFT_PWM */
#if defined(EXTRUDER_0_AUTO_FAN_PIN) && (EXTRUDER_0_AUTO_FAN_PIN > -1)
#  ifdef FAN_SOFT_PWM
bool fan_measuring = false;
#  endif
volatile uint8_t fan_check_error = 0;
unsigned long extruder_autofan_last_check = 0;
void setExtruderAutoFanState(uint8_t) {}
void checkExtruderAutoFans()           {}
#endif

#if defined(FANCHECK) && defined(TACH_0) && (TACH_0 > -1)
void readFanTach() {}
#endif
#if defined(FANCHECK) && defined(TACH_1) && (TACH_1 > -1)
void setup_fan_interrupt() {}
#endif
void checkFans()    {}
void resetFanCheck() {}
void hotendFanSetFullSpeed()       {}
void hotendDefaultAutoFanState()   {}

#ifdef BABYSTEPPING
volatile int babystepsTodo[3] = {0, 0, 0};
#endif

/* -----------------------------------------------------------------------
 * Temperature subsystem stubs
 * ----------------------------------------------------------------------- */
void manage_heater() {}                    // no heaters
bool get_temp_error()       { return false; }
int  getHeaterPower(int)    { return 0; }
void disable_heater()       {}
void updatePID()            {}
void resetPID(uint8_t)      {}
void soft_pwm_init()        {}
void temp_mgr_init()        {}

#ifdef PIDTEMP
int pid_cycle = 0;
int pid_number_of_cycles = 0;
float _Kp = 0, _Ki = 0, _Kd = 0;
float scalePID_i(float i)   { return i; }
float scalePID_d(float d)   { return d; }
float unscalePID_i(float i) { return i; }
float unscalePID_d(float d) { return d; }
bool pidTuningRunning()     { return false; }
void preparePidTuning()     {}
#endif

#ifdef PINDA_THERMISTOR
bool has_temperature_compensation() { return false; }
#endif

/* -----------------------------------------------------------------------
 * Inactivity management stub
 * Called from planner and stepper wait loops; we do nothing here.
 * Motor disable-on-idle can be added here later if desired.
 * ----------------------------------------------------------------------- */
void manage_inactivity(bool /*ignore_stepper_queue*/) {}

/* -----------------------------------------------------------------------
 * M500_conf cs — motion configuration struct used by planner.cpp.
 * Initialised with ToolIndexer defaults matching the variant header.
 * ----------------------------------------------------------------------- */
M500_conf cs = {
    /* version                          */ "V3\0",
    /* axis_steps_per_mm[4]             */ DEFAULT_AXIS_STEPS_PER_UNIT,
    /* max_feedrate_normal[4]           */ DEFAULT_MAX_FEEDRATE,
    /* max_acceleration_mm_per_s2_normal[4] */ DEFAULT_MAX_ACCELERATION,
    /* acceleration                     */ DEFAULT_ACCELERATION,
    /* retract_acceleration             */ DEFAULT_RETRACT_ACCELERATION,
    /* minimumfeedrate                  */ DEFAULT_MINIMUMFEEDRATE,
    /* mintravelfeedrate                */ DEFAULT_MINTRAVELFEEDRATE,
    /* min_segment_time_us              */ DEFAULT_MINSEGMENTTIME,
    /* max_jerk[4]                      */ {DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK},
    /* add_homing[3]                    */ {0.0f, 0.0f, 0.0f},
    /* zprobe_zoffset                   */ 0.0f,
    /* Kp, Ki, Kd                       */ 0.0f, 0.0f, 0.0f,
    /* bedKp, bedKi, bedKd              */ 0.0f, 0.0f, 0.0f,
    /* lcd_contrast                     */ 0,
    /* autoretract_enabled              */ false,
    /* retract_length                   */ 0.0f,
    /* retract_feedrate                 */ 0.0f,
    /* retract_zlift                    */ 0.0f,
    /* retract_recover_length           */ 0.0f,
    /* retract_recover_feedrate         */ 0.0f,
    /* volumetric_enabled               */ false,
    /* filament_size[1]                 */ {1.75f},
    /* max_feedrate_silent[4]           */ DEFAULT_MAX_FEEDRATE_SILENT,
    /* max_acceleration_mm_per_s2_silent[4] */ DEFAULT_MAX_ACCELERATION_SILENT,
    /* axis_ustep_resolution[4]         */ {0, 0, 0, 0},
    /* travel_acceleration              */ DEFAULT_TRAVEL_ACCELERATION,
    /* mm_per_arc_segment               */ DEFAULT_MM_PER_ARC_SEGMENT,
    /* min_mm_per_arc_segment           */ DEFAULT_MIN_MM_PER_ARC_SEGMENT,
    /* n_arc_correction                 */ DEFAULT_N_ARC_CORRECTION,
    /* min_arc_segments                 */ DEFAULT_MIN_ARC_SEGMENTS,
    /* arc_segments_per_sec             */ DEFAULT_ARC_SEGMENTS_PER_SEC,
};

/* -----------------------------------------------------------------------
 * ConfigurationStore stubs — no EEPROM config persistence.
 * ----------------------------------------------------------------------- */
void Config_ResetDefault() {}
void Config_StoreSettings() {}
bool Config_RetrieveSettings() { return true; }

/* -----------------------------------------------------------------------
 * Printer state stubs — referenced by Marlin.h / printer_state.h
 * ----------------------------------------------------------------------- */
// printer_state.h defines PrinterState enum; no global needed unless referenced.

/* -----------------------------------------------------------------------
 * util.cpp symbols — FW version info referenced from various places.
 * We provide minimal stubs so the linker is satisfied.
 * ----------------------------------------------------------------------- */
#include "util.h"

const uint16_t FW_VERSION_NR[4] = {1, 0, 0, 0};

static const char fw_version_str[] PROGMEM = "1.0.0-ToolIndexer";
const char* FW_VERSION_STR_P() { return fw_version_str; }

static const char fw_version_hash[] PROGMEM = "custom";
const char FW_VERSION_HASH[] = "custom";
const char* FW_VERSION_HASH_P() { return fw_version_hash; }

bool show_upgrade_dialog_if_version_newer(const char*) { return false; }
bool eeprom_fw_version_older_than_p(const uint16_t (&)[4]) { return false; }
void update_current_firmware_version_to_eeprom() {}

/* -----------------------------------------------------------------------
 * Marlin_main.cpp globals — referenced by planner.cpp / stepper.cpp.
 * In the original firmware these live in Marlin_main.cpp; we provide
 * them here so the retained drivers link without Marlin_main.
 * ----------------------------------------------------------------------- */
#include "mesh_bed_calibration.h"  // world2machine externs
#include <avr/pgmspace.h>

// Current logical position (mm) — kept in sync by custom_motion.cpp
float current_position[NUM_AXIS] = {0.0f, 0.0f, 0.0f, 0.0f};

// Active feedrate (mm/min) — set before calling plan_buffer_line
float feedrate = 1500.0f;

// Print-fan speed (0–255). We have no fan; set to 0.
uint8_t fanSpeed = 0;

// Whether each axis has been homed
bool axis_known_position[3] = {false, false, false};

// world2machine — no bed skew correction in this application.
// WORLD2MACHINE_CORRECTION_NONE = 0
uint8_t world2machine_correction_mode = 0;
float   world2machine_rotation_and_skew[2][2]     = {{1.0f, 0.0f}, {0.0f, 1.0f}};
float   world2machine_rotation_and_skew_inv[2][2]  = {{1.0f, 0.0f}, {0.0f, 1.0f}};
float   world2machine_shift[2]                     = {0.0f, 0.0f};

// PROGMEM strings used by SERIAL_ECHO_START / SERIAL_ERROR_START macros
const char echomagic[]  PROGMEM = "echo:";
const char errormagic[] PROGMEM = "Error:";

// Serial helpers — forward to MYSERIAL which is UART0
void serialprintPGM(const char *str)
{
    while (pgm_read_byte(str)) MYSERIAL.write(pgm_read_byte(str++));
}

void serialprintlnPGM(const char *str)
{
    serialprintPGM(str);
    MYSERIAL.println();
}

/* -----------------------------------------------------------------------
 * ADC callback — invoked by the ADC ISR (adc.cpp) after each conversion.
 * We don't need to process any ADC results (no heaters, no thermistors),
 * so this is a no-op.
 * ----------------------------------------------------------------------- */
void adc_callback() {}

/* -----------------------------------------------------------------------
 * Crash detection stub — called by tmc2130_st_isr() on stall.
 * We just disable the stepper ISR to stop motion safely.
 * ----------------------------------------------------------------------- */
#include "stepper.h"
void crashdet_stop_and_save_print()
{
    DISABLE_STEPPER_DRIVER_INTERRUPT();
}

/* -----------------------------------------------------------------------
 * Farm/ECool mode stub — referenced by st_init().
 * Always returns false: no farm mode, normal ECool behaviour.
 * ----------------------------------------------------------------------- */
bool FarmOrUserECool() { return false; }

/* -----------------------------------------------------------------------
 * EEPROM stub — Sound_Init() calls eeprom_init_default_byte().
 * We return the default value without touching real EEPROM.
 * ----------------------------------------------------------------------- */
#include "eeprom.h"
uint8_t eeprom_init_default_byte(uint8_t * /*__p*/, uint8_t def) { return def; }
