// -------------------------------------------------------------------------------
// KilterTilter Main Sketch
//
// - Setup and Loop at top for readability
// - Grouped helper sections with header comments
// -------------------------------------------------------------------------------
// Various notes:
// At a medium range speed we get about 6ms between ticks.
// SerialUSB seems to sometimes cause problems: random hangs, display causing mischief...

#define BOARD_TYPE BOARD_TYPE_ARCTIC_CYTRON

#include "Adafruit_Keypad.h"
#include "DueFlashStorage.h"
#include "Wire.h"
#include "boards.h"
#include <CytronMotorDriver.h>

#include "display.hpp"
#include "flashconfig.h"
#include "pinlock.hpp"
#include "motordriver.hpp"
#include <Arduino.h>
// Enable most verbose logging for this build
//#define LOG_LEVEL LOG_VERBOSE
#include "logging.hpp"
#include "util.hpp"
#include "buttons.hpp"
#include <stdarg.h>

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

// -------------------------------------------------------------------------------
// GLOBALS
// - Centralized variable declarations with brief usage notes
// -------------------------------------------------------------------------------
// Acceleration timing accumulator used to step up speed in get_master_motor_speed()
uint32_t delta_acc = 0;
// Acceleration phase flag: true while ramping up speed
bool g_accel_phase = false;

// Button latches (set by ISRs, consumed in handle_buttons())
volatile bool g_extend_pressed  = false;
volatile bool g_retract_pressed = false;

// Primary display instance for UI views
Display display;

#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_WITH_KEYPAD)
DueFlashStorage flash;
FlashConfig flashconfig(flash);
PinLock pinlock(display, flashconfig);
#endif

// General print buffer.
// General print buffer for temporary formatting (self-test reports etc.)
char buf[512];

// Current target index in tick_targets[]
uint32_t g_target_idx = 0;
// Counters for HAL ticks during retract/extend windows (reset on movement)
uint32_t target_ticks_retracting = 0;
uint32_t target_ticks_extending  = 0;
// Baseline motor position for cumulative progress during a movement
int32_t g_progress_start_ticks = 0;

// Motor positions can actually be negative -- we have special
// casing for fully extended and fully retracted, however.
// motor positions are tracked inside MotorDriver

#define MAX_ANGLES (11)

// from to.
uint32_t tick_targets[MAX_ANGLES] = {
    000,    // 20
    3000,   // 25
    6000,   // 30
    9000,   // 35
    12500,  // 40
    16400,  // 45
    20000,  // 50
    24000,  // 55
    29000,  // 60
    33000,  // 65
    37000,  // 70
};

// PID controllers are embedded inside each MotorDriver; no global arrays needed.

// Globals.
// Deprecated: motor speeds tracked in each MotorDriver
// Tick counters now live inside each MotorDriver instance
// Last HAL clock tick per motor (reserved; not currently used)
volatile int32_t lastClk[NUM_MOTORS] = {0};

typedef enum { CMD_STOP, CMD_EXTEND, CMD_RETRACT, CMD_HOME, MAX_CMD } motor_cmd_t;

// We have a command -- which is what we want to happen.
// and a state -- which is what is actually happening, based on sensor feedback.
// Currently executing command (STOP/EXTEND/RETRACT/HOME)
volatile motor_cmd_t current_motor_command = CMD_STOP;

// Verbose state printing throttle helpers (print_state())
uint32_t stopping_delta = 0;  // Reserved for future stopping diagnostics
uint32_t last_print     = 0;  // Throttle marker for periodic logs

// Error-check throttle marker (used by error_check_motor_positions())
uint32_t g_last_error_check = 0;

float get_current_angle(uint32_t ticks);
// setDebounce lives in util.cpp
void enable_hal_irqs();
void disable_hal_irqs();
void reset_tick_counters(uint32_t m, bool all);
void reset_motor_positions();
void self_test();
void error_check_motor_positions();
int32_t get_master_motor_speed(void);
bool motors_idle_for(uint32_t ms);
void reset_idle_timer();
void isr_extend();
void isr_retract();
void panicf(const char *fmt, ...);
void panic(const char *msg);
void handle_buttons();


void set_current_command(motor_cmd_t cmd);
void print_state(uint32_t print_rate);



// -------------------------------------------------------------------------------
// SETUP & LOOP
// -------------------------------------------------------------------------------

void setup();
void loop();

// Move setup and loop to the top for readability

// -------------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    display.init();
    Wire.setClock(50000);

    //display.startup_animation();

#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_WITH_KEYPAD)
    LOGI("Init motors (ARCTIC_WITH_KEYPAD).");
#endif

    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(HAL_CNT[i], INPUT_PULLUP);
        pinMode(HAL_CLK[i], INPUT_PULLUP);
    }

#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_WITH_KEYPAD)
    flashconfig.init();
    pinlock.init();
#endif

    LOGI("LCD initialized.");

    delay(500);
    enable_hal_irqs();
    reset_tick_counters(0, true);
    reset_motor_positions();

    pinMode(BUTTON_EXTEND_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RETRACT_PIN, INPUT_PULLUP);
    if (digitalRead(BUTTON_EXTEND_PIN) == LOW) {
        g_extend_pressed = true;
    }
    if (digitalRead(BUTTON_RETRACT_PIN) == LOW) {
        g_retract_pressed = true;
    }

    bool e_pressed = false;
    bool r_pressed = false;
    int speed      = 0;

    e_pressed = g_extend_pressed;
    r_pressed = g_retract_pressed;

    if (e_pressed && !r_pressed) {
        display.set_refresh_rate(0.75f);
        display.update_manual_view((long)motors[MASTER_MOTOR].getPosition());
        display.set_view(LCD_MANUAL_VIEW);
        detachInterrupt(digitalPinToInterrupt(BUTTON_EXTEND_PIN));
        detachInterrupt(digitalPinToInterrupt(BUTTON_RETRACT_PIN));
        while (1) {
            e_pressed = (digitalRead(BUTTON_EXTEND_PIN) == LOW);
            r_pressed = (digitalRead(BUTTON_RETRACT_PIN) == LOW);
            speed     = 0;
            if (e_pressed) {
                speed = MINSPEED;
                LOGI("Extending.");
            }

            if (r_pressed) {
                speed = -1 * MINSPEED;
                LOGI("Retracting.");
            }

            for (int i = 0; i < NUM_MOTORS; ++i) {
                motors[i].setSpeed(speed);
                delay(100);
            }

            long pos = (long)motors[MASTER_MOTOR].getPosition();
            display.update_manual_view(pos);
            display.refresh();
            delay(100);
        }
    }

    if (!(e_pressed && r_pressed)) {
        self_test();
    } else {
        LOGI("Skipping self-test.");
    }

    //disable_hal_irqs();

    for (int p = 1; p < NUM_MOTORS; p++) {
        motors[p].pidBegin(8, 0.001, 0);
        motors[p].pidConfigureLimits(MINSPEED, MAXSPEED);
        motors[p].pidSetSampleTime(50);
        motors[p].pidStart();
    }

    set_current_command(CMD_HOME);
    g_target_idx = 0;

    attachInterrupt(digitalPinToInterrupt(BUTTON_EXTEND_PIN), isr_extend, FALLING);
    setDebounce(BUTTON_EXTEND_PIN, 1000);
    attachInterrupt(digitalPinToInterrupt(BUTTON_RETRACT_PIN), isr_retract, FALLING);
    setDebounce(BUTTON_RETRACT_PIN, 1000);

    uint32_t dummy = g_APinDescription[BUTTON_EXTEND_PIN].pPort->PIO_ISR;
    dummy          = g_APinDescription[BUTTON_RETRACT_PIN].pPort->PIO_ISR;

    display.update_target_view(LOWEST_ANGLE, 0);

    if (g_has_keypad) {
        display.set_view(LCD_LOCKSCREEN_VIEW);
    } else {
        display.update_homing_view();
        display.set_view(LCD_HOMING_VIEW);
    }
    dummy = g_APinDescription[BUTTON_RETRACT_PIN].pPort->PIO_ISR;
}

// -------------------------------------------------------------------------------
// MAIN LOOP
// -------------------------------------------------------------------------------
void loop() {
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_WITH_KEYPAD)
    if (g_has_keypad && pinlock.check_locked()) {
        display.update_lockscreen_view(pinlock.get_input_pin());
        display.refresh();
    } else {
        handle_buttons();
    }
#else
    handle_buttons();
#endif

    for (int m = 0; m < NUM_MOTORS; ++m) {
        if (m == MASTER_MOTOR) {
            int new_speed = get_master_motor_speed();
            motors[MASTER_MOTOR].setSpeed(new_speed);
            continue;
        }

        switch (current_motor_command) {
            case CMD_STOP:
                motors[m].setSpeed(0);
                motors[m].pidReset();
                LOGV("Loop:M%d CMD_STOP spd=0 masterV=%lu pos=[%ld %ld] stepsOut=[%lu %lu] stepsIn=[%lu %lu]",
                     m,
                     (unsigned long)abs(motors[MASTER_MOTOR].getSpeed()),
                     (long)motors[0].getPosition(),
                     (long)motors[1].getPosition(),
                     (unsigned long)motors[0].getStepsOut(),
                     (unsigned long)motors[1].getStepsOut(),
                     (unsigned long)motors[0].getStepsIn(),
                     (unsigned long)motors[1].getStepsIn());
                break;

            case CMD_EXTEND: {
                int32_t spd = motors[m].computeFollow(false,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                LOGV("Loop:M%d CMD_EXTEND spd=%ld masterV=%lu tgtIdx=%u tgtTicks=%u pos=[%ld %ld]",
                     m,
                     (long)spd,
                     (unsigned long)abs(motors[MASTER_MOTOR].getSpeed()),
                     (unsigned)g_target_idx,
                     (unsigned)tick_targets[g_target_idx],
                     (long)motors[0].getPosition(),
                     (long)motors[1].getPosition());
                break;
            }

            case CMD_RETRACT: {
                int32_t spd = motors[m].computeFollow(true,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                LOGV("Loop:M%d CMD_RETRACT spd=%ld masterV=%lu tgtIdx=%u tgtTicks=%u pos=[%ld %ld]",
                     m,
                     (long)spd,
                     (unsigned long)abs(motors[MASTER_MOTOR].getSpeed()),
                     (unsigned)g_target_idx,
                     (unsigned)tick_targets[g_target_idx],
                     (long)motors[0].getPosition(),
                     (long)motors[1].getPosition());
                break;
            }

            case CMD_HOME: {
                int32_t spd = motors[m].computeFollow(true,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                LOGV("Loop:M%d CMD_HOME spd=%ld masterV=%lu pos=[%ld %ld] idleMs=[%lu %lu]",
                     m,
                     (long)spd,
                     (unsigned long)abs(motors[MASTER_MOTOR].getSpeed()),
                     (long)motors[0].getPosition(),
                     (long)motors[1].getPosition(),
                     (unsigned long)motors[0].getIdleDurationMs(),
                     (unsigned long)motors[1].getIdleDurationMs());
                break;
            }

            default:
                LOGE("Invalid command.");
                panic("BAD CMD");
        }
    }

    print_state(500);
    //error_check_motor_positions();

    uint32_t target_ticks = tick_targets[g_target_idx];
    int32_t pos           = motors[MASTER_MOTOR].getPosition();
    uint8_t pct = compute_progress_percent(pos, g_progress_start_ticks, (int32_t)target_ticks);

    float tgt_angle =
        (g_target_idx == 0) ? LOWEST_ANGLE : (g_target_idx * ANGLE_STEP + LOWEST_ANGLE);
    display.update_target_view(tgt_angle, pct);
    display.refresh();

    delay(200);
}

// Progress logic moved to util.cpp as compute_progress_percent()

// Some housekeeping every time we initiate a new command.
void set_current_command(motor_cmd_t cmd) {
    if (current_motor_command == cmd) {
        // No change.
        return;
    }
    if ((current_motor_command == CMD_RETRACT) && (cmd == CMD_HOME)) {
        // already retracting, so let's just update the current executing command,
        current_motor_command = cmd;
        return;
    }
    current_motor_command = cmd;

    // Movement commands.
    if (cmd == CMD_EXTEND || cmd == CMD_RETRACT || cmd == CMD_HOME) {
        // Faster refresh while moving
        display.set_refresh_rate(1.0f);

        // Restart the error timer -- we want to wait a bit before we panic,
        // in csae we are just starting up.
        g_last_error_check = millis();

        // The idle timer is similar.
        reset_idle_timer();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
        // Restart tick counters (but not motor positions)
        reset_tick_counters(0, true);
        //enable_hal_irqs();
        // Set baseline only when transitioning from STOP to movement
        // to keep cumulative progress across target hops.
        if (current_motor_command == CMD_STOP) {
            g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
        }
    } else if (cmd == CMD_STOP) {
        // Slower refresh while stopped
        display.set_refresh_rate(30.0f);
        // When stopped, set baseline to current
        g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
    }
}

void reset_tick_counters(uint32_t m, bool all) {
    LOGI("Resetting tick counters.");

    target_ticks_extending  = 0;
    target_ticks_retracting = 0;

    if (all) {
        for (int j = 0; j < NUM_MOTORS; j++) {
            motors[j].resetSteps(true);
        }
    } else {
        motors[m].resetSteps(true);
    }
}

void reset_motor_positions() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPosition(0);
    }
}

void signal0_clk() {
    if (digitalRead(HAL_CNT[0]) == 0) {
        motors[0].incrementStepIn();
        motors[0].incrementPosition(-1);
    } else {
        motors[0].incrementStepOut();
        motors[0].incrementPosition(1);
    }
}

void signal1_clk() {
    if (digitalRead(HAL_CNT[1]) == 0) {
        motors[1].incrementStepIn();
        motors[1].incrementPosition(-1);
    } else {
        motors[1].incrementStepOut();
        motors[1].incrementPosition(1);
    }
}

void signal2_clk() {
    if (digitalRead(HAL_CNT[2]) == 0) {
        motors[2].incrementStepIn();
        motors[2].incrementPosition(-1);
    } else {
        motors[2].incrementStepOut();
        motors[2].incrementPosition(1);
    }
}

void signal3_clk() {
    if (digitalRead(HAL_CNT[3]) == 0) {
        motors[3].incrementStepIn();
        motors[3].incrementPosition(-1);
    } else {
        motors[3].incrementStepOut();
        motors[3].incrementPosition(1);
    }
}

void panic(const char *msg) {
    char shortmsg[64];
    strncpy(shortmsg, msg, sizeof(shortmsg) - 1);
    shortmsg[sizeof(shortmsg) - 1] = '\0';

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }

    // Show a clear panic banner using dedicated API and persist it
    display.show_panic(shortmsg);
    LOGE("PANIC: %s", shortmsg);

    // Stay here; keep the display responsive for visibility
    while (1) {
        display.refresh();
        delay(1000);
    }
}

void panicf(const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // Ensure display message fits 16 chars for the second line
    char line2[17];
    strncpy(line2, buf, 16);
    line2[16] = '\0';
    panic(line2);
}

void error_check_motor_positions() {
    // Stall detection via MotorDriver internal windowed checks

    // If we are stopped, we have no movement.
    if (current_motor_command == CMD_STOP) {
        return;
    }
    // If we are homing, we don't want to error check either.
    // This is a risk; but ... there we are.
    if (current_motor_command == CMD_HOME) {
        return;
    }

    if (g_accel_phase) {
        return;
    }

    // Perform self-check approximately every second using throttle helper.
    if (!throttle(g_last_error_check, 1000)) {
        return;
    }
    // we can be bottomed out; in which case we don't need to panic
    if (current_motor_command == CMD_HOME) {
        if (motors_idle_for(500)) {
            return;
        }
    }

    // Check HAL feedback via driver's windowed errorCheck; panic only on failure
    for (int i = 0; i < NUM_MOTORS; i++) {
        bool ok = motors[i].errorCheck(
            motors[i].getStepsIn(),
            motors[i].getStepsOut(),
            /*min_delta=*/3,
            /*window_ms=*/1000);
        if (!ok) {
            LOGE("Stall: motor %d pos=%ld si=%ld so=%ld",
                 i,
                 (long)motors[i].getPosition(),
                 (long)motors[i].getStepsIn(),
                 (long)motors[i].getStepsOut());
            panicf("M%d STALL p%ld", i, (long)motors[i].getPosition());
        }
    }
    // g_last_error_check updated by throttle()
}

void self_test() {
    LOGI("Self-test started.");
    display.print("Self test 1", "Extending.");

    const int test_speed = MINSPEED;
    const int test_time  = 1000;
    int test_start_time  = millis();

    // Enable the HAL measurements.
    enable_hal_irqs();

    // If the motors have managed to extend completely;
    // we can't extend more, and the test will fail.
    // So we start out by retracting; but this isn't strictly part of
    // the testing so we forcefully move them (slowly).

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(-test_speed);
    }

    delay(test_time + 500U);
    reset_tick_counters(0, true);

    // Now the test actually starts.
    // Extend all motors for a bit
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(test_speed);
    }
    delay(test_time);

    display.print("Phase 1 done", "Checking...");

    // Stop motors.
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }

    // We have extended for a bit, stop and report state.
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Check 1: have we gotten any sensor data at all?
        if (motors[i].getStepsOut() < 100U) {
            LOGE("Self-test failed: motor %d si:%d so:%d",
                 i,
                 motors[i].getStepsIn(),
                 motors[i].getStepsOut());

            sprintf(buf, "M:%d SO:%d", i, (int)motors[i].getStepsOut());
            display.print("SELF TEST FAILED", buf);
            // Keep on-screen message concise
            panicf("M%d NO OUT", i);
        }
        LOGI("Self-test extend: motor %d si:%d so:%d",
             i,
             motors[i].getStepsIn(),
             motors[i].getStepsOut());
    }

    // Second phase.
    display.print("Self test 2", "Retracting.");

    // We go back a bit.
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(-test_speed);
    }
    delay(test_time);

    display.print("Phase 2 done", "Checking...");

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }

    // And report.
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Check 2: check for steps in
        if (motors[i].getStepsIn() < 100U) {
            LOGE("Self-test failed: motor %d si:%d so:%d",
                 i,
                 (int)motors[i].getStepsIn(),
                 (int)motors[i].getStepsOut());

            sprintf(buf, "M:%d SI:%d", i, (int)motors[i].getStepsIn());
            display.print("SELF TEST FAILED:", buf);
            // Keep on-screen message concise
            panicf("M%d NO IN", i);
        }

        LOGI("Self-test retract: motor %d si:%d so:%d",
             i,
             (int)motors[i].getStepsIn(),
             (int)motors[i].getStepsOut());
    }

    // Compare the differenes

    bool final_success = true;
    int test_passed[4] = {1, 1, 1, 1};
    // Let's compare the numbers
    for (int i = 0; i < NUM_MOTORS; ++i) {
        // We accept some deviation, but not much; this should be fairly clean
        // signals with pullups on them. If they aren't we likely have bad ground,
        // bad connection.
        if (abs((int)motors[i].getStepsOut() - (int)motors[i].getStepsIn()) > 60) {
            LOGE("Self-test fail: motor %d diff %d",
                 i,
                 abs((int)motors[i].getStepsOut() - (int)motors[i].getStepsIn()));
            final_success = false;
        }
    }

    if (final_success == false) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            LOGI("Self-test report: motor %d si:%d so:%d",
                 i,
                 motors[i].getStepsIn(),
                 motors[i].getStepsOut());
        }
        display.print("FAIL: BAD SENSOR DATA", "Check connections.");
        // Transition to standardized PANIC screen with concise message
        panicf("CHECK CONNECTIONS");
    }

    // Success banner
    if (final_success) {
        display.show_short_message("Self test", "PASSED", 1500);
    }
}

/**
 * @brief Check for button presses and react.
 *
 */

void handle_buttons() {
    // Helpers are split into testable functions in buttons.cpp
    // Here we only orchestrate globals and display updates.
    extern int compute_next_target_index(
        bool extend, bool retract, int current_idx, int max_angles);
    extern int decide_command_for_target(int32_t current_pos,
                                         uint32_t new_target_pos,
                                         int cmd_stop,
                                         int cmd_extend,
                                         int cmd_retract,
                                         int cmd_home);
    extern bool is_target_change_permitted(int32_t current_pos,
                                           uint32_t new_target_pos,
                                           int current_cmd,
                                           int cmd_extend,
                                           int cmd_retract);
    bool extend  = g_extend_pressed;
    bool retract = g_retract_pressed;
    // Clear latched flags after reading
    g_extend_pressed  = false;
    g_retract_pressed = false;

    // Guard only by target index: disallow extending past max index
    // and retracting past min index; keep handling flow intact.

    // Trigger refresh if any button was pressed
    if (extend || retract) {
        display.trigger_refresh();
    }

    uint32_t new_target_position = 0;
    int32_t new_target_idx       = 0;

    // Exit early when homing. 
    if (current_motor_command == CMD_HOME) {
        // display.update_homing_view();
        goto exit_button_handling;
    }

    // Act on the buttons.
    // We pressed the extend button. We can add to the extending index in 2
    // conditions:
    // 1. we are stopped or retracting
    // 2. we are still extending
    // In both these cases, the indexed target is where we are heading.

    new_target_idx = compute_next_target_index(extend, retract, g_target_idx, MAX_ANGLES);
    if (new_target_idx == g_target_idx) {
        // No change; bounds already logged by compute_next_target_index
        goto exit_button_handling;
    }

    // Ok, so we got a new target index.
    if ((new_target_idx >= MAX_ANGLES) || (new_target_idx < 0)) {
        // reject -- use old target index.
        LOGW("Out of range; ignoring.");
        goto exit_button_handling;
    }

    // Ok, not total rejection -- we are heading towards a new target.
    new_target_position = tick_targets[new_target_idx];
    // Target position will be acted upon; logging occurs in decide_command_for_target

    // Easy case -- we are standing still.
    if (current_motor_command == CMD_STOP) {
        int decided = decide_command_for_target(motors[MASTER_MOTOR].getPosition(),
                                                new_target_position,
                                                CMD_STOP,
                                                CMD_EXTEND,
                                                CMD_RETRACT,
                                                CMD_HOME);
        set_current_command((motor_cmd_t)decided);
        // Idle decision logging handled in decide_command_for_target
        // We accept this change.
        g_target_idx = new_target_idx;
        // Keep cumulative baseline; do not reset on target change
        goto exit_button_handling;
    } else {
        // We are executing a command and only
        // accept changes in the same direction.
        if (!is_target_change_permitted(motors[MASTER_MOTOR].getPosition(),
                                        new_target_position,
                                        current_motor_command,
                                        CMD_EXTEND,
                                        CMD_RETRACT)) {
            // Direction mismatch logged in is_target_change_permitted
            display.show_short_message("ERROR:", "LOWERING", 2000);
            new_target_idx = g_target_idx;
        } else {
            // We accept this change.
            LOGI("Change accepted.");
            g_target_idx = new_target_idx;
            if (new_target_idx == 0) {
                // Special case -- we are homing.
                set_current_command(CMD_HOME);
                // Logging for homing occurs in decide_command_for_target
            }
            // Keep cumulative baseline; do not reset on target change
        }
    }
exit_button_handling:
    display.update_target_view((g_target_idx * ANGLE_STEP) + LOWEST_ANGLE,
                               compute_progress_percent(motors[MASTER_MOTOR].getPosition(),
                                                        g_progress_start_ticks,
                                                        (int32_t)tick_targets[g_target_idx]));
    display.refresh();
    return;
}

void print_state(uint32_t print_rate) {
    if (!throttle(last_print, print_rate)) {
        return;
    }

    // Assume exactly two motors: 0 = master, 1 = slave
    LOGV("CMD:%d TTR:%u TTE:%u T:%u",
         (int)current_motor_command,
         (unsigned)target_ticks_retracting,
         (unsigned)target_ticks_extending,
         (unsigned)tick_targets[g_target_idx]);

    LOGV("SO:[%lu %lu] SI:[%lu %lu]",
         (unsigned long)motors[0].getStepsOut(),
         (unsigned long)motors[1].getStepsOut(),
         (unsigned long)motors[0].getStepsIn(),
         (unsigned long)motors[1].getStepsIn());

    LOGV("P:[%ld %ld] V:[%lu %lu]",
         (long)motors[0].getPosition(),
         (long)motors[1].getPosition(),
         (unsigned long)abs(motors[0].getSpeed()),
         (unsigned long)abs(motors[1].getSpeed()));

    // last_print updated by throttle()
}

// -------------------------------------------------------------------------------
// HAL IRQ MANAGEMENT
// -------------------------------------------------------------------------------
void enable_hal_irqs() {
    attachInterrupt(digitalPinToInterrupt(HAL_CLK[0]), signal0_clk, RISING);
    setDebounce(HAL_CLK[0], 100);
    attachInterrupt(digitalPinToInterrupt(HAL_CLK[1]), signal1_clk, RISING);
    setDebounce(HAL_CLK[1], 100);
#if NUM_MOTORS > 2
    attachInterrupt(digitalPinToInterrupt(HAL_CLK[2]), signal2_clk, RISING);
    setDebounce(HAL_CLK[2], 100);
#endif
#if NUM_MOTORS > 3
    attachInterrupt(digitalPinToInterrupt(HAL_CLK[3]), signal3_clk, RISING);
    setDebounce(HAL_CLK[3], 100);
#endif
}

void disable_hal_irqs() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        detachInterrupt(digitalPinToInterrupt(HAL_CLK[i]));
    }
}

// Interrupt-based button handling with simple debounce using hardware filter.
void isr_extend() {
    // Active low buttons; confirm state to avoid spurious triggers.
    if (digitalRead(BUTTON_EXTEND_PIN) == LOW) {
        g_extend_pressed = true;
    }
}

void isr_retract() {
    if (digitalRead(BUTTON_RETRACT_PIN) == LOW) {
        g_retract_pressed = true;
    }
}

// -------------------------------------------------------------------------------
// MASTER SPEED COMPUTATION
// The master motor controls everything. The other motors try to follow via PID.
// -------------------------------------------------------------------------------
int32_t get_master_motor_speed(void) {
    // Figure out our targets.
    int32_t target = tick_targets[g_target_idx];
    if (current_motor_command == CMD_HOME) {
        // Going home.
        target = -1000000;
    }

    // int32_t current_step = motor_position[MASTER_MOTOR];
    // int distance_to_target = (int)abs((int)target - (int)current_step);

    // Return the same speed as we had if we don't change it in here.
    uint32_t return_speed = abs(motors[MASTER_MOTOR].getSpeed());

    // Bring us up to minspeed slowly
    if ((abs(motors[MASTER_MOTOR].getSpeed()) < MINSPEED) && (current_motor_command != CMD_STOP)) {
        g_accel_phase = true;
    }

    // accelerating -- we'll start slow, then increase slowly.
    // To avoid surge.
    if (g_accel_phase) {
        // Increase speed every 50ms
        if ((millis() - delta_acc) > 50) {
            // If we have just started, the motor speed is 0.
            // In reverse, we won't really move until we hit MINSPEED.
            // With certain motors.
            if (abs(motors[MASTER_MOTOR].getSpeed()) == 0) {
                return_speed = MINSPEED;
            }

            return_speed = return_speed + (ACCEL_STEP);

            LOGI("Accelerating to: %d", return_speed);

            // Clamp the speed in case we went over.
            if (return_speed >= MASTER_MAX) {
                return_speed = MASTER_MAX;
                LOGI("Acceleration done. Final speed: %d", return_speed);
                g_accel_phase = false;
                delta_acc     = 0;
            }
            delta_acc = millis();
        }
    }

    // Figure out if we are at target, or home.
    // ----------------------------------------
    bool target_reached = false;

    // If all motors have been idle for 2000ms, we are likely home.
    if (current_motor_command == CMD_HOME) {
        if (motors_idle_for(2000)) {
            LOGI("Homing done.");
            disable_hal_irqs();
            reset_tick_counters(0, true);
            reset_motor_positions();
            if (g_has_keypad) {
                display.set_view(LCD_LOCKSCREEN_VIEW);
            } else {
                display.set_view(LCD_TARGET_VIEW);
            }

            target_reached = true;
        }
    }

    if (current_motor_command == CMD_RETRACT) {
        // fully retract -- we had target_idx 0 and we were idle.
        if ((motors[MASTER_MOTOR].getPosition() <= target)) {
            // We check for less than, because we are going back up and can
            // have managed to sneak slightly passed our target.
            target_reached = true;
        }
    }

    if ((current_motor_command == CMD_EXTEND)) {
        if ((motors[MASTER_MOTOR].getPosition() >= target)) {
            target_reached = true;
        }
    }

    if (target_reached) {
        LOGI("Target reached. Stopping main motor.");
        return_speed = 0;
        set_current_command(CMD_STOP);
        g_accel_phase = false;
    }

    // If we are retracting, return a negative speed.
    if ((current_motor_command == CMD_HOME) || (current_motor_command == CMD_RETRACT)) {
        return return_speed * -1;
    }

    return return_speed;
}

// setDebounce moved to util.cpp

// -------------------------------------------------------------------------------
// IDLE MEASUREMENT
// - Reset when a new command is given
// - Start measuring after 200ms to allow motors to begin moving
// - Per-motor idle tracked inside MotorDriver
// -------------------------------------------------------------------------------
bool motors_idle_for(uint32_t ms) {
    for (int m = 0; m < NUM_MOTORS; m++) {
        // Idle duration is measured as time since last movement
        if (motors[m].getIdleDurationMs() < ms) {
            return false;
        }
    }
    return true;
}

void reset_idle_timer() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motors[i].resetIdle();
    }
}