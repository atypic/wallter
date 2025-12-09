// ESP32-S3 entry point for KilterTilter
// Ported from kiltertilter.ino with SAM/Due specifics removed

#define BOARD_TYPE BOARD_TYPE_ESP32_S3_DEVKITC1

#include <Arduino.h>
#include <Wire.h>
#include "boards.h"
#include "display.hpp"
#include "flashconfig.h"
#include "pinlock.hpp"
#include "motordriver.hpp"
#include "logging.hpp"
#include "util.hpp"
#include "buttons.hpp"

// Globals and helpers replicated from the sketch
uint32_t delta_acc = 0;
bool g_accel_phase = false;
volatile bool g_extend_pressed  = false;
volatile bool g_retract_pressed = false;

Display display;

char buf[512];

uint32_t g_target_idx             = 0;
uint32_t target_ticks_retracting  = 0;
uint32_t target_ticks_extending   = 0;
int32_t g_progress_start_ticks    = 0;

#define MAX_ANGLES (11)

uint32_t tick_targets[MAX_ANGLES] = {
    000, 3000, 6000, 9000, 12500, 16400, 20000, 24000, 29000, 33000, 37000,
};

volatile int32_t lastClk[NUM_MOTORS] = {0};

typedef enum { CMD_STOP, CMD_EXTEND, CMD_RETRACT, CMD_HOME, MAX_CMD } motor_cmd_t;
volatile motor_cmd_t current_motor_command = CMD_STOP;

uint32_t stopping_delta = 0;
uint32_t last_print     = 0;
uint32_t g_last_error_check = 0;

// fwd decls
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

// ISR helpers
void signal0_clk();
void signal1_clk();
#if NUM_MOTORS > 2
void signal2_clk();
#endif
#if NUM_MOTORS > 3
void signal3_clk();
#endif

void setup() {
    Serial.begin(115200);
    display.init();
    Wire.setClock(50000);

    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(HAL_CNT[i], INPUT_PULLUP);
        pinMode(HAL_CLK[i], INPUT_PULLUP);
    }

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

    bool e_pressed = g_extend_pressed;
    bool r_pressed = g_retract_pressed;

    if (e_pressed && !r_pressed) {
        display.set_refresh_rate(0.75f);
        display.update_manual_view((long)motors[MASTER_MOTOR].getPosition());
        display.set_view(LCD_MANUAL_VIEW);
        detachInterrupt(digitalPinToInterrupt(BUTTON_EXTEND_PIN));
        detachInterrupt(digitalPinToInterrupt(BUTTON_RETRACT_PIN));
        while (1) {
            e_pressed = (digitalRead(BUTTON_EXTEND_PIN) == LOW);
            r_pressed = (digitalRead(BUTTON_RETRACT_PIN) == LOW);
            int speed = 0;
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

    display.update_target_view(LOWEST_ANGLE, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);
}

void loop() {
    handle_buttons();

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
                break;
            case CMD_EXTEND: {
                int32_t spd = motors[m].computeFollow(false,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            case CMD_RETRACT: {
                int32_t spd = motors[m].computeFollow(true,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            case CMD_HOME: {
                int32_t spd = motors[m].computeFollow(true,
                                                      g_accel_phase,
                                                      abs(motors[MASTER_MOTOR].getSpeed()),
                                                      motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            default:
                LOGE("Invalid command.");
                panic("BAD CMD");
        }
    }

    print_state(500);

    uint32_t target_ticks = tick_targets[g_target_idx];
    int32_t pos           = motors[MASTER_MOTOR].getPosition();
    uint8_t pct = compute_progress_percent(pos, g_progress_start_ticks, (int32_t)target_ticks);

    float tgt_angle = (g_target_idx == 0) ? LOWEST_ANGLE : (g_target_idx * ANGLE_STEP + LOWEST_ANGLE);
    display.update_target_view(tgt_angle, pct);
    display.refresh();

    delay(200);
}

void set_current_command(motor_cmd_t cmd) {
    if (current_motor_command == cmd) {
        return;
    }
    if ((current_motor_command == CMD_RETRACT) && (cmd == CMD_HOME)) {
        current_motor_command = cmd;
        return;
    }
    current_motor_command = cmd;

    if (cmd == CMD_EXTEND || cmd == CMD_RETRACT || cmd == CMD_HOME) {
        display.set_refresh_rate(1.0f);
        g_last_error_check = millis();
        reset_idle_timer();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
        reset_tick_counters(0, true);
        if (current_motor_command == CMD_STOP) {
            g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
        }
    } else if (cmd == CMD_STOP) {
        display.set_refresh_rate(30.0f);
        g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
    }
}

void reset_tick_counters(uint32_t m, bool all) {
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

#if NUM_MOTORS > 2
void signal2_clk() {
    if (digitalRead(HAL_CNT[2]) == 0) {
        motors[2].incrementStepIn();
        motors[2].incrementPosition(-1);
    } else {
        motors[2].incrementStepOut();
        motors[2].incrementPosition(1);
    }
}
#endif

#if NUM_MOTORS > 3
void signal3_clk() {
    if (digitalRead(HAL_CNT[3]) == 0) {
        motors[3].incrementStepIn();
        motors[3].incrementPosition(-1);
    } else {
        motors[3].incrementStepOut();
        motors[3].incrementPosition(1);
    }
}
#endif

void panic(const char *msg) {
    char shortmsg[64];
    strncpy(shortmsg, msg, sizeof(shortmsg) - 1);
    shortmsg[sizeof(shortmsg) - 1] = '\0';
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }
    display.show_panic(shortmsg);
    LOGE("PANIC: %s", shortmsg);
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
    char line2[17];
    strncpy(line2, buf, 16);
    line2[16] = '\0';
    panic(line2);
}

void error_check_motor_positions() {
    if (current_motor_command == CMD_STOP || current_motor_command == CMD_HOME) {
        return;
    }
    if (g_accel_phase) {
        return;
    }
    if (!throttle(g_last_error_check, 1000)) {
        return;
    }
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
}

void self_test() {
    LOGI("Self-test started.");
    display.print("Self test 1", "Extending.");
    const int test_speed = MINSPEED;
    const int test_time  = 1000;

    enable_hal_irqs();

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(-test_speed);
    }
    delay(test_time + 500U);
    reset_tick_counters(0, true);

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(test_speed);
    }
    delay(test_time);

    display.print("Phase 1 done", "Checking...");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].getStepsOut() < 100U) {
            LOGE("Self-test failed: motor %d si:%d so:%d",
                 i,
                 motors[i].getStepsIn(),
                 motors[i].getStepsOut());
            sprintf(buf, "M:%d SO:%d", i, (int)motors[i].getStepsOut());
            display.print("SELF TEST FAILED", buf);
            panicf("M%d NO OUT", i);
        }
    }

    display.print("Self test 2", "Retracting.");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(-test_speed);
    }
    delay(test_time);

    display.print("Phase 2 done", "Checking...");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].getStepsIn() < 100U) {
            LOGE("Self-test failed: motor %d si:%d so:%d",
                 i,
                 (int)motors[i].getStepsIn(),
                 (int)motors[i].getStepsOut());
            sprintf(buf, "M:%d SI:%d", i, (int)motors[i].getStepsIn());
            display.print("SELF TEST FAILED:", buf);
            panicf("M%d NO IN", i);
        }
    }

    bool final_success = true;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (abs((int)motors[i].getStepsOut() - (int)motors[i].getStepsIn()) > 60) {
            final_success = false;
        }
    }
    if (!final_success) {
        display.print("FAIL: BAD SENSOR DATA", "Check connections.");
        panicf("CHECK CONNECTIONS");
    }
    display.show_short_message("Self test", "PASSED", 1500);
}

void handle_buttons() {
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
    g_extend_pressed  = false;
    g_retract_pressed = false;

    if (extend || retract) {
        display.trigger_refresh();
    }

    uint32_t new_target_position = 0;
    int32_t new_target_idx       = 0;

    if (current_motor_command == CMD_HOME) {
        goto exit_button_handling;
    }

    new_target_idx = compute_next_target_index(extend, retract, g_target_idx, MAX_ANGLES);
    if (new_target_idx == g_target_idx) {
        goto exit_button_handling;
    }
    if ((new_target_idx >= MAX_ANGLES) || (new_target_idx < 0)) {
        LOGW("Out of range; ignoring.");
        goto exit_button_handling;
    }

    new_target_position = tick_targets[new_target_idx];

    if (current_motor_command == CMD_STOP) {
        int decided = decide_command_for_target(motors[MASTER_MOTOR].getPosition(),
                                                new_target_position,
                                                CMD_STOP,
                                                CMD_EXTEND,
                                                CMD_RETRACT,
                                                CMD_HOME);
        set_current_command((motor_cmd_t)decided);
        g_target_idx = new_target_idx;
        goto exit_button_handling;
    } else {
        if (!is_target_change_permitted(motors[MASTER_MOTOR].getPosition(),
                                        new_target_position,
                                        current_motor_command,
                                        CMD_EXTEND,
                                        CMD_RETRACT)) {
            display.show_short_message("ERROR:", "LOWERING", 2000);
            new_target_idx = g_target_idx;
        } else {
            LOGI("Change accepted.");
            g_target_idx = new_target_idx;
            if (new_target_idx == 0) {
                set_current_command(CMD_HOME);
            }
        }
    }
exit_button_handling:
    display.update_target_view((g_target_idx * ANGLE_STEP) + LOWEST_ANGLE,
                               compute_progress_percent(motors[MASTER_MOTOR].getPosition(),
                                                        g_progress_start_ticks,
                                                        (int32_t)tick_targets[g_target_idx]));
    display.refresh();
}

void print_state(uint32_t print_rate) {
    if (!throttle(last_print, print_rate)) {
        return;
    }
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
}

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

void isr_extend() {
    if (digitalRead(BUTTON_EXTEND_PIN) == LOW) {
        g_extend_pressed = true;
    }
}

void isr_retract() {
    if (digitalRead(BUTTON_RETRACT_PIN) == LOW) {
        g_retract_pressed = true;
    }
}

int32_t get_master_motor_speed(void) {
    int32_t target = tick_targets[g_target_idx];
    if (current_motor_command == CMD_HOME) {
        target = -1000000;
    }
    uint32_t return_speed = abs(motors[MASTER_MOTOR].getSpeed());
    if ((abs(motors[MASTER_MOTOR].getSpeed()) < MINSPEED) && (current_motor_command != CMD_STOP)) {
        g_accel_phase = true;
    }
    if (g_accel_phase) {
        if ((millis() - delta_acc) > 50) {
            if (abs(motors[MASTER_MOTOR].getSpeed()) == 0) {
                return_speed = MINSPEED;
            }
            return_speed = return_speed + (ACCEL_STEP);
            if (return_speed >= MASTER_MAX) {
                return_speed = MASTER_MAX;
                g_accel_phase = false;
                delta_acc     = 0;
            }
            delta_acc = millis();
        }
    }
    bool target_reached = false;
    if (current_motor_command == CMD_HOME) {
        if (motors_idle_for(2000)) {
            disable_hal_irqs();
            reset_tick_counters(0, true);
            reset_motor_positions();
            display.set_view(LCD_TARGET_VIEW);
            target_reached = true;
        }
    }
    if (current_motor_command == CMD_RETRACT) {
        if ((motors[MASTER_MOTOR].getPosition() <= target)) {
            target_reached = true;
        }
    }
    if ((current_motor_command == CMD_EXTEND)) {
        if ((motors[MASTER_MOTOR].getPosition() >= target)) {
            target_reached = true;
        }
    }
    if (target_reached) {
        return_speed = 0;
        set_current_command(CMD_STOP);
        g_accel_phase = false;
    }
    if ((current_motor_command == CMD_HOME) || (current_motor_command == CMD_RETRACT)) {
        return return_speed * -1;
    }
    return return_speed;
}

bool motors_idle_for(uint32_t ms) {
    for (int m = 0; m < NUM_MOTORS; m++) {
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
