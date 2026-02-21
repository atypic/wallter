#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include <stdarg.h>
#include "hal/gpio_ll.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "display.hpp"
#include "motordriver.hpp"
#include "buttons.hpp"

static const char *TAG = "main";
// Forward declaration for button handler
static void handle_buttons();
static void setup();
static Display display;
static MotorDriver motors[NUM_MOTORS] = {
    MotorDriver(HAL_PWM[0], HAL_DIR[0]),
    MotorDriver(HAL_PWM[1], HAL_DIR[1])
#if NUM_MOTORS > 2
    , MotorDriver(HAL_PWM[2], HAL_DIR[2])
#endif
#if NUM_MOTORS > 3
    , MotorDriver(HAL_PWM[3], HAL_DIR[3])
#endif
};

// Hardware glitch filters
static gpio_glitch_filter_handle_t btn_extend_filter = nullptr;
static gpio_glitch_filter_handle_t btn_retract_filter = nullptr;
static gpio_glitch_filter_handle_t hal_clk_filter[NUM_MOTORS] [[maybe_unused]] = {};

static volatile bool g_extend_pressed = false;
static volatile bool g_retract_pressed = false;
static bool g_boot_menu_requested = false;

// Master motor index and motion constants (tune as needed)
#define MASTER_MOTOR 0
static bool g_accel_phase = false;
static uint64_t delta_acc_ms = 0; // last accel update time in ms

// Software edge filter for HAL clocks (aggressive debounce)
static uint64_t g_last_clk_us[NUM_MOTORS] = {0};
static const uint32_t HAL_CLK_FILTER_US = 80; // ignore edges within 80us

// Hardware glitch filter handles debouncing; no manual checks needed

static void IRAM_ATTR isr_button_extend(void *arg) {
    (void)arg;
    g_extend_pressed = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}
static void IRAM_ATTR isr_button_retract(void *arg) {
    (void)arg;
    g_retract_pressed = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}

static void IRAM_ATTR isr_hal_clk(void *arg) {
    int idx = (int)(intptr_t)arg;
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    uint64_t last_us = g_last_clk_us[idx];
    if (last_us != 0 && (now_us - last_us) < HAL_CLK_FILTER_US) {
        // Ignore spurious fast edges
        return;
    }
    g_last_clk_us[idx] = now_us;
    int cnt = gpio_ll_get_level(&GPIO, (gpio_num_t)HAL_CNT[idx]);
    if (cnt == 0) {
        motors[idx].incrementStepIn();
    } else {
        motors[idx].incrementStepOut();
    }
}

// HAL clock IRQ management
static void enable_hal_irqs() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        gpio_isr_handler_add((gpio_num_t)HAL_CLK[i], isr_hal_clk, (void*)(intptr_t)i);
    }
}

[[maybe_unused]] static void disable_hal_irqs() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        gpio_isr_handler_remove((gpio_num_t)HAL_CLK[i]);
    }
}

// Idle helpers and master speed computation
// Target positions (ticks) for angles (match Arduino flow)
static constexpr int MAX_ANGLES = wallter::kMaxAngles;
static uint32_t g_target_ticks[MAX_ANGLES] = {0};
static uint32_t g_target_idx = 0;
static uint32_t target_ticks_retracting  = 0;
static uint32_t target_ticks_extending   = 0;

static constexpr int CAL_FIRST_ANGLE = wallter::calibration::kFirstAngle;
static constexpr int CAL_LAST_ANGLE  = wallter::calibration::kLastAngle;
static constexpr int CAL_STEP_ANGLE  = wallter::calibration::kStepAngle;
static constexpr int CAL_COUNT       = wallter::calibration::kCount;
static constexpr uint32_t CAL_ADJUST_STEP_TICKS = 50;

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static inline bool read_extend_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}
static inline bool read_retract_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}



// Command identifiers
enum { CMD_STOP = 0, CMD_EXTEND = 1, CMD_RETRACT = 2, CMD_HOME = 3 };
static int current_cmd = CMD_STOP;
static int32_t g_progress_start_ticks = 0;
static uint64_t g_last_error_check_ms = 0;
static uint64_t last_print_ms = 0;
static uint64_t g_last_command_change_ms = 0; // grace period for error checks
static int g_consecutive_stall[NUM_MOTORS] = {0};
static int32_t g_last_steps_in[NUM_MOTORS] = {0};
static int32_t g_last_steps_out[NUM_MOTORS] = {0};

static bool motors_idle_for(uint32_t ms) {
    for (int m = 0; m < NUM_MOTORS; m++) {
        if (motors[m].getIdleDurationMs() < ms) {
            return false;
        }
    }
    return true;
}

static void reset_idle_timer() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        motors[i].resetIdle();
    }
}

static uint8_t compute_progress_percent(int32_t current_pos, int32_t start_pos, int32_t target_pos) {
    if (target_pos == start_pos) {
        return 100;
    }
    int64_t num = (int64_t)current_pos - (int64_t)start_pos;
    int64_t den = (int64_t)target_pos - (int64_t)start_pos;
    if (den == 0) {
        return 0;
    }
    if ((den > 0 && current_pos >= target_pos) || (den < 0 && current_pos <= target_pos)) {
        return 100;
    }
    int32_t pct = (int32_t)((num * 100 + (den > 0 ? den / 2 : -den / 2)) / den);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

static void set_current_command(int cmd) {

    // Short circuit if same command requested.
    if (current_cmd == cmd) {
        return;
    }

    // Else, we transition into a new state.
    // This will have consequences, as the main loop will
    // act on the new state.

    current_cmd = cmd;
    g_last_command_change_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    // Nudge the UI to reflect state changes immediately
    display.refresh();

    // If we have one of the movement commands, 
    // we accept the command.
    if (cmd == CMD_EXTEND || cmd == CMD_RETRACT || cmd == CMD_HOME) {
        display.set_refresh_rate(1.0f);
        reset_idle_timer();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
        target_ticks_extending = 0;
        target_ticks_retracting = 0;
        for (int j = 0; j < NUM_MOTORS; ++j) 
        {
            motors[j].resetSteps(true);
        }


        g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
    } else if (cmd == CMD_STOP) {
        display.set_refresh_rate(30.0f);
        g_progress_start_ticks = motors[MASTER_MOTOR].getPosition();
        for (int i = 0; i < NUM_MOTORS; ++i) {
            motors[i].resetIdle();
        }
    }
}

static void reset_tick_counters(uint32_t m, bool all) {
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

static void reset_motor_positions() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setPosition(0);
    }
}

static int32_t get_master_motor_speed() {
    int32_t target = (int32_t)g_target_ticks[g_target_idx];
    if (current_cmd == CMD_HOME) {
        target = -1000000; // match Arduino behavior
    }

    uint32_t return_speed = (uint32_t)abs(motors[MASTER_MOTOR].getSpeed());
    if ((return_speed < (uint32_t)MINSPEED) && (current_cmd != CMD_STOP)) {
        g_accel_phase = true;
    }
    if (g_accel_phase) {
        uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
        if ((now_ms - delta_acc_ms) > 50ULL) {
            if (return_speed == 0) {
                return_speed = MINSPEED;
            }
            return_speed += ACCEL_STEP;
            if (return_speed >= (uint32_t)MASTER_MAX) {
                return_speed = MASTER_MAX;
                g_accel_phase = false;
                delta_acc_ms = 0;
            }
            delta_acc_ms = now_ms;
        }
    }

    bool target_reached = false;
    int32_t current_pos = motors[MASTER_MOTOR].getPosition();
    if (current_cmd == CMD_HOME) {
        if (motors_idle_for(2000)) {
            // reset tick counters and positions after homing idle
            reset_tick_counters(0, true);
            reset_motor_positions();
            // Match Arduino: disable HAL IRQs after homing completes
            //disable_hal_irqs();
            display.set_view(LCD_TARGET_VIEW);
            target_reached = true;
        }
    } else if (current_cmd == CMD_RETRACT) {
        // If we're retracting and near home, switch to homing to fully run-in
        if (current_pos <= 1000) {
            ESP_LOGI(TAG, "Auto-transition: retract pos=%ld -> CMD_HOME", (long)current_pos);
            set_current_command(CMD_HOME);
        }
        if (current_pos <= (int32_t)target) {
            target_reached = true;
        }
    } else if (current_cmd == CMD_EXTEND) {
        if (current_pos >= (int32_t)target) {
            ESP_LOGI(TAG, "Target reached");
            target_reached = true;
        }
    }

    if (target_reached) {
        return_speed = 0;
        ESP_LOGI(TAG, "Stopping motors as target reached");
        set_current_command(CMD_STOP);
        g_accel_phase = false;
    }

    if ((current_cmd == CMD_HOME) || (current_cmd == CMD_RETRACT)) {
        return -(int32_t)return_speed;
    }
    return (int32_t)return_speed;
}

// ESP-IDF friendly throttle helper using esp_timer
static inline bool throttle(uint64_t &last_ms, uint32_t period_ms) {
    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    if (last_ms == 0) {
        // First call should allow printing
        last_ms = now_ms;
        return true;
    }
    if ((now_ms - last_ms) < (uint64_t)period_ms) {
        return false;
    }
    last_ms = now_ms;
    return true;
}

// Panic: stop motors, show message, and loop
static void panicf(const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].setSpeed(0);
    }
    char line1[17];
    snprintf(line1, sizeof(line1), "PANIC:");
    // Avoid potential hang in show_short_message; use safe path
    display.print(line1, buf);
    uint64_t end_ms = (uint64_t)(esp_timer_get_time() / 1000ULL) + 2000ULL;
    while ((uint64_t)(esp_timer_get_time() / 1000ULL) < end_ms) {
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ESP_LOGE(TAG, "PANIC: %s", buf);
    while (1) {
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void error_check_motor_positions() {
    if (current_cmd == CMD_STOP || current_cmd == CMD_HOME) {
        return;
    }
    if (g_accel_phase) {
        return;
    }
    // Provide grace period after command change to let motors react
    uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    if ((now_ms - g_last_command_change_ms) < 800ULL) {
        return;
    }
    if (!throttle(g_last_error_check_ms, 1000)) {
        return;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t si = motors[i].getStepsIn();
        int32_t so = motors[i].getStepsOut();
        int32_t dsi = si - g_last_steps_in[i];
        int32_t dso = so - g_last_steps_out[i];
        g_last_steps_in[i] = si;
        g_last_steps_out[i] = so;

        int32_t pos = motors[i].getPosition();
        // Skip checks when retracting close to home to avoid encoder jitter false positives
        if (current_cmd == CMD_RETRACT && pos <= 1800) {
            g_consecutive_stall[i] = 0;
            continue;
        }

        // Expect movement in the direction of travel
        bool moved_expected = false;
        if (current_cmd == CMD_RETRACT) moved_expected = (dso > 0);
        else if (current_cmd == CMD_EXTEND) moved_expected = (dsi > 0);

        bool ok;
        if (!moved_expected || g_accel_phase) {
            // Be lenient during low-speed or ambiguous deltas
            ok = motors[i].errorCheck(si, so, /*min_delta=*/1, /*window_ms=*/800);
        } else {
            ok = true;
        }

        if (!ok) {
            g_consecutive_stall[i]++;
            ESP_LOGE(TAG, "MotorDriver: Stall candidate: dir=%d dsi=%ld dso=%ld si=%ld so=%ld pos=%ld",
                     (current_cmd == CMD_RETRACT) ? 1 : -1,
                     (long)dsi, (long)dso, (long)si, (long)so, (long)pos);
        } else {
            g_consecutive_stall[i] = 0;
        }

        if (g_consecutive_stall[i] >= 2) {
            ESP_LOGE(TAG, "Stall: motor %d pos=%ld si=%ld so=%ld dsi=%ld dso=%ld",
                     i, (long)pos, (long)si, (long)so, (long)dsi, (long)dso);
            char msg[32];
            snprintf(msg, sizeof(msg), "M%d STALL p%ld", i, (long)pos);
            panicf("%s", msg);
        }
    }
}

static void print_state(uint32_t print_rate_ms) {
    if (!throttle(last_print_ms, print_rate_ms)) {
        return;
    }
    uint32_t tgt = g_target_ticks[g_target_idx];
    int32_t pos0 = motors[0].getPosition();
    int32_t pos1 = motors[1].getPosition();
    uint32_t so0 = motors[0].getStepsOut();
    uint32_t so1 = motors[1].getStepsOut();
    uint32_t si0 = motors[0].getStepsIn();
    uint32_t si1 = motors[1].getStepsIn();
    uint32_t idle0 = motors[0].getIdleDurationMs();
    uint32_t idle1 = motors[1].getIdleDurationMs();
    ESP_LOGI(TAG, "CMD:%d tgtIdx:%u tgtTicks:%u pos:[%ld %ld] stepsOut:[%lu %lu] stepsIn:[%lu %lu] idleMs:[%u %u]",
             (int)current_cmd,
             (unsigned)g_target_idx,
             (unsigned)tgt,
             (long)pos0,
             (long)pos1,
             (unsigned long)so0,
             (unsigned long)so1,
             (unsigned long)si0,
             (unsigned long)si1,
             (unsigned)idle0,
             (unsigned)idle1);
}

static void configure_gpio() {
    gpio_config_t io = {};
    io.mode = GPIO_MODE_INPUT;
    io.intr_type = GPIO_INTR_NEGEDGE;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask = (1ULL << BUTTON_EXTEND_PIN) | (1ULL << BUTTON_RETRACT_PIN);
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)BUTTON_EXTEND_PIN, isr_button_extend, NULL);
    gpio_isr_handler_add((gpio_num_t)BUTTON_RETRACT_PIN, isr_button_retract, NULL);

    // Create button glitch filters
    gpio_pin_glitch_filter_config_t bcfg = {};
    bcfg.gpio_num = (gpio_num_t)BUTTON_EXTEND_PIN;
    gpio_new_pin_glitch_filter(&bcfg, &btn_extend_filter);
    bcfg.gpio_num = (gpio_num_t)BUTTON_RETRACT_PIN;
    gpio_new_pin_glitch_filter(&bcfg, &btn_retract_filter);

    gpio_config_t clk = {};
    clk.mode = GPIO_MODE_INPUT;
    clk.intr_type = GPIO_INTR_POSEDGE;
    // Ensure HAL clock pins use pull-ups
    clk.pull_up_en = GPIO_PULLUP_ENABLE;
    clk.pull_down_en = GPIO_PULLDOWN_DISABLE;
    uint64_t mask = 0;
    for (int i = 0; i < NUM_MOTORS; ++i) mask |= (1ULL << HAL_CLK[i]);
    clk.pin_bit_mask = mask;
    gpio_config(&clk);
    // Ensure HAL count pins use pull-ups as well
    gpio_config_t cnt = {};
    cnt.mode = GPIO_MODE_INPUT;
    cnt.intr_type = GPIO_INTR_DISABLE;
    cnt.pull_up_en = GPIO_PULLUP_ENABLE;
    cnt.pull_down_en = GPIO_PULLDOWN_DISABLE;
    mask = 0;
    for (int i = 0; i < NUM_MOTORS; ++i) mask |= (1ULL << HAL_CNT[i]);
    cnt.pin_bit_mask = mask;
    gpio_config(&cnt);
    // Optionally create HAL clock pin glitch filters (not enabled by default)
    for (int i = 0; i < NUM_MOTORS; ++i) {
        // Uncomment to enable HAL clock filters if needed
        // gpio_pin_glitch_filter_config_t cfg = {};
        // cfg.gpio_num = (gpio_num_t)HAL_CLK[i];
        // gpio_new_pin_glitch_filter(&cfg, &hal_clk_filter[i]);
    }
    // HAL clock ISR handlers added via enable_hal_irqs()
}

static void motor_control_iteration() {
    // Master/followers speed control
    for (int m = 0; m < NUM_MOTORS; ++m) {
        if (m == MASTER_MOTOR) {
            int new_speed = get_master_motor_speed();
            motors[MASTER_MOTOR].setSpeed(new_speed);
            continue;
        }
        switch (current_cmd) {
            case CMD_STOP:
                motors[m].setSpeed(0);
                motors[m].pidReset();
                break;
            case CMD_EXTEND: {
                int32_t spd = motors[m].computeFollow(false, g_accel_phase, abs(motors[MASTER_MOTOR].getSpeed()), motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            case CMD_RETRACT: {
                int32_t spd = motors[m].computeFollow(true, g_accel_phase, abs(motors[MASTER_MOTOR].getSpeed()), motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            case CMD_HOME: {
                int32_t spd = motors[m].computeFollow(true, g_accel_phase, abs(motors[MASTER_MOTOR].getSpeed()), motors[MASTER_MOTOR]);
                motors[m].setSpeed(spd);
                break;
            }
            default:
                ESP_LOGE(TAG, "Invalid command.");
                panicf("BAD CMD");
        }
    }
}

static void run_homing_blocking(uint32_t timeout_ms) {
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);
    display.trigger_refresh();
    reset_idle_timer();
    set_current_command(CMD_HOME);

    uint64_t start = now_ms();
    while (current_cmd != CMD_STOP) {
        motor_control_iteration();
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
        if ((now_ms() - start) > timeout_ms) {
            panicf("HOME TIMEOUT");
        }
    }
}

enum BootMenuChoice { MENU_CALIBRATE = 0, MENU_SELF_TEST = 1 };

static BootMenuChoice run_boot_menu() {
    int sel = 0;
    bool prev_up = false;
    bool prev_dn = false;
    bool prev_both = false;
    display.set_refresh_rate(0.2f);

    auto render = [&]() {
        if (sel == 0) {
            display.print(">Calibrate", " Run self test");
        } else {
            display.print(" Calibrate", ">Run self test");
        }
    };
    render();

    // Wait for initial release to avoid auto-activating due to boot hold
    while (read_extend_pressed() || read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    prev_up = false;
    prev_dn = false;
    prev_both = false;

    while (1) {
        bool up = read_extend_pressed();
        bool dn = read_retract_pressed();
        bool both = up && dn;

        if (!both) {
            if (up && !prev_up) {
                sel = (sel + 1) % 2;
                render();
            }
            if (dn && !prev_dn) {
                sel = (sel + 1) % 2;
                render();
            }
        }
        if (both && !prev_both) {
            // Confirm selection
            while (read_extend_pressed() || read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            return (sel == 0) ? MENU_CALIBRATE : MENU_SELF_TEST;
        }

        prev_up = up;
        prev_dn = dn;
        prev_both = both;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

static void run_calibration_mode() {
    // Start by homing fully ("board all the way back")
    run_homing_blocking(/*timeout_ms=*/45000);

    // Calibration loop for 20..60 degrees in 5 degree steps
    bool prev_up = false;
    bool prev_dn = false;
    bool prev_both = false;

    uint32_t last_ui_ticks = 0xFFFFFFFFu;
    int last_ui_angle = -1;
    uint64_t last_ui_ms = 0;

    for (int angle = CAL_FIRST_ANGLE; angle <= CAL_LAST_ANGLE; angle += CAL_STEP_ANGLE) {
        int idx = wallter::angle_to_index(angle);
        if (idx < 0) {
            panicf("BAD ANG %d", angle);
        }
        g_target_idx = (uint32_t)idx;
        // Ensure we are stopped before starting adjustments for this angle
        set_current_command(CMD_STOP);

        while (1) {
            // Drive motors toward current target ticks for this angle
            int32_t pos = motors[MASTER_MOTOR].getPosition();
            uint32_t tgt = g_target_ticks[g_target_idx];
            if (pos < (int32_t)tgt) {
                set_current_command(CMD_EXTEND);
            } else if (pos > (int32_t)tgt) {
                set_current_command(CMD_RETRACT);
            } else {
                set_current_command(CMD_STOP);
            }

            motor_control_iteration();

            // UI: "20: <hal cnt>"
            char line1[17];
            snprintf(line1, sizeof(line1), "%d: %lu", angle, (unsigned long)g_target_ticks[g_target_idx]);
            // Avoid hammering I2C: only update when changed or periodically.
            uint64_t tnow = now_ms();
            uint32_t cur_ticks = g_target_ticks[g_target_idx];
            if (angle != last_ui_angle || cur_ticks != last_ui_ticks || (tnow - last_ui_ms) > 250ULL) {
                display.print(line1, "");
                last_ui_angle = angle;
                last_ui_ticks = cur_ticks;
                last_ui_ms = tnow;
            }

            bool up = read_extend_pressed();
            bool dn = read_retract_pressed();
            bool both = up && dn;

            if (!both) {
                if (up && !prev_up) {
                    g_target_ticks[g_target_idx] += CAL_ADJUST_STEP_TICKS;
                }
                if (dn && !prev_dn) {
                    if (g_target_ticks[g_target_idx] >= CAL_ADJUST_STEP_TICKS) {
                        g_target_ticks[g_target_idx] -= CAL_ADJUST_STEP_TICKS;
                    } else {
                        g_target_ticks[g_target_idx] = 0;
                    }
                }
            }

            if (both && !prev_both) {
                // Store current calibration (angles 20..60)
                wallter::calibration::save_from_target_ticks(g_target_ticks, MAX_ANGLES);
                display.print("stored", "");
                uint64_t end = now_ms() + 600ULL;
                while (now_ms() < end) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                // Wait for button release so we don't double-trigger
                while (read_extend_pressed() || read_retract_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                break; // next angle
            }

            prev_up = up;
            prev_dn = dn;
            prev_both = both;
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    }

    // Derive 15 degrees entry again, and return to home.
    {
        int idx20 = wallter::angle_to_index(20);
        int idx15 = wallter::angle_to_index(15);
        if (idx20 >= 0 && idx15 >= 0) {
            g_target_ticks[idx15] = g_target_ticks[idx20] / 2U;
        }
        g_target_ticks[0] = 0;
    }

    display.print("Cal complete", "Homing...");
    run_homing_blocking(/*timeout_ms=*/45000);
}

static void run_self_test_sequence();

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Init");

    // NVS for calibration persistence
    wallter::calibration::init_nvs();
    wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);

    setup();

    // Boot menu: hold EXTEND while starting (latched during setup after GPIO config)
    bool boot_extend = g_boot_menu_requested;
    bool boot_retract = read_retract_pressed();
    if (boot_extend) {
        // Clear any latched interrupt flags before entering menu.
        g_extend_pressed = false;
        g_retract_pressed = false;

        BootMenuChoice choice = run_boot_menu();
        if (choice == MENU_CALIBRATE) {
            display.print("Calibration", "Starting...");
            run_calibration_mode();
        } else {
            run_self_test_sequence();
        }

        // Re-load calibration after a calibration run (ensures interpolation is applied consistently)
        wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);
        g_extend_pressed = false;
        g_retract_pressed = false;
        g_boot_menu_requested = false;
    } else {
        (void)boot_retract;
        ESP_LOGI(TAG, "Normal boot: skipping self-test (use boot menu)");
    }

    // Self testing done or skipped, let's go home.
    // Post-setup initial view
    display.update_target_view(LOWEST_ANGLE, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);

    // Make sure we reset idle timer since we've been using the motors.
    reset_idle_timer();
    set_current_command(CMD_HOME);

    // this is the main loop
    while (1) {
        // Handle buttons in a dedicated function (Arduino-style)
        handle_buttons();

        motor_control_iteration();

        // Periodic logging and stall checks, update target view
        print_state(500);
        uint32_t target_ticks = g_target_ticks[g_target_idx];
        int32_t pos           = motors[MASTER_MOTOR].getPosition();
        uint8_t pct = compute_progress_percent(pos, g_progress_start_ticks, (int32_t)target_ticks);
        float tgt_angle = (g_target_idx == 0) ? LOWEST_ANGLE : (g_target_idx * ANGLE_STEP + LOWEST_ANGLE);
        display.update_target_view(tgt_angle, pct);
        error_check_motor_positions();
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Extracted setup routine (Arduino-style)
static void setup() {
    // Configure GPIO early so the boot-menu hold can be latched quickly.
    configure_gpio();
    // Enable button glitch filters for simple debounce
    if (btn_extend_filter) {
        gpio_glitch_filter_enable(btn_extend_filter);
    }
    if (btn_retract_filter) {
        gpio_glitch_filter_enable(btn_retract_filter);
    }
    // Latch initial button states (active-low)
    g_boot_menu_requested = (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
    if (g_boot_menu_requested) {
        g_extend_pressed = true;
    }
    if (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0) {
        g_retract_pressed = true;
    }

    // Init motors and display
    for (int i = 0; i < NUM_MOTORS; ++i) motors[i].init();
    vTaskDelay(pdMS_TO_TICKS(500));
    display.init();
    vTaskDelay(pdMS_TO_TICKS(500));
    display.set_refresh_rate(1.0f);
    // Run startup animation like old display
    display.startup_animation();

    // Add HAL clock IRQs (equivalent to Arduino attachInterrupt on HAL_CLK)
    enable_hal_irqs();

    // Reset counters and positions like Arduino setup()
    reset_tick_counters(0, true);
    reset_motor_positions();

    // (button filters + initial button latching handled before animation)

    // Configure follower PID motors
    for (int p = 1; p < NUM_MOTORS; p++) {
        motors[p].pidBegin(8, 0.001, 0);
        motors[p].pidConfigureLimits(MINSPEED, MAXSPEED);
        motors[p].pidSetSampleTime(50);
        motors[p].pidStart();
    }

    // Initialize stall tracking baselines
    for (int i = 0; i < NUM_MOTORS; ++i) {
        g_consecutive_stall[i] = 0;
        g_last_steps_in[i] = motors[i].getStepsIn();
        g_last_steps_out[i] = motors[i].getStepsOut();
    }

    set_current_command(CMD_HOME);
    g_target_idx = 0;

    // Button ISRs configured in configure_gpio().

    // Initial display view matching Arduino setup
    display.update_target_view(LOWEST_ANGLE, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);
}

// Replicate Arduino handle_buttons() logic without blocking loops
static void handle_buttons() {
    bool extend  = g_extend_pressed;
    bool retract = g_retract_pressed;
    // Clear latched flags
    g_extend_pressed  = false;
    g_retract_pressed = false;

    // Exit early during homing
    if (current_cmd == CMD_HOME) {
        return;
    }

    int new_idx = compute_next_target_index(extend, retract, (int)g_target_idx, MAX_ANGLES);
    if (new_idx == (int)g_target_idx) {
        return; // no change
    }
    if (new_idx < 0 || new_idx >= MAX_ANGLES) {
        ESP_LOGW(TAG, "Out of range; ignoring.");
        return;
    }

    uint32_t new_target_pos = g_target_ticks[new_idx];
    int32_t pos = motors[MASTER_MOTOR].getPosition();

    if (current_cmd == CMD_STOP) {
        int decided = decide_command_for_target(pos, new_target_pos, CMD_STOP, CMD_EXTEND, CMD_RETRACT, CMD_HOME);
        set_current_command(decided);
        g_target_idx = (uint32_t)new_idx;
        return;
    }

    if (!is_target_change_permitted(pos, new_target_pos, current_cmd, CMD_EXTEND, CMD_RETRACT)) {
        display.show_short_message(const_cast<char*>("ERROR:"), const_cast<char*>("LOWERING"), 2000);
        return;
    }
    ESP_LOGI(TAG, "Change accepted.");
    g_target_idx = (uint32_t)new_idx;
    if (new_idx == 0) {
        // When selecting the bottom-most index, retract toward zero
        // and only enter CMD_HOME automatically once idle-at-zero is detected.
        set_current_command(CMD_RETRACT);
    }
    display.trigger_refresh();
}

// Extracted self-test to re-use in boot menu.
static void run_self_test_sequence() {
    const int test_speed = MINSPEED;
    const int test_time  = 1000;
    ESP_LOGI(TAG, "Self-test started.");
    ESP_LOGI(TAG, "ST: init pos=[%ld %ld] si=[%lu %lu] so=[%lu %lu]",
         (long)motors[0].getPosition(), (long)motors[1].getPosition(),
         (unsigned long)motors[0].getStepsIn(), (unsigned long)motors[1].getStepsIn(),
         (unsigned long)motors[0].getStepsOut(), (unsigned long)motors[1].getStepsOut());
    display.print("Self test 1", "Extending.");

    // Retract then extend
    ESP_LOGI(TAG, "ST: retract phase start, speed=%d", test_speed);
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(-test_speed);
    ESP_LOGI(TAG, "ST: retract phase running...");
    vTaskDelay(pdMS_TO_TICKS(test_time + 500));
    ESP_LOGI(TAG, "ST: retract done. si=[%lu %lu] so=[%lu %lu]",
         (unsigned long)motors[0].getStepsIn(), (unsigned long)motors[1].getStepsIn(),
         (unsigned long)motors[0].getStepsOut(), (unsigned long)motors[1].getStepsOut());
    reset_tick_counters(0, true);
    ESP_LOGI(TAG, "ST: counters reset. si=[%lu %lu] so=[%lu %lu]",
         (unsigned long)motors[0].getStepsIn(), (unsigned long)motors[1].getStepsIn(),
         (unsigned long)motors[0].getStepsOut(), (unsigned long)motors[1].getStepsOut());
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(test_speed);
    ESP_LOGI(TAG, "ST: extend phase start, speed=%d", test_speed);
    vTaskDelay(pdMS_TO_TICKS(test_time));
    ESP_LOGI(TAG, "ST: extend done. si=[%lu %lu] so=[%lu %lu]",
         (unsigned long)motors[0].getStepsIn(), (unsigned long)motors[1].getStepsIn(),
         (unsigned long)motors[0].getStepsOut(), (unsigned long)motors[1].getStepsOut());
    display.print("Phase 1 done", "Checking...");
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(0);
    ESP_LOGI(TAG, "ST: motors stopped after phase 1.");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].getStepsOut() < 100U) {
            ESP_LOGE(TAG, "Self-test failed: motor %d si:%d so:%d", i, (int)motors[i].getStepsIn(), (int)motors[i].getStepsOut());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SO:%d", i, (int)motors[i].getStepsOut());
            display.print("SELF TEST FAILED", tbuf);
            panicf("M%d NO OUT", i);
        }
    }

    display.print("Self test 2", "Retracting.");
    ESP_LOGI(TAG, "ST: phase 2 retract start.");
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(-test_speed);
    vTaskDelay(pdMS_TO_TICKS(test_time));
    ESP_LOGI(TAG, "ST: phase 2 retract done. si=[%lu %lu] so=[%lu %lu]",
         (unsigned long)motors[0].getStepsIn(), (unsigned long)motors[1].getStepsIn(),
         (unsigned long)motors[0].getStepsOut(), (unsigned long)motors[1].getStepsOut());
    display.print("Phase 2 done", "Checking...");
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].setSpeed(0);
    ESP_LOGI(TAG, "ST: motors stopped after phase 2.");
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (motors[i].getStepsIn() < 100U) {
            ESP_LOGE(TAG, "Self-test failed: motor %d si:%d so:%d", i, (int)motors[i].getStepsIn(), (int)motors[i].getStepsOut());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SI:%d", i, (int)motors[i].getStepsIn());
            display.print("SELF TEST FAILED:", tbuf);
            panicf("M%d NO IN", i);
        }
    }
    bool final_success = true;
    for (int i = 0; i < NUM_MOTORS; ++i) {
        if (abs((int)motors[i].getStepsOut() - (int)motors[i].getStepsIn()) > 200) {
            final_success = false;
        }
    }
    if (!final_success) {
        display.print("FAIL: BAD SENSOR DATA", "Check connections.");
        panicf("CHECK CONNECTIONS");
    }
    display.print("Self test", "PASSED");
    uint64_t end_ms = now_ms() + 1500ULL;
    while (now_ms() < end_ms) {
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
