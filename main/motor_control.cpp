#include "motor_control.hpp"

#include "app_config.hpp"

#include "commands.hpp"
#include "display.hpp"
#include "motordriver.hpp"
#include "buttons.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

namespace wallter::control {

static const char *TAG = "control";

static Context g_ctx;

static int g_current_cmd = wallter::CMD_STOP;
static bool g_accel_phase = false;
static uint64_t g_delta_acc_ms = 0;

static int32_t g_progress_start_ticks = 0;
static uint64_t g_last_error_check_ms = 0;
static uint64_t g_last_print_ms = 0;
static uint64_t g_last_command_change_ms = 0;
static int g_consecutive_stall[NUM_MOTORS] = {0};
static int32_t g_last_steps_in[NUM_MOTORS] = {0};
static int32_t g_last_steps_out[NUM_MOTORS] = {0};

// Follow safety: only let follower motors move once the master has produced
// at least one encoder edge in the current command.
static bool g_master_moved_since_cmd = false;
static uint32_t g_master_last_si = 0;
static uint32_t g_master_last_so = 0;

// Homing stall/timeout tracking (used to avoid getting stuck in CMD_HOME).
static uint64_t g_home_stall_last_ms = 0;
static uint32_t g_home_stall_last_steps = 0;
static uint32_t g_home_stall_accum_ms = 0;

static constexpr uint32_t kHomeTimeoutMs = 45000;
static constexpr uint32_t kHomeStallWindowMs = 1000;
static constexpr uint32_t kHomeStallMinDeltaStepsFloor = 3;
static constexpr uint32_t kHomeStallNeededMs = 2000;

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static inline bool throttle(uint64_t &last_ms, uint32_t period_ms) {
    uint64_t n = now_ms();
    if (last_ms == 0) {
        last_ms = n;
        return true;
    }
    if ((n - last_ms) < (uint64_t)period_ms) {
        return false;
    }
    last_ms = n;
    return true;
}

static void reset_home_stall_tracking() {
    g_home_stall_last_ms = 0;
    g_home_stall_last_steps = 0;
    g_home_stall_accum_ms = 0;
}

static uint32_t home_min_delta_steps_for_speed(int32_t speed) {
    uint32_t abs_spd = (uint32_t)((speed < 0) ? -speed : speed);
    // Heuristic: when commanding higher speed, we expect more encoder ticks per second.
    // This makes homing tolerant to a few noise edges at the hard-stop.
    uint32_t scaled = abs_spd / 8U; // 255 -> 31
    if (scaled < kHomeStallMinDeltaStepsFloor) scaled = kHomeStallMinDeltaStepsFloor;
    return scaled;
}

static bool master_stalled_for(uint32_t needed_ms, bool retract_direction, int32_t commanded_speed) {
    uint64_t n = now_ms();
    if (g_home_stall_last_ms == 0) {
        g_home_stall_last_ms = n;
        g_home_stall_last_steps = retract_direction
                                  ? g_ctx.motors[g_ctx.master_motor_index].getStepsIn()
                                  : g_ctx.motors[g_ctx.master_motor_index].getStepsOut();
        g_home_stall_accum_ms = 0;
        return false;
    }

    uint64_t elapsed = n - g_home_stall_last_ms;
    if (elapsed < (uint64_t)kHomeStallWindowMs) {
        return false;
    }

    uint32_t steps = retract_direction
                     ? g_ctx.motors[g_ctx.master_motor_index].getStepsIn()
                     : g_ctx.motors[g_ctx.master_motor_index].getStepsOut();
    uint32_t delta = (steps >= g_home_stall_last_steps) ? (steps - g_home_stall_last_steps) : 0;
    g_home_stall_last_steps = steps;
    g_home_stall_last_ms = n;

    uint32_t min_delta = home_min_delta_steps_for_speed(commanded_speed);
    if (delta < min_delta) {
        // Treat low step rate as stall (tolerant of occasional noise edges).
        g_home_stall_accum_ms += kHomeStallWindowMs;
    } else {
        g_home_stall_accum_ms = 0;
    }

    return g_home_stall_accum_ms >= needed_ms;
}

static bool motors_idle_for(uint32_t ms) {
    for (int m = 0; m < g_ctx.num_motors; m++) {
        if (g_ctx.motors[m].getIdleDurationMs() < ms) {
            return false;
        }
    }
    return true;
}

void reset_idle_timer() {
    for (int i = 0; i < g_ctx.num_motors; ++i) {
        g_ctx.motors[i].resetIdle();
    }
}

void reset_tick_counters(uint32_t m, bool all) {
    (void)m;
    if (all) {
        for (int j = 0; j < g_ctx.num_motors; j++) {
            g_ctx.motors[j].resetSteps(true);
        }
    } else if ((int)m < g_ctx.num_motors) {
        g_ctx.motors[m].resetSteps(true);
    }
}

void reset_motor_positions() {
    for (int i = 0; i < g_ctx.num_motors; i++) {
        g_ctx.motors[i].setPosition(0);
    }
}

uint8_t compute_progress_percent(int32_t current_pos, int32_t start_pos, int32_t target_pos) {
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

void panicf(const char *fmt, ...) {
    char buf[64];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    for (int i = 0; i < g_ctx.num_motors; i++) {
        g_ctx.motors[i].setSpeed(0);
    }

    char line1[17];
    snprintf(line1, sizeof(line1), "PANIC:");
    g_ctx.display->print(line1, buf);

    uint64_t end_ms = now_ms() + 2000ULL;
    while (now_ms() < end_ms) {
        g_ctx.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGE(TAG, "PANIC: %s", buf);
    while (1) {
        g_ctx.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init(const Context &ctx) {
    g_ctx = ctx;

    // Sanitize limits.
    if (g_ctx.min_target_idx < 0) g_ctx.min_target_idx = 0;
    if (g_ctx.max_target_idx <= 0 || g_ctx.max_target_idx >= g_ctx.max_angles) {
        g_ctx.max_target_idx = g_ctx.max_angles - 1;
    }
    if (g_ctx.min_target_idx > g_ctx.max_target_idx) {
        g_ctx.min_target_idx = 0;
        g_ctx.max_target_idx = g_ctx.max_angles - 1;
    }

    // Baselines for stall detection.
    for (int i = 0; i < g_ctx.num_motors; ++i) {
        g_consecutive_stall[i] = 0;
        g_last_steps_in[i] = g_ctx.motors[i].getStepsIn();
        g_last_steps_out[i] = g_ctx.motors[i].getStepsOut();
    }

    g_master_last_si = g_ctx.motors[g_ctx.master_motor_index].getStepsIn();
    g_master_last_so = g_ctx.motors[g_ctx.master_motor_index].getStepsOut();
    g_master_moved_since_cmd = false;

    g_current_cmd = wallter::CMD_STOP;
    g_accel_phase = false;
    g_delta_acc_ms = 0;
    g_progress_start_ticks = g_ctx.motors[g_ctx.master_motor_index].getPosition();
    g_last_command_change_ms = now_ms();

}

int command() {
    return g_current_cmd;
}

int32_t master_position() {
    return g_ctx.motors[g_ctx.master_motor_index].getPosition();
}

int32_t progress_start_ticks() {
    return g_progress_start_ticks;
}

void set_command(int cmd) {
    if (g_current_cmd == cmd) {
        return;
    }

    g_current_cmd = cmd;
    g_last_command_change_ms = now_ms();

    g_ctx.display->refresh();

    if (cmd == wallter::CMD_HOME) {
        reset_home_stall_tracking();
    }

    if (cmd == wallter::CMD_EXTEND || cmd == wallter::CMD_RETRACT || cmd == wallter::CMD_HOME) {
        g_ctx.display->set_refresh_rate(1.0f);
        reset_idle_timer();
        for (int i = 0; i < g_ctx.num_motors; ++i) {
            g_ctx.motors[i].resetIdle();
        }
        for (int j = 0; j < g_ctx.num_motors; ++j) {
            g_ctx.motors[j].resetSteps(true);
        }
        g_master_last_si = g_ctx.motors[g_ctx.master_motor_index].getStepsIn();
        g_master_last_so = g_ctx.motors[g_ctx.master_motor_index].getStepsOut();
        g_master_moved_since_cmd = false;
        g_progress_start_ticks = g_ctx.motors[g_ctx.master_motor_index].getPosition();
    } else if (cmd == wallter::CMD_STOP) {
        g_ctx.display->set_refresh_rate(30.0f);
        g_progress_start_ticks = g_ctx.motors[g_ctx.master_motor_index].getPosition();
        for (int i = 0; i < g_ctx.num_motors; ++i) {
            g_ctx.motors[i].resetIdle();
        }
        g_master_moved_since_cmd = false;
    }
}

static int32_t get_master_motor_speed() {
    int32_t target = (int32_t)g_ctx.target_ticks[*g_ctx.target_idx];
    if (g_current_cmd == wallter::CMD_HOME) {
        target = -1000000;
    }

    uint32_t return_speed = (uint32_t)abs(g_ctx.motors[g_ctx.master_motor_index].getSpeed());
    if ((return_speed < (uint32_t)MINSPEED) && (g_current_cmd != wallter::CMD_STOP)) {
        g_accel_phase = true;
    }

    if (g_accel_phase) {
        uint64_t n = now_ms();
        if ((n - g_delta_acc_ms) > 50ULL) {
            if (return_speed == 0) {
                return_speed = MINSPEED;
            }
            return_speed += ACCEL_STEP;
            if (return_speed >= (uint32_t)MASTER_MAX) {
                return_speed = MASTER_MAX;
                g_accel_phase = false;
                g_delta_acc_ms = 0;
            }
            g_delta_acc_ms = n;
        }
    }

    bool target_reached = false;
    int32_t current_pos = g_ctx.motors[g_ctx.master_motor_index].getPosition();

    if (g_current_cmd == wallter::CMD_HOME) {
        // Guard: never allow homing to run forever in the non-blocking main loop.
        // Mirror the blocking helper's timeout behavior.
        uint64_t n = now_ms();
        if ((n - g_last_command_change_ms) > (uint64_t)kHomeTimeoutMs) {
            ESP_LOGE(TAG, "Homing timeout (%ums)", (unsigned)kHomeTimeoutMs);
            g_ctx.display->show_short_message(const_cast<char *>("ERROR:"),
                                              const_cast<char *>("HOME TIMEOUT"),
                                              2500);
            g_ctx.display->set_view(LCD_TARGET_VIEW);
            target_reached = true;
        }

        if (!target_reached) {
            // Retract until stall indicates hard-stop ("bottomed out").
            // When bottomed out, reset all tick counters and motor positions to 0.
            if (master_stalled_for(kHomeStallNeededMs,
                                   /*retract_direction=*/true,
                                   /*commanded_speed=*/-(int32_t)return_speed)) {
                reset_tick_counters(0, true);
                reset_motor_positions();
                g_ctx.display->set_view(LCD_TARGET_VIEW);
                target_reached = true;
            }
        }
    } else if (g_current_cmd == wallter::CMD_RETRACT) {
        // Only do the near-home auto-transition when we're actually heading to home.
        // Otherwise small-angle targets (low tick values) can incorrectly trigger homing.
        if (*g_ctx.target_idx == 0 && current_pos <= 1000) {
            ESP_LOGI(TAG, "Auto-transition: retract-to-home pos=%ld -> CMD_HOME", (long)current_pos);
            set_command(wallter::CMD_HOME);
        }
        if (current_pos <= (int32_t)target) {
            target_reached = true;
        }
    } else if (g_current_cmd == wallter::CMD_EXTEND) {
        if (current_pos >= (int32_t)target) {
            ESP_LOGI(TAG, "Target reached");
            target_reached = true;
        }
    }

    if (target_reached) {
        return_speed = 0;
        ESP_LOGI(TAG, "Stopping motors as target reached");
        set_command(wallter::CMD_STOP);
        g_accel_phase = false;
    }

    if (g_current_cmd == wallter::CMD_RETRACT) {
        return -(int32_t)return_speed;
    }
    if (g_current_cmd == wallter::CMD_HOME) {
        // Homing always retracts to the hard-stop.
        return -(int32_t)return_speed;
    }
    return (int32_t)return_speed;
}

void iterate() {
    // Update master movement latch from encoder steps.
    {
        uint32_t si = g_ctx.motors[g_ctx.master_motor_index].getStepsIn();
        uint32_t so = g_ctx.motors[g_ctx.master_motor_index].getStepsOut();
        if (si != g_master_last_si || so != g_master_last_so) {
            g_master_moved_since_cmd = true;
            g_master_last_si = si;
            g_master_last_so = so;
        }
    }

    for (int m = 0; m < g_ctx.num_motors; ++m) {
        if (m == g_ctx.master_motor_index) {
            int new_speed = get_master_motor_speed();
            g_ctx.motors[g_ctx.master_motor_index].setSpeed(new_speed);
            continue;
        }

        // Safety: don't let followers move unless the master has shown real motion
        // (encoder edges) since this command started.
        if (g_current_cmd != wallter::CMD_STOP && !g_master_moved_since_cmd && abs(g_ctx.motors[g_ctx.master_motor_index].getSpeed()) > 0) {
            g_ctx.motors[m].setSpeed(0);
            g_ctx.motors[m].pidReset();
            continue;
        }

        switch (g_current_cmd) {
            case wallter::CMD_STOP:
                g_ctx.motors[m].setSpeed(0);
                g_ctx.motors[m].pidReset();
                break;
            case wallter::CMD_EXTEND: {
                int32_t spd = g_ctx.motors[m].computeFollow(false,
                                                           g_accel_phase,
                                                           abs(g_ctx.motors[g_ctx.master_motor_index].getSpeed()),
                                                           g_ctx.motors[g_ctx.master_motor_index]);
                g_ctx.motors[m].setSpeed(spd);
                break;
            }
            case wallter::CMD_RETRACT:
            case wallter::CMD_HOME: {
                bool reverse = (g_current_cmd == wallter::CMD_RETRACT) || (g_current_cmd == wallter::CMD_HOME);
                int32_t spd = g_ctx.motors[m].computeFollow(reverse,
                                                           g_accel_phase,
                                                           abs(g_ctx.motors[g_ctx.master_motor_index].getSpeed()),
                                                           g_ctx.motors[g_ctx.master_motor_index]);
                g_ctx.motors[m].setSpeed(spd);
                break;
            }
            default:
                ESP_LOGE(TAG, "Invalid command.");
                panicf("BAD CMD");
        }
    }
}

void run_homing_blocking(uint32_t timeout_ms) {
    g_ctx.display->update_homing_view();
    g_ctx.display->set_view(LCD_HOMING_VIEW);
    g_ctx.display->trigger_refresh();

    reset_idle_timer();
    set_command(wallter::CMD_HOME);

    uint64_t start = now_ms();
    while (g_current_cmd != wallter::CMD_STOP) {
        iterate();
        g_ctx.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
        if ((now_ms() - start) > timeout_ms) {
            panicf("HOME TIMEOUT");
        }
    }
}

void print_state(uint32_t print_rate_ms) {
    if (!throttle(g_last_print_ms, print_rate_ms)) {
        return;
    }

    uint32_t tgt = g_ctx.target_ticks[*g_ctx.target_idx];
    int32_t pos0 = g_ctx.motors[0].getPosition();
    int32_t pos1 = g_ctx.motors[1].getPosition();
    uint32_t so0 = g_ctx.motors[0].getStepsOut();
    uint32_t so1 = g_ctx.motors[1].getStepsOut();
    uint32_t si0 = g_ctx.motors[0].getStepsIn();
    uint32_t si1 = g_ctx.motors[1].getStepsIn();
    uint32_t idle0 = g_ctx.motors[0].getIdleDurationMs();
    uint32_t idle1 = g_ctx.motors[1].getIdleDurationMs();
    int32_t spd0 = g_ctx.motors[0].getSpeed();
    int32_t spd1 = g_ctx.motors[1].getSpeed();

    ESP_LOGI(TAG,
             "CMD:%d tgtIdx:%u tgtTicks:%u pos:[%ld %ld] spd:[%ld %ld] stepsOut:[%lu %lu] stepsIn:[%lu %lu] idleMs:[%u %u]",
             (int)g_current_cmd,
             (unsigned)*g_ctx.target_idx,
             (unsigned)tgt,
             (long)pos0,
             (long)pos1,
             (long)spd0,
             (long)spd1,
             (unsigned long)so0,
             (unsigned long)so1,
             (unsigned long)si0,
             (unsigned long)si1,
             (unsigned)idle0,
             (unsigned)idle1);
}

void error_check_motor_positions() {
    if (g_current_cmd == wallter::CMD_STOP || g_current_cmd == wallter::CMD_HOME) {
        return;
    }
    if (g_accel_phase) {
        return;
    }

    uint64_t n = now_ms();
    if ((n - g_last_command_change_ms) < 800ULL) {
        return;
    }

    if (!throttle(g_last_error_check_ms, 1000)) {
        return;
    }

    for (int i = 0; i < g_ctx.num_motors; i++) {
        int32_t si = g_ctx.motors[i].getStepsIn();
        int32_t so = g_ctx.motors[i].getStepsOut();
        int32_t dsi = si - g_last_steps_in[i];
        int32_t dso = so - g_last_steps_out[i];
        g_last_steps_in[i] = si;
        g_last_steps_out[i] = so;

        int32_t pos = g_ctx.motors[i].getPosition();
        if (g_current_cmd == wallter::CMD_RETRACT && pos <= 1800) {
            g_consecutive_stall[i] = 0;
            continue;
        }

        bool moved_expected = false;
        // With our step accounting: EXTEND increments stepsOut, RETRACT increments stepsIn.
        if (g_current_cmd == wallter::CMD_RETRACT) moved_expected = (dsi > 0);
        else if (g_current_cmd == wallter::CMD_EXTEND) moved_expected = (dso > 0);

        bool ok;
        if (!moved_expected || g_accel_phase) {
            ok = g_ctx.motors[i].errorCheck(si, so, /*min_delta=*/1, /*window_ms=*/800);
        } else {
            ok = true;
        }

        if (!ok) {
            g_consecutive_stall[i]++;
            ESP_LOGE(TAG,
                     "MotorDriver: Stall candidate: dir=%d dsi=%ld dso=%ld si=%ld so=%ld pos=%ld",
                     (g_current_cmd == wallter::CMD_RETRACT) ? 1 : -1,
                     (long)dsi,
                     (long)dso,
                     (long)si,
                     (long)so,
                     (long)pos);
        } else {
            g_consecutive_stall[i] = 0;
        }

        if (g_consecutive_stall[i] >= 2) {
            ESP_LOGE(TAG,
                     "Stall: motor %d pos=%ld si=%ld so=%ld dsi=%ld dso=%ld",
                     i,
                     (long)pos,
                     (long)si,
                     (long)so,
                     (long)dsi,
                     (long)dso);

            // Safety: stop instead of hard-panicking so we can keep debugging in the field.
            for (int m = 0; m < g_ctx.num_motors; ++m) {
                g_ctx.motors[m].setSpeed(0);
                g_ctx.motors[m].pidReset();
            }
            set_command(wallter::CMD_STOP);
            char msg[32];
            snprintf(msg, sizeof(msg), "M%d NO ENCODER", i);
            g_ctx.display->show_short_message(const_cast<char *>("ERROR:"), msg, 2500);
            g_consecutive_stall[i] = 0;
            return;
        }
    }
}

void handle_buttons(bool extend_event, bool retract_event) {
    if (g_current_cmd == wallter::CMD_HOME) {
        return;
    }

    int new_idx = compute_next_target_index(extend_event,
                                            retract_event,
                                            (int)*g_ctx.target_idx,
                                            g_ctx.max_angles,
                                            g_ctx.min_target_idx,
                                            g_ctx.max_target_idx);
    if (new_idx == (int)*g_ctx.target_idx) {
        return;
    }
    if (new_idx < 0 || new_idx >= g_ctx.max_angles) {
        ESP_LOGW(TAG, "Out of range; ignoring.");
        return;
    }

    uint32_t new_target_pos = g_ctx.target_ticks[new_idx];
    int32_t pos = g_ctx.motors[g_ctx.master_motor_index].getPosition();

    if (g_current_cmd == wallter::CMD_STOP) {
        int decided = decide_command_for_target(pos,
                                               new_target_pos,
                                               wallter::CMD_STOP,
                                               wallter::CMD_EXTEND,
                                               wallter::CMD_RETRACT,
                                               wallter::CMD_HOME);
        set_command(decided);
        *g_ctx.target_idx = (uint32_t)new_idx;
        return;
    }

    if (!is_target_change_permitted(pos, new_target_pos, g_current_cmd, wallter::CMD_EXTEND, wallter::CMD_RETRACT)) {
        g_ctx.display->show_short_message(const_cast<char *>("ERROR:"), const_cast<char *>("LOWERING"), 2000);
        return;
    }

    ESP_LOGI(TAG, "Change accepted.");
    *g_ctx.target_idx = (uint32_t)new_idx;

    if (new_idx == 0) {
        set_command(wallter::CMD_RETRACT);
    }

    g_ctx.display->trigger_refresh();
}

void update_limits(int min_target_idx, int max_target_idx, uint32_t home_offset_raw_ticks) {
    if (min_target_idx < 0) min_target_idx = 0;
    if (max_target_idx < 0) max_target_idx = 0;
    if (max_target_idx >= g_ctx.max_angles) max_target_idx = g_ctx.max_angles - 1;
    if (min_target_idx > max_target_idx) {
        min_target_idx = 0;
        max_target_idx = g_ctx.max_angles - 1;
    }

    g_ctx.min_target_idx = min_target_idx;
    g_ctx.max_target_idx = max_target_idx;
    (void)home_offset_raw_ticks;
    g_ctx.home_offset_raw_ticks = 0;

    if ((int)*g_ctx.target_idx < g_ctx.min_target_idx) *g_ctx.target_idx = (uint32_t)g_ctx.min_target_idx;
    if ((int)*g_ctx.target_idx > g_ctx.max_target_idx) *g_ctx.target_idx = (uint32_t)g_ctx.max_target_idx;
}

} // namespace wallter::control
