#include "modes.hpp"

#include "app_config.hpp"
#include "accel.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "commands.hpp"

#include "inputs.hpp"

#include "motor_control.hpp"

#include "display.hpp"
#include "motordriver.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <stdio.h>
#include <stdlib.h>

namespace wallter::modes {

static const char *TAG = "modes";

BootMenuChoice run_boot_menu(Services &svc) {
    int sel = 0;
    bool prev_extend = false;
    bool prev_retract = false;

    static const char *items[] = {"Set max angle", "Set min angle", "Angle offset", "Run self test", "Jog mode", "HAL test", "Reset cal", "Test accel", "Cal accel"};
    static constexpr int kNumItems = (int)(sizeof(items) / sizeof(items[0]));

    svc.display->set_refresh_rate(0.2f);

    auto render = [&]() {
        // Show the selected item on line 1, and the next item on line 2.
        // (Simple, readable on 16x2; RETRACT cycles; EXTEND selects.)
        int s = sel;
        if (s < 0) s = 0;
        if (s >= kNumItems) s = kNumItems - 1;

        char line1[17];
        char line2[17];
        snprintf(line1, sizeof(line1), ">%s", items[s]);
        snprintf(line2, sizeof(line2), " %s", items[(s + 1) % kNumItems]);
        svc.display->print(line1, line2);
    };
    render();

    // Wait for initial release to avoid auto-activating due to boot hold
    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    while (1) {
        bool extend = svc.read_extend_pressed();
        bool retract = svc.read_retract_pressed();

        // RETRACT cycles the highlighted option.
        if (retract && !prev_retract) {
            sel = (sel + 1) % kNumItems;
            render();
        }

        // EXTEND selects the highlighted option.
        if (extend && !prev_extend) {
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            if (sel == 0) return MENU_SET_MAX_ANGLE;
            if (sel == 1) return MENU_SET_MIN_ANGLE;
            if (sel == 2) return MENU_SET_OFFSET;
            if (sel == 3) return MENU_SELF_TEST;
            if (sel == 4) return MENU_JOG;
            if (sel == 5) return MENU_HAL_TEST;
            if (sel == 6) return MENU_RESET_CAL;
            if (sel == 7) return MENU_TEST_ACCEL;
            return MENU_CAL_ACCEL;
        }

        prev_extend = extend;
        prev_retract = retract;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void run_reset_calibration_data(Services &svc) {
    svc.display->set_refresh_rate(0.5f);
    svc.display->print("Reset cal", "Please wait");
    svc.display->trigger_refresh();

    // Stop motors just in case.
    for (int i = 0; i < svc.num_motors; ++i) {
        svc.motors[i].setSpeed(0);
    }

    esp_err_t err = wallter::calibration::erase_calibration();
    if (err == ESP_OK) {
        svc.display->print("Reset cal", "Done");
    } else {
        svc.display->print("Reset cal", "FAILED");
    }

    uint64_t end_ms = svc.now_ms() + 1500ULL;
    while (svc.now_ms() < end_ms) {
        svc.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_set_max_angle_mode(Services &svc) {
    if (!svc.cal_meta) {
        svc.panicf("NO META");
    }

    // Snap the stored max angle to the nearest valid step; default to HIGHEST_ANGLE.
    // Valid values are any ANGLE_STEP multiple in [ANGLE_STEP, HIGHEST_ANGLE].
    int cur_max = (int)svc.cal_meta->max_angle_deg;
    if (cur_max < ANGLE_STEP || cur_max > HIGHEST_ANGLE || (cur_max % ANGLE_STEP) != 0) {
        cur_max = DEFAULT_CAL_MAX_ANGLE;
    }

    bool prev_extend = false;
    bool prev_retract = false;

    // Drain any held buttons before entering the menu.
    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    svc.display->set_refresh_rate(0.2f);

    const char kDeg = (char)0xDF;
    auto render = [&]() {
        char line1[17];
        snprintf(line1, sizeof(line1), "Set max: %d%c", cur_max, kDeg);
        svc.display->print(line1, "R=Next  E=Save");
    };
    render();

    while (1) {
        bool extend = svc.read_extend_pressed();
        bool retract = svc.read_retract_pressed();

        // RETRACT cycles from ANGLE_STEP up to HIGHEST_ANGLE, then wraps.
        if (retract && !prev_retract) {
            cur_max += ANGLE_STEP;
            if (cur_max > HIGHEST_ANGLE) cur_max = ANGLE_STEP;
            render();
        }

        // EXTEND confirms and saves.
        if (extend && !prev_extend) {
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }

            svc.cal_meta->max_angle_deg = (uint8_t)cur_max;
            (void)wallter::calibration::save_meta(*svc.cal_meta);

            char line1[17];
            snprintf(line1, sizeof(line1), "Max: %d%c saved", cur_max, kDeg);
            svc.display->print(line1, "");
            uint64_t end = svc.now_ms() + 800ULL;
            while (svc.now_ms() < end) {
                svc.display->refresh();
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            return;
        }

        prev_extend = extend;
        prev_retract = retract;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void run_set_min_angle_mode(Services &svc) {
    if (!svc.cal_meta) {
        svc.panicf("NO META");
    }

    // Snap the stored min angle to the nearest valid step; default to LOWEST_ANGLE.
    int cur_min = (int)svc.cal_meta->min_angle_deg;
    if (cur_min < ANGLE_STEP || cur_min > HIGHEST_ANGLE || (cur_min % ANGLE_STEP) != 0) {
        cur_min = DEFAULT_CAL_MIN_ANGLE;
    }

    bool prev_extend = false;
    bool prev_retract = false;

    // Drain any held buttons before entering the menu.
    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    svc.display->set_refresh_rate(0.2f);

    const char kDeg = (char)0xDF;
    auto render = [&]() {
        char line1[17];
        snprintf(line1, sizeof(line1), "Set min: %d%c", cur_min, kDeg);
        svc.display->print(line1, "R=Next  E=Save");
    };
    render();

    while (1) {
        bool extend = svc.read_extend_pressed();
        bool retract = svc.read_retract_pressed();

        // RETRACT cycles from ANGLE_STEP up to HIGHEST_ANGLE, then wraps.
        if (retract && !prev_retract) {
            cur_min += ANGLE_STEP;
            if (cur_min > HIGHEST_ANGLE) cur_min = ANGLE_STEP;
            render();
        }

        // EXTEND confirms and saves.
        if (extend && !prev_extend) {
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }

            svc.cal_meta->min_angle_deg = (uint8_t)cur_min;
            (void)wallter::calibration::save_meta(*svc.cal_meta);

            char line1[17];
            snprintf(line1, sizeof(line1), "Min: %d%c saved", cur_min, kDeg);
            svc.display->print(line1, "");
            uint64_t end = svc.now_ms() + 800ULL;
            while (svc.now_ms() < end) {
                svc.display->refresh();
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            return;
        }

        prev_extend = extend;
        prev_retract = retract;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void run_set_angle_offset_mode(Services &svc) {
    if (!svc.cal_meta) {
        svc.panicf("NO META");
    }

    // Current offset in tenths of a degree; range -127..+127 (±12.7°).
    int cur = (int)svc.cal_meta->angle_offset_tenths;

    bool prev_extend = false;
    bool prev_retract = false;

    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    svc.display->set_refresh_rate(0.2f);

    const char kDeg = (char)0xDF;
    auto render = [&]() {
        char line1[17];
        // Show as e.g. "+1.8°" or "-0.5°"
        char tmp[24];
        int whole = cur / 10;
        int frac  = (cur < 0 ? -cur : cur) % 10;
        snprintf(tmp, sizeof(tmp), "Ofs:%+d.%d%c", whole, frac, kDeg);
        tmp[16] = '\0';
        strncpy(line1, tmp, sizeof(line1));
        line1[16] = '\0';
        svc.display->print(line1, "R=+0.1  E=Save");
    };
    render();

    while (1) {
        bool extend = svc.read_extend_pressed();
        bool retract = svc.read_retract_pressed();

        // RETRACT increments by 0.1°, wrapping from +12.7 back to -12.7.
        if (retract && !prev_retract) {
            cur++;
            if (cur > 127) cur = -127;
            render();
        }

        // EXTEND confirms and saves.
        if (extend && !prev_extend) {
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }

            svc.cal_meta->angle_offset_tenths = (int8_t)cur;
            (void)wallter::calibration::save_meta(*svc.cal_meta);

            char tmp[24];
            char line1[17];
            int whole = cur / 10;
            int frac  = (cur < 0 ? -cur : cur) % 10;
            snprintf(tmp, sizeof(tmp), "%+d.%d%c ok", whole, frac, kDeg);
            tmp[16] = '\0';
            strncpy(line1, tmp, sizeof(line1));
            line1[16] = '\0';
            svc.display->print(line1, "");
            uint64_t end = svc.now_ms() + 800ULL;
            while (svc.now_ms() < end) {
                svc.display->refresh();
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            return;
        }

        prev_extend = extend;
        prev_retract = retract;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void run_self_test_sequence(Services &svc) {
    const int test_speed = MINSPEED;
    const int test_time  = 1000;
    const int settle_time = 250;
    static constexpr uint32_t kSelfTestExpectedTicks = 100U;
    static constexpr uint32_t kSelfTestAllowedMargin = 20U;
    static constexpr uint32_t kSelfTestMinTicks = kSelfTestExpectedTicks - kSelfTestAllowedMargin;

    auto wait_with_encoder_poll = [&](uint32_t total_ms) {
        // Self-test drives motors directly (no control loop), so we must poll PCNT
        // to transfer encoder counts into MotorDriver counters.
        static constexpr uint32_t kChunkMs = 25;
        uint32_t remaining = total_ms;
        while (remaining > 0) {
            uint32_t d = (remaining > kChunkMs) ? kChunkMs : remaining;
            vTaskDelay(pdMS_TO_TICKS(d));
            wallter::inputs::poll_encoder_counts();
            remaining -= d;
        }
    };

    ESP_LOGI(TAG, "Self-test started.");
    ESP_LOGI(TAG, "Init pos=[%ld %ld] si=[%lu %lu] so=[%lu %lu]",
             (long)svc.motors[0].getPosition(),
             (long)svc.motors[1].getPosition(),
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.display->print("Self test 1", "Extending.");

    // Retract then extend
    ESP_LOGI(TAG, "Retract phase start, speed=%d", test_speed);
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(-test_speed);
    ESP_LOGI(TAG, "Retract phase running...");
    wait_with_encoder_poll((uint32_t)test_time + 500U);

    ESP_LOGI(TAG, "Retract done. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    // Stop + settle before resetting counters and reversing direction.
    // With HAL_VALIDATE_DIR_WITH_MOTOR enabled, immediate reversals can drop
    // ticks while the mechanism is still moving in the previous direction.
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(0);
    wait_with_encoder_poll((uint32_t)settle_time);

    svc.reset_tick_counters(0, true);
    ESP_LOGI(TAG, "Counters reset. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(test_speed);
    ESP_LOGI(TAG, "Extend phase start, speed=%d", test_speed);
    wait_with_encoder_poll((uint32_t)test_time);

    ESP_LOGI(TAG, "Extend done. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.display->print("Phase 1 done", "Checking...");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(0);

    for (int i = 0; i < svc.num_motors; i++) {
        uint32_t so = svc.motors[i].getStepsOut();
        if (so < kSelfTestMinTicks) {
            ESP_LOGE(TAG, "Self-test failed: motor %d so=%lu (min=%lu) si=%lu",
                     i,
                     (unsigned long)so,
                     (unsigned long)kSelfTestMinTicks,
                     (unsigned long)svc.motors[i].getStepsIn());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SO:%lu", i, (unsigned long)so);
            svc.display->print("SELF TEST FAILED", tbuf);
            svc.panicf("M%d NO OUT", i);
        }
    }

    // Stop + settle before resetting counters and reversing direction.
    // This matches the first phase behavior and keeps per-phase tick checks isolated.
    wait_with_encoder_poll((uint32_t)settle_time);
    svc.reset_tick_counters(0, true);
    ESP_LOGI(TAG, "Counters reset (before retract2). si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.display->print("Self test 2", "Retracting.");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(-test_speed);
    wait_with_encoder_poll((uint32_t)test_time);

    ESP_LOGI(TAG, "Retract2 done. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.display->print("Phase 2 done", "Checking...");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(0);

    for (int i = 0; i < svc.num_motors; i++) {
        uint32_t si = svc.motors[i].getStepsIn();
        if (si < kSelfTestMinTicks) {
            ESP_LOGE(TAG, "Self-test failed: motor %d si=%lu (min=%lu) so=%lu",
                     i,
                     (unsigned long)si,
                     (unsigned long)kSelfTestMinTicks,
                     (unsigned long)svc.motors[i].getStepsOut());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SI:%lu", i, (unsigned long)si);
            svc.display->print("SELF TEST FAILED:", tbuf);
            svc.panicf("M%d NO IN", i);
        }
    }

    // Note: historically we also compared extend vs retract tick counts.
    // In practice that can false-trigger due to load, stiction, and end-stop effects.
    // The per-direction minimum tick checks above are the reliable "is feedback present" test.
    for (int i = 0; i < svc.num_motors; ++i) {
        uint32_t so = svc.motors[i].getStepsOut();
        uint32_t si = svc.motors[i].getStepsIn();
        // Only warn if we're accumulating a meaningful amount of ticks in BOTH directions
        // within the same phase window. In a pure extend phase, si should be ~0; in a pure
        // retract phase, so should be ~0.
        uint32_t mn = (so < si) ? so : si;
        uint32_t mx = (so > si) ? so : si;
        uint32_t diff = mx - mn;
        if (mn >= (kSelfTestMinTicks / 2U) && diff * 100U > mx * 60U) {
            ESP_LOGW(TAG, "Self-test warning: motor %d tick mismatch si=%lu so=%lu",
                     i,
                     (unsigned long)si,
                     (unsigned long)so);
            svc.display->show_short_message(const_cast<char *>("WARN:"),
                                            const_cast<char *>("TICK MISMATCH"),
                                            1500);
        }
    }

    svc.display->print("Self test", "PASSED");
    uint64_t end_ms = svc.now_ms() + 1500ULL;
    while (svc.now_ms() < end_ms) {
        svc.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_hal_feedback_test(Services &svc) {
    static constexpr int kTestSpeed = MINSPEED;
    static constexpr uint32_t kRunMs = 900;
    static constexpr uint32_t kSettleMs = 200;
    static constexpr uint32_t kMinTicks = 60;

    auto stop_all = [&]() {
        for (int i = 0; i < svc.num_motors; ++i) {
            svc.motors[i].setSpeed(0);
        }
    };

    auto wait_release = [&]() {
        while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    };

    auto ticks_total = [&](int motor_idx) -> uint32_t {
        return svc.motors[motor_idx].getStepsIn() + svc.motors[motor_idx].getStepsOut();
    };

    struct MotorResult {
        uint32_t extend_ticks{};
        uint32_t retract_ticks{};
        bool extend_ok{};
        bool retract_ok{};
    };

    svc.display->set_refresh_rate(0.5f);
    svc.display->print("HAL test", "Running...");
    svc.display->trigger_refresh();

    stop_all();
    wait_release();

    MotorResult results[NUM_MOTORS] = {};

    int motors_to_test = svc.num_motors;
    if (motors_to_test > NUM_MOTORS) motors_to_test = NUM_MOTORS;

    for (int m = 0; m < motors_to_test; ++m) {
        // Extend phase
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(kSettleMs));
        svc.reset_tick_counters(0, true);
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(30));

        svc.display->print("HAL test", (m == 0) ? "M0 EXTEND" : (m == 1) ? "M1 EXTEND" : "EXTEND");
        svc.motors[m].setSpeed(+kTestSpeed);
        vTaskDelay(pdMS_TO_TICKS(kRunMs));
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(kSettleMs));

        results[m].extend_ticks = ticks_total(m);
        results[m].extend_ok = (results[m].extend_ticks >= kMinTicks);

        ESP_LOGI(TAG,
                 "HALT: M%d extend ticks=%lu (si=%lu so=%lu)",
                 m,
                 (unsigned long)results[m].extend_ticks,
                 (unsigned long)svc.motors[m].getStepsIn(),
                 (unsigned long)svc.motors[m].getStepsOut());

        // Retract phase
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(kSettleMs));
        svc.reset_tick_counters(0, true);
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(30));

        svc.display->print("HAL test", (m == 0) ? "M0 RETRACT" : (m == 1) ? "M1 RETRACT" : "RETRACT");
        svc.motors[m].setSpeed(-kTestSpeed);
        vTaskDelay(pdMS_TO_TICKS(kRunMs));
        stop_all();
        vTaskDelay(pdMS_TO_TICKS(kSettleMs));

        results[m].retract_ticks = ticks_total(m);
        results[m].retract_ok = (results[m].retract_ticks >= kMinTicks);

        ESP_LOGI(TAG,
                 "HALT: M%d retract ticks=%lu (si=%lu so=%lu)",
                 m,
                 (unsigned long)results[m].retract_ticks,
                 (unsigned long)svc.motors[m].getStepsIn(),
                 (unsigned long)svc.motors[m].getStepsOut());

        // Per-motor summary screen
        uint32_t e = results[m].extend_ticks;
        uint32_t r = results[m].retract_ticks;
        if (e > 9999U) e = 9999U;
        if (r > 9999U) r = 9999U;

        char line1[17];
        char line2[17];
        snprintf(line1, sizeof(line1), "M%d E%4lu R%4lu", m, (unsigned long)e, (unsigned long)r);
        if (results[m].extend_ok && results[m].retract_ok) {
            snprintf(line2, sizeof(line2), "OK");
        } else {
            // Give a quick hint.
            if (!results[m].extend_ok && !results[m].retract_ok) {
                snprintf(line2, sizeof(line2), "FAIL: NO TICKS");
            } else if (!results[m].extend_ok) {
                snprintf(line2, sizeof(line2), "FAIL: EXTEND");
            } else {
                snprintf(line2, sizeof(line2), "FAIL: RETRACT");
            }
        }
        svc.display->print(line1, line2);
        uint64_t end_ms = svc.now_ms() + 1400ULL;
        while (svc.now_ms() < end_ms) {
            svc.display->refresh();
            vTaskDelay(pdMS_TO_TICKS(40));
        }
    }

    // Final summary: show which motor(s) failed.
    bool all_ok = true;
    for (int m = 0; m < motors_to_test; ++m) {
        if (!(results[m].extend_ok && results[m].retract_ok)) {
            all_ok = false;
        }
    }

    if (motors_to_test >= 2) {
        char l1[17];
        char l2[17];
        snprintf(l1, sizeof(l1), "M0 %s  M1 %s",
                 (results[0].extend_ok && results[0].retract_ok) ? "OK" : "FAIL",
                 (results[1].extend_ok && results[1].retract_ok) ? "OK" : "FAIL");
        snprintf(l2, sizeof(l2), "%s", all_ok ? "HAL OK" : "Check wiring");
        svc.display->print(l1, l2);
    } else if (motors_to_test == 1) {
        svc.display->print(all_ok ? "M0 OK" : "M0 FAIL", all_ok ? "HAL OK" : "Check wiring");
    } else {
        svc.display->print("HAL test", "No motors");
    }

    // Wait for a button press so the user can read the result.
    while (!svc.read_extend_pressed() && !svc.read_retract_pressed()) {
        svc.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    wait_release();

    stop_all();
    svc.reset_tick_counters(0, true);
    for (int i = 0; i < svc.num_motors; ++i) {
        svc.motors[i].setPosition(0);
    }
}

void run_jog_mode(Services &svc) {
    // Pure jog mode: directly command motors by holding EXTEND/RETRACT.
    // BOTH opens a submenu to choose which motor(s) to jog.
    // NOTE: This mode intentionally does not use the control state machine.

    enum class JogSel {
        BOTH = 0,
        M1 = 1,
        M2 = 2,
    };

    auto sel_label = [&](JogSel s) -> const char * {
        switch (s) {
            case JogSel::BOTH: return "BOTH";
            case JogSel::M1:   return "M1";
            case JogSel::M2:   return "M2";
        }
        return "?";
    };

    auto run_sel_menu = [&]() -> JogSel {
        JogSel sel = JogSel::BOTH;
        bool prev_extend = false;
        bool prev_retract = false;

        // Wait for release to avoid accidental activation.
        while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        while (1) {
            wallter::inputs::poll_encoder_counts();

            char l1[17];
            snprintf(l1, sizeof(l1), ">Jog %s", sel_label(sel));
            svc.display->print(l1, "R=Next E=OK");

            bool extend = svc.read_extend_pressed();
            bool retract = svc.read_retract_pressed();

            if (retract && !prev_retract) {
                sel = (sel == JogSel::M2) ? JogSel::BOTH : (JogSel)((int)sel + 1);
            }
            if (extend && !prev_extend) {
                while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                return sel;
            }

            prev_extend = extend;
            prev_retract = retract;
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    };

    JogSel sel = JogSel::BOTH;
    sel = run_sel_menu();

    static constexpr int32_t kJogSpeed = MAXSPEED;
    bool prev_both = false;
    int32_t last_shown_p1 = INT32_MIN;
    int32_t last_shown_p2 = INT32_MIN;
    uint64_t last_shown_ms = 0;

    while (1) {
        // This mode drives motors directly (no control loop), so we must poll PCNT
        // to transfer encoder counts into MotorDriver counters.
        wallter::inputs::poll_encoder_counts();

        bool extend = svc.read_extend_pressed();
        bool retract = svc.read_retract_pressed();
        bool both = extend && retract;

        if (both && !prev_both) {
            // Stop before menu.
            for (int m = 0; m < svc.num_motors; ++m) {
                svc.motors[m].setSpeed(0);
            }
            sel = run_sel_menu();
        }

        int32_t spd = 0;
        if (!both) {
            if (extend) spd = kJogSpeed;
            if (retract) spd = -kJogSpeed;
        }

        // Apply to selected motors
        for (int m = 0; m < svc.num_motors; ++m) {
            bool apply = false;
            if (sel == JogSel::BOTH) {
                apply = true;
            } else if (sel == JogSel::M1) {
                apply = (m == 0);
            } else if (sel == JogSel::M2) {
                apply = (m == 1);
            }
            svc.motors[m].setSpeed(apply ? spd : 0);
        }

        // Display: show both motor positions (best-effort even if num_motors < 2)
        int32_t p1 = (svc.num_motors >= 1) ? svc.motors[0].getPosition() : 0;
        int32_t p2 = (svc.num_motors >= 2) ? svc.motors[1].getPosition() : 0;
        if (p1 < 0) p1 = 0;
        if (p2 < 0) p2 = 0;
        if (p1 > 99999) p1 = 99999;
        if (p2 > 99999) p2 = 99999;

        uint64_t now_ms = svc.now_ms();
        bool changed = (p1 != last_shown_p1) || (p2 != last_shown_p2) || ((now_ms - last_shown_ms) > 300ULL);
        if (changed) {
            char l1[17];
            char l2[17];
            // Example: "Jog BOTH" / "1:01234 2:05678"
            snprintf(l1, sizeof(l1), "Jog %-5.5s", sel_label(sel));
            snprintf(l2, sizeof(l2), "1:%5ld 2:%5ld", (long)p1, (long)p2);
            svc.display->print(l1, l2);
            last_shown_p1 = p1;
            last_shown_p2 = p2;
            last_shown_ms = now_ms;
        }

        prev_both = both;
        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

void run_test_accel_mode(Services &svc) {
    svc.display->set_refresh_rate(0.2f);
    svc.display->print("Test accel", "Reading...");

    // Press both buttons simultaneously to exit.
    bool prev_both = false;
    while (1) {
        wallter::accel::update(0.0f);
        float angle = wallter::accel::read_angle_deg();

        char line2[17];
        snprintf(line2, sizeof(line2), "Angle: %.1f", (double)angle);
        svc.display->print("Test accel", line2);
        svc.display->trigger_refresh();

        bool ext = svc.read_extend_pressed();
        bool ret = svc.read_retract_pressed();
        bool both = ext && ret;
        if (both && !prev_both) break;
        prev_both = both;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void run_cal_accel_mode(Services &svc) {
    if (!svc.cal_meta) {
        svc.panicf("NO META");
    }

    // Current offset in tenths of a degree.
    int ofs = (int)svc.cal_meta->angle_offset_tenths;

    svc.display->set_refresh_rate(0.2f);

    // Wait for buttons release from menu selection.
    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    bool prev_ext = false;
    bool prev_ret = false;

    while (1) {
        wallter::accel::update(0.0f);
        float angle = wallter::accel::read_angle_deg();

        char line1[17];
        char line2[17];
        snprintf(line1, sizeof(line1), "Ang:%.1f", (double)angle);
        float ofs_deg = (float)ofs * 0.1f;
        snprintf(line2, sizeof(line2), "O:%+.1f E+ R-", (double)ofs_deg);
        svc.display->print(line1, line2);
        svc.display->trigger_refresh();

        bool ext = svc.read_extend_pressed();
        bool ret = svc.read_retract_pressed();

        // Both pressed: save and exit.
        if (ext && ret) {
            svc.cal_meta->angle_offset_tenths = (int8_t)ofs;
            wallter::accel::set_angle_offset((float)ofs * 0.1f);
            (void)wallter::calibration::save_meta(*svc.cal_meta);

            svc.display->print("Offset saved", "");
            svc.display->trigger_refresh();
            uint64_t end = svc.now_ms() + 800ULL;
            while (svc.now_ms() < end) {
                svc.display->refresh();
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            return;
        }

        // EXTEND: increase offset by 0.1°
        if (ext && !prev_ext) {
            if (ofs < 127) ofs++;
            wallter::accel::set_angle_offset((float)ofs * 0.1f);
        }
        // RETRACT: decrease offset by 0.1°
        if (ret && !prev_ret) {
            if (ofs > -127) ofs--;
            wallter::accel::set_angle_offset((float)ofs * 0.1f);
        }

        prev_ext = ext;
        prev_ret = ret;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace wallter::modes
