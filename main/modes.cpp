#include "modes.hpp"

#include "app_config.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "commands.hpp"

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

    svc.display->set_refresh_rate(0.2f);

    auto render = [&]() {
        // Show the selected item on line 1, and the next item on line 2.
        // (Simple, readable on 16x2; RETRACT cycles; EXTEND selects.)
        if (sel == 0) {
            svc.display->print(">Calibrate", " Jog mode");
        } else if (sel == 1) {
            svc.display->print(">Run self test", " Calibrate");
        } else {
            svc.display->print(">Jog mode", " Run self test");
        }
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
            sel = (sel + 1) % 3;
            render();
        }

        // EXTEND selects the highlighted option.
        if (extend && !prev_extend) {
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                vTaskDelay(pdMS_TO_TICKS(30));
            }
            if (sel == 0) return MENU_CALIBRATE;
            if (sel == 1) return MENU_SELF_TEST;
            return MENU_JOG;
        }

        prev_extend = extend;
        prev_retract = retract;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void run_calibration_mode(Services &svc) {
    if (!svc.cal_meta || !svc.home_offset_raw_ticks) {
        svc.panicf("NO META");
    }

    auto clamp_meta = [&]() {
        int min_deg = (int)svc.cal_meta->min_angle_deg;
        int max_deg = (int)svc.cal_meta->max_angle_deg;
        if (min_deg < LOWEST_ANGLE) min_deg = LOWEST_ANGLE;
        if (max_deg > HIGHEST_ANGLE) max_deg = HIGHEST_ANGLE;
        if (((min_deg - LOWEST_ANGLE) % ANGLE_STEP) != 0) min_deg = LOWEST_ANGLE;
        if (((max_deg - LOWEST_ANGLE) % ANGLE_STEP) != 0) max_deg = HIGHEST_ANGLE;
        if (min_deg > max_deg) max_deg = min_deg;
        svc.cal_meta->min_angle_deg = (uint8_t)min_deg;
        svc.cal_meta->max_angle_deg = (uint8_t)max_deg;
    };
    clamp_meta();

    // Start by homing. With a configured min-angle, CMD_HOME will end at that min.
    svc.run_homing_blocking(/*timeout_ms=*/45000);

    enum class CalAction {
        STORE_HAL = 0,
        SET_MIN = 1,
        SET_MAX = 2,
        SKIP = 3,
    };

    auto action_label = [&](CalAction a) -> const char * {
        switch (a) {
            case CalAction::STORE_HAL: return ">Store HAL";
            case CalAction::SET_MIN:   return ">Set Min";
            case CalAction::SET_MAX:   return ">Set Max";
            case CalAction::SKIP:      return ">Skip";
        }
        return ">?";
    };

    auto run_action_menu = [&](int angle, int configured_min_angle, CalAction initial_sel) -> CalAction {
        CalAction sel = initial_sel;
        bool prev_extend = false;
        bool prev_retract = false;

        // Wait for release so we don't instantly select due to the BOTH-press that opened the menu.
        while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(30));
        }

        while (1) {
            // Render
            const char deg = (char)0xDF;
            char line1[17];
            char line2[17];
            char mark = (angle == configured_min_angle) ? '*' : ' ';
            int32_t pos = svc.motors[svc.master_motor_index].getPosition();
            if (pos < 0) pos = 0;
            if (pos > 99999) pos = 99999;
            uint32_t shown_raw = *svc.home_offset_raw_ticks + (uint32_t)pos;
            if (shown_raw > 99999U) shown_raw = 99999U;
            snprintf(line1, sizeof(line1), "%2d%c%c H%5lu", angle, deg, mark, (unsigned long)shown_raw);
            snprintf(line2, sizeof(line2), "%s", action_label(sel));
            svc.display->print(line1, line2);

            bool extend = svc.read_extend_pressed();
            bool retract = svc.read_retract_pressed();

            // RETRACT cycles
            if (retract && !prev_retract) {
                sel = (sel == CalAction::SKIP) ? CalAction::STORE_HAL : (CalAction)((int)sel + 1);
            }

            // EXTEND selects
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

    auto save_all_to_nvs = [&]() {
        // Convert shifted ticks back to raw ticks for storage.
        int min_idx = wallter::angle_to_index((int)svc.cal_meta->min_angle_deg);
        if (min_idx < 0) min_idx = 0;
        uint32_t offset = *svc.home_offset_raw_ticks;

        uint32_t raw[wallter::kMaxAngles] = {0};
        for (int i = 0; i < svc.max_angles; ++i) {
            if (i < min_idx) {
                raw[i] = 0;
            } else {
                raw[i] = svc.target_ticks[i] + offset;
            }
        }

        (void)wallter::calibration::save_from_target_ticks(raw, svc.max_angles);
        (void)wallter::calibration::save_meta(*svc.cal_meta);
    };

    int configured_min_angle = (int)svc.cal_meta->min_angle_deg;
    int configured_max_angle = (int)svc.cal_meta->max_angle_deg;
    (void)configured_max_angle;

    static constexpr int32_t kJogSpeed = MAXSPEED;

    for (int angle = LOWEST_ANGLE; angle <= HIGHEST_ANGLE; angle += ANGLE_STEP) {
        int idx = wallter::angle_to_index(angle);
        if (idx < 0) {
            svc.panicf("BAD ANG %d", angle);
        }
        *svc.target_idx = (uint32_t)idx;

        // Manual jog screen: hold EXTEND/RETRACT to move.
        // Press BOTH to open the store/min/max/skip menu.
        CalAction menu_default = CalAction::STORE_HAL;
        bool prev_both = false;

        int32_t last_shown_pos = INT32_MIN;
        uint64_t last_shown_ms = 0;

        // Per-angle loop: manual jog
        while (1) {
            bool extend = svc.read_extend_pressed();
            bool retract = svc.read_retract_pressed();
            bool both = extend && retract;

            // Manual jog by directly commanding motor speeds.
            // This avoids the control state machine's retract->home auto-transition and LCD view changes.
            int32_t spd = 0;
            if (!both) {
                if (extend) spd = kJogSpeed;
                if (retract) spd = -kJogSpeed;
            }
            for (int m = 0; m < svc.num_motors; ++m) {
                svc.motors[m].setSpeed(spd);
            }

            // Display
            const char deg = (char)0xDF;
            char line1[17];
            char line2[17];
            char mark = (angle == configured_min_angle) ? '*' : ' ';
            uint32_t offset_raw = *svc.home_offset_raw_ticks;

            int32_t pos = svc.motors[svc.master_motor_index].getPosition();
            if (pos < 0) pos = 0;
            if (pos > 99999) pos = 99999;
            uint32_t shown_raw = offset_raw + (uint32_t)pos;
            if (shown_raw > 99999U) shown_raw = 99999U;
            // Line 1: angle + HAL CNT
            snprintf(line1, sizeof(line1), "%2d%c%c H%5lu", angle, deg, mark, (unsigned long)shown_raw);
            // Line 2: instructions
            snprintf(line2, sizeof(line2), "Jog; BOTH=Menu");

            uint64_t now_ms = svc.now_ms();
            int32_t coarse_pos = (pos / 50) * 50;
            bool pos_changed = (last_shown_pos == INT32_MIN) || (coarse_pos != last_shown_pos);
            bool time_changed = (now_ms - last_shown_ms) >= 350ULL;
            if (pos_changed || time_changed) {
                svc.display->print(line1, line2);
                last_shown_pos = coarse_pos;
                last_shown_ms = now_ms;
            }

            if (both && !prev_both) {
                // Enter the action menu.
                for (int m = 0; m < svc.num_motors; ++m) {
                    svc.motors[m].setSpeed(0);
                }

                CalAction action = run_action_menu(angle, configured_min_angle, menu_default);
                menu_default = action;

                // Snapshot position after menu interaction
                int32_t pos_now = svc.motors[svc.master_motor_index].getPosition();
                if (pos_now < 0) pos_now = 0;

                if (action == CalAction::SKIP) {
                    svc.display->print("Skipped", "");
                    uint64_t end = svc.now_ms() + 350ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));

                    // Ensure motors are stopped when leaving this step.
                    for (int m = 0; m < svc.num_motors; ++m) {
                        svc.motors[m].setSpeed(0);
                    }
                    break;
                }

                if (action == CalAction::STORE_HAL) {
                    svc.target_ticks[idx] = (uint32_t)pos_now;
                    save_all_to_nvs();
                    svc.display->print("Stored", "");
                    uint64_t end = svc.now_ms() + 450ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));

                    for (int m = 0; m < svc.num_motors; ++m) {
                        svc.motors[m].setSpeed(0);
                    }
                    break;
                }

                if (action == CalAction::SET_MAX) {
                    svc.cal_meta->max_angle_deg = (uint8_t)angle;
                    clamp_meta();
                    configured_max_angle = (int)svc.cal_meta->max_angle_deg;
                    int min_idx2 = wallter::angle_to_index((int)svc.cal_meta->min_angle_deg);
                    int max_idx2 = wallter::angle_to_index((int)svc.cal_meta->max_angle_deg);
                    if (min_idx2 < 0) min_idx2 = 0;
                    if (max_idx2 < 0) max_idx2 = svc.max_angles - 1;
                    wallter::control::update_limits(min_idx2, max_idx2, *svc.home_offset_raw_ticks);
                    save_all_to_nvs();
                    svc.display->print("Max set", "Stored");
                    uint64_t end = svc.now_ms() + 600ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));
                    // End calibration when max is set.
                    goto calibration_done;
                }

                if (action == CalAction::SET_MIN) {
                    // Recompute offsets/ticks when redefining min.
                    int old_min_idx = wallter::angle_to_index((int)svc.cal_meta->min_angle_deg);
                    if (old_min_idx < 0) old_min_idx = 0;
                    uint32_t old_offset = *svc.home_offset_raw_ticks;

                    uint32_t raw[wallter::kMaxAngles] = {0};
                    for (int i = 0; i < svc.max_angles; ++i) {
                        raw[i] = (i < old_min_idx) ? 0U : (svc.target_ticks[i] + old_offset);
                    }

                    svc.cal_meta->min_angle_deg = (uint8_t)angle;
                    if ((int)svc.cal_meta->max_angle_deg < (int)svc.cal_meta->min_angle_deg) {
                        svc.cal_meta->max_angle_deg = svc.cal_meta->min_angle_deg;
                    }
                    clamp_meta();

                    int new_min_idx = wallter::angle_to_index((int)svc.cal_meta->min_angle_deg);
                    int new_max_idx = wallter::angle_to_index((int)svc.cal_meta->max_angle_deg);
                    if (new_min_idx < 0) new_min_idx = 0;
                    if (new_max_idx < 0) new_max_idx = svc.max_angles - 1;

                    uint32_t new_offset = old_offset + (uint32_t)pos_now;
                    raw[new_min_idx] = new_offset;
                    *svc.home_offset_raw_ticks = new_offset;

                    for (int i = 0; i < svc.max_angles; ++i) {
                        if (i < new_min_idx) {
                            svc.target_ticks[i] = 0;
                        } else {
                            uint32_t t = raw[i];
                            svc.target_ticks[i] = (t >= new_offset) ? (t - new_offset) : 0;
                        }
                    }

                    // Declare current mechanical position as logical home (0 ticks).
                    for (int m = 0; m < svc.num_motors; ++m) {
                        svc.motors[m].resetSteps(true);
                        svc.motors[m].setPosition(0);
                    }

                    *svc.target_idx = (uint32_t)new_min_idx;
                    wallter::control::update_limits(new_min_idx, new_max_idx, new_offset);
                    save_all_to_nvs();

                    configured_min_angle = (int)svc.cal_meta->min_angle_deg;
                    configured_max_angle = (int)svc.cal_meta->max_angle_deg;
                    (void)configured_max_angle;
                    svc.display->print("Min set", "Stored");
                    uint64_t end = svc.now_ms() + 650ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));
                    break;
                }
            }

            prev_both = both;
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }

calibration_done:
    svc.display->print("Cal complete", "Homing...");
    svc.run_homing_blocking(/*timeout_ms=*/45000);
}

void run_self_test_sequence(Services &svc) {
    const int test_speed = MINSPEED;
    const int test_time  = 1000;

    ESP_LOGI(TAG, "Self-test started.");
    ESP_LOGI(TAG, "ST: init pos=[%ld %ld] si=[%lu %lu] so=[%lu %lu]",
             (long)svc.motors[0].getPosition(),
             (long)svc.motors[1].getPosition(),
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.display->print("Self test 1", "Extending.");

    // Retract then extend
    ESP_LOGI(TAG, "ST: retract phase start, speed=%d", test_speed);
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(-test_speed);
    ESP_LOGI(TAG, "ST: retract phase running...");
    vTaskDelay(pdMS_TO_TICKS(test_time + 500));

    ESP_LOGI(TAG, "ST: retract done. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    svc.reset_tick_counters(0, true);
    ESP_LOGI(TAG, "ST: counters reset. si=[%lu %lu] so=[%lu %lu]",
             (unsigned long)svc.motors[0].getStepsIn(),
             (unsigned long)svc.motors[1].getStepsIn(),
             (unsigned long)svc.motors[0].getStepsOut(),
             (unsigned long)svc.motors[1].getStepsOut());

    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(test_speed);
    ESP_LOGI(TAG, "ST: extend phase start, speed=%d", test_speed);
    vTaskDelay(pdMS_TO_TICKS(test_time));

    svc.display->print("Phase 1 done", "Checking...");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(0);

    for (int i = 0; i < svc.num_motors; i++) {
        if (svc.motors[i].getStepsOut() < 100U) {
            ESP_LOGE(TAG, "Self-test failed: motor %d si:%d so:%d",
                     i,
                     (int)svc.motors[i].getStepsIn(),
                     (int)svc.motors[i].getStepsOut());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SO:%d", i, (int)svc.motors[i].getStepsOut());
            svc.display->print("SELF TEST FAILED", tbuf);
            svc.panicf("M%d NO OUT", i);
        }
    }

    svc.display->print("Self test 2", "Retracting.");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(-test_speed);
    vTaskDelay(pdMS_TO_TICKS(test_time));

    svc.display->print("Phase 2 done", "Checking...");
    for (int i = 0; i < svc.num_motors; i++) svc.motors[i].setSpeed(0);

    for (int i = 0; i < svc.num_motors; i++) {
        if (svc.motors[i].getStepsIn() < 100U) {
            ESP_LOGE(TAG, "Self-test failed: motor %d si:%d so:%d",
                     i,
                     (int)svc.motors[i].getStepsIn(),
                     (int)svc.motors[i].getStepsOut());
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "M:%d SI:%d", i, (int)svc.motors[i].getStepsIn());
            svc.display->print("SELF TEST FAILED:", tbuf);
            svc.panicf("M%d NO IN", i);
        }
    }

    bool final_success = true;
    for (int i = 0; i < svc.num_motors; ++i) {
        if (abs((int)svc.motors[i].getStepsOut() - (int)svc.motors[i].getStepsIn()) > 200) {
            final_success = false;
        }
    }

    if (!final_success) {
        svc.display->print("FAIL: BAD SENSOR DATA", "Check connections.");
        svc.panicf("CHECK CONNECTIONS");
    }

    svc.display->print("Self test", "PASSED");
    uint64_t end_ms = svc.now_ms() + 1500ULL;
    while (svc.now_ms() < end_ms) {
        svc.display->refresh();
        vTaskDelay(pdMS_TO_TICKS(50));
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

} // namespace wallter::modes
