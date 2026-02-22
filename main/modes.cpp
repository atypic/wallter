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
    bool prev_up = false;
    bool prev_dn = false;
    bool prev_both = false;

    svc.display->set_refresh_rate(0.2f);

    auto render = [&]() {
        if (sel == 0) {
            svc.display->print(">Calibrate", " Run self test");
        } else {
            svc.display->print(" Calibrate", ">Run self test");
        }
    };
    render();

    // Wait for initial release to avoid auto-activating due to boot hold
    while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    while (1) {
        bool up = svc.read_extend_pressed();
        bool dn = svc.read_retract_pressed();
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
            while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
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

    static constexpr uint32_t kAdjustStepTicks = 50;

    enum class CalAction {
        STORE = 0,
        DISCARD = 1,
        SET_MIN = 2,
        SET_MAX = 3,
    };

    auto run_submenu = [&]() -> CalAction {
        CalAction sel = CalAction::STORE;
        bool prev_up = false;
        bool prev_dn = false;
        bool prev_both = false;

        auto render = [&](CalAction a) {
            char line1[17];
            // Example: "Act m20 M60" (<= 11 chars)
            snprintf(line1,
                     sizeof(line1),
                     "Act m%u M%u",
                     (unsigned)svc.cal_meta->min_angle_deg,
                     (unsigned)svc.cal_meta->max_angle_deg);
            switch (a) {
                case CalAction::STORE:
                    svc.display->print(line1, ">Store");
                    break;
                case CalAction::DISCARD:
                    svc.display->print(line1, ">Discard");
                    break;
                case CalAction::SET_MIN:
                    svc.display->print(line1, ">Set Min");
                    break;
                case CalAction::SET_MAX:
                    svc.display->print(line1, ">Set Max");
                    break;
            }
        };

        render(sel);

        while (1) {
            bool up = svc.read_extend_pressed();
            bool dn = svc.read_retract_pressed();
            bool both = up && dn;

            if (!both) {
                if (up && !prev_up) {
                    sel = (sel == CalAction::SET_MAX) ? CalAction::STORE : (CalAction)((int)sel + 1);
                    render(sel);
                }
                if (dn && !prev_dn) {
                    sel = (sel == CalAction::STORE) ? CalAction::SET_MAX : (CalAction)((int)sel - 1);
                    render(sel);
                }
            }

            if (both && !prev_both) {
                // Confirm selection; wait for release.
                while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                return sel;
            }

            prev_up = up;
            prev_dn = dn;
            prev_both = both;
            vTaskDelay(pdMS_TO_TICKS(40));
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

    bool prev_up = false;
    bool prev_dn = false;
    bool prev_both = false;

    uint32_t last_ui_ticks = 0xFFFFFFFFu;
    int last_ui_angle = -1;
    uint64_t last_ui_ms = 0;

    int min_angle = (int)svc.cal_meta->min_angle_deg;
    int max_angle = (int)svc.cal_meta->max_angle_deg;

    for (int angle = min_angle; angle <= max_angle; angle += ANGLE_STEP) {
        int idx = wallter::angle_to_index(angle);
        if (idx < 0) {
            svc.panicf("BAD ANG %d", angle);
        }

        *svc.target_idx = (uint32_t)idx;

        const uint32_t original_ticks = svc.target_ticks[idx];

        // Ensure we are stopped before starting adjustments for this angle
        svc.set_current_command(wallter::CMD_STOP);

        while (1) {
            // Drive motors toward current target ticks for this angle
            int32_t pos = svc.motors[svc.master_motor_index].getPosition();
            uint32_t tgt = svc.target_ticks[*svc.target_idx];
            if (pos < (int32_t)tgt) {
                svc.set_current_command(wallter::CMD_EXTEND);
            } else if (pos > (int32_t)tgt) {
                svc.set_current_command(wallter::CMD_RETRACT);
            } else {
                svc.set_current_command(wallter::CMD_STOP);
            }

            svc.motor_control_iteration();

            // UI: "20: <ticks>" (mark min angle)
            char line1[17];
            char line2[17];
            snprintf(line2,
                     sizeof(line2),
                     "m%u M%u",
                     (unsigned)svc.cal_meta->min_angle_deg,
                     (unsigned)svc.cal_meta->max_angle_deg);
            if (angle == (int)svc.cal_meta->min_angle_deg) {
                snprintf(line1, sizeof(line1), "%dm:%lu", angle, (unsigned long)svc.target_ticks[*svc.target_idx]);
            } else {
                snprintf(line1, sizeof(line1), "%d:%lu", angle, (unsigned long)svc.target_ticks[*svc.target_idx]);
            }

            // Avoid hammering I2C: only update when changed or periodically.
            uint64_t tnow = svc.now_ms();
            uint32_t cur_ticks = svc.target_ticks[*svc.target_idx];
            if (angle != last_ui_angle || cur_ticks != last_ui_ticks || (tnow - last_ui_ms) > 250ULL) {
                svc.display->print(line1, line2);
                last_ui_angle = angle;
                last_ui_ticks = cur_ticks;
                last_ui_ms = tnow;
            }

            bool up = svc.read_extend_pressed();
            bool dn = svc.read_retract_pressed();
            bool both = up && dn;

            if (both && !prev_both) {
                // Pause motion and open submenu.
                svc.set_current_command(wallter::CMD_STOP);
                svc.motor_control_iteration();
                vTaskDelay(pdMS_TO_TICKS(80));

                CalAction action = run_submenu();

                if (action == CalAction::DISCARD) {
                    svc.target_ticks[idx] = original_ticks;
                    svc.display->print("discarded", "");
                    uint64_t end = svc.now_ms() + 500ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));
                    break; // next angle
                }

                if (action == CalAction::SET_MAX) {
                    svc.cal_meta->max_angle_deg = (uint8_t)angle;
                    clamp_meta();
                    // Update control limits immediately.
                    int min_idx2 = wallter::angle_to_index((int)svc.cal_meta->min_angle_deg);
                    int max_idx2 = wallter::angle_to_index((int)svc.cal_meta->max_angle_deg);
                    if (min_idx2 < 0) min_idx2 = 0;
                    if (max_idx2 < 0) max_idx2 = svc.max_angles - 1;
                    wallter::control::update_limits(min_idx2, max_idx2, *svc.home_offset_raw_ticks);
                    save_all_to_nvs();
                    svc.display->print("max set", "stored");
                    uint64_t end = svc.now_ms() + 650ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));
                    // End calibration early at max.
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

                    uint32_t new_offset = raw[new_min_idx];
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
                    svc.display->print("min set", "stored");
                    uint64_t end = svc.now_ms() + 650ULL;
                    while (svc.now_ms() < end) vTaskDelay(pdMS_TO_TICKS(30));
                    // After changing min, restart loop bounds.
                    min_angle = (int)svc.cal_meta->min_angle_deg;
                    max_angle = (int)svc.cal_meta->max_angle_deg;
                    angle = min_angle - ANGLE_STEP;
                    break;
                }

                // STORE (default)
                save_all_to_nvs();
                svc.display->print("stored", "");
                uint64_t end = svc.now_ms() + 600ULL;
                while (svc.now_ms() < end) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }
                break; // next angle
            }

            if (!both) {
                if (up && !prev_up) {
                    svc.target_ticks[*svc.target_idx] += kAdjustStepTicks;
                }
                if (dn && !prev_dn) {
                    if (svc.target_ticks[*svc.target_idx] >= kAdjustStepTicks) {
                        svc.target_ticks[*svc.target_idx] -= kAdjustStepTicks;
                    } else {
                        svc.target_ticks[*svc.target_idx] = 0;
                    }
                }
            }

            prev_up = up;
            prev_dn = dn;
            prev_both = both;
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    }

calibration_done:

    // Derive 15 degrees entry again, and return to home.
    {
        int idx20 = wallter::angle_to_index(20);
        int idx15 = wallter::angle_to_index(15);
        if (idx20 >= 0 && idx15 >= 0) {
            svc.target_ticks[idx15] = svc.target_ticks[idx20] / 2U;
        }
        if (svc.max_angles > 0) {
            svc.target_ticks[0] = 0;
        }
    }

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

} // namespace wallter::modes
