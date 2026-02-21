#include "modes.hpp"

#include "app_config.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "commands.hpp"

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
    // Start by homing fully ("board all the way back")
    svc.run_homing_blocking(/*timeout_ms=*/45000);

    static constexpr uint32_t kAdjustStepTicks = 50;

    bool prev_up = false;
    bool prev_dn = false;
    bool prev_both = false;

    uint32_t last_ui_ticks = 0xFFFFFFFFu;
    int last_ui_angle = -1;
    uint64_t last_ui_ms = 0;

    for (int angle = wallter::calibration::kFirstAngle; angle <= wallter::calibration::kLastAngle; angle += wallter::calibration::kStepAngle) {
        int idx = wallter::angle_to_index(angle);
        if (idx < 0) {
            svc.panicf("BAD ANG %d", angle);
        }

        *svc.target_idx = (uint32_t)idx;

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

            // UI: "20: <hal cnt>"
            char line1[17];
            snprintf(line1, sizeof(line1), "%d: %lu", angle, (unsigned long)svc.target_ticks[*svc.target_idx]);

            // Avoid hammering I2C: only update when changed or periodically.
            uint64_t tnow = svc.now_ms();
            uint32_t cur_ticks = svc.target_ticks[*svc.target_idx];
            if (angle != last_ui_angle || cur_ticks != last_ui_ticks || (tnow - last_ui_ms) > 250ULL) {
                svc.display->print(line1, "");
                last_ui_angle = angle;
                last_ui_ticks = cur_ticks;
                last_ui_ms = tnow;
            }

            bool up = svc.read_extend_pressed();
            bool dn = svc.read_retract_pressed();
            bool both = up && dn;

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

            if (both && !prev_both) {
                // Store current calibration (angles 20..60)
                (void)wallter::calibration::save_from_target_ticks(svc.target_ticks, svc.max_angles);

                svc.display->print("stored", "");
                uint64_t end = svc.now_ms() + 600ULL;
                while (svc.now_ms() < end) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                }

                // Wait for button release so we don't double-trigger
                while (svc.read_extend_pressed() || svc.read_retract_pressed()) {
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
