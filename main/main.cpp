#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.hpp"
#include "accel.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "commands.hpp"
#include "modes.hpp"
#include "motor_control.hpp"
#include "inputs.hpp"
#include "display.hpp"
#include "motordriver.hpp"
#include "factory_test.hpp"
#include "ble_ota.hpp"

static const char *TAG = "main";

// Forward declaration for button handler
static void handle_buttons();
static void setup();
static Display display;
static MotorDriver motors[NUM_MOTORS] = {
    MotorDriver(HAL_PWM[0], HAL_DIR[0], LEDC_CHANNEL_0, MOTOR_OUTPUT_INVERT),
    MotorDriver(HAL_PWM[1], HAL_DIR[1], LEDC_CHANNEL_1, MOTOR_OUTPUT_INVERT)
#if NUM_MOTORS > 2
    , MotorDriver(HAL_PWM[2], HAL_DIR[2], LEDC_CHANNEL_2, MOTOR_OUTPUT_INVERT)
#endif
#if NUM_MOTORS > 3
    , MotorDriver(HAL_PWM[3], HAL_DIR[3], LEDC_CHANNEL_3, MOTOR_OUTPUT_INVERT)
#endif
};

// Master motor index (tune as needed)
#define MASTER_MOTOR 0

static constexpr int MAX_ANGLES = wallter::kMaxAngles;
static uint32_t g_target_ticks[MAX_ANGLES] = {0};
static float g_target_angles[MAX_ANGLES] = {0.0f};
static uint32_t g_target_idx = 0;

static wallter::calibration::CalMeta g_cal_meta = { (uint8_t)DEFAULT_CAL_MIN_ANGLE, (uint8_t)DEFAULT_CAL_MAX_ANGLE, (int8_t)DEFAULT_ACCEL_ANGLE_OFFSET_TENTHS };
static int g_min_target_idx = 0;
static int g_max_target_idx = 0;

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void apply_calibration_meta_and_shift() {
    // The real table (and min index) is built by rebuild_target_table() after homing.
    // Pre-homing: use the full slot range as a safe placeholder.
    g_min_target_idx = 0;
    g_max_target_idx = MAX_ANGLES - 1;
    g_target_idx = 0;
}

static void dump_calibration_table() {
    ESP_LOGI(TAG, "Max angle setting: %u° (HIGHEST=%d STEP=%d slots=%d)",
             (unsigned)g_cal_meta.max_angle_deg, (int)HIGHEST_ANGLE, (int)ANGLE_STEP, (int)MAX_ANGLES);
}

static void load_calibration_all() {
    wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);
    (void)wallter::calibration::load_meta_or_default(g_cal_meta);
    apply_calibration_meta_and_shift();

    // Apply accelerometer angle offset from calibration.
    wallter::accel::set_angle_offset((float)g_cal_meta.angle_offset_tenths * 0.1f);

    // Pre-homing placeholder: fill with every ANGLE_STEP multiple from ANGLE_STEP to HIGHEST_ANGLE.
    // rebuild_target_table() replaces this with real values after the home angle is measured.
    for (int i = 0; i < MAX_ANGLES; ++i) {
        g_target_angles[i] = (float)((i + 1) * ANGLE_STEP);
    }

    dump_calibration_table();

    // Always start at the configured minimum angle on boot/reload.
    // (Calibration mode may leave target_idx pointing at the last visited step.)
    g_target_idx = (uint32_t)g_min_target_idx;
}

// Rebuilds the target angle table after homing completes.
// The table always starts at the calibrated min angle and goes up in
// ANGLE_STEP increments to the calibrated max.
static void rebuild_target_table() {
    float home_deg = wallter::accel::read_angle_deg();

    // User-configured min, snapped up to a step boundary.
    int min_deg = (int)g_cal_meta.min_angle_deg;
    min_deg = ((min_deg + ANGLE_STEP - 1) / ANGLE_STEP) * ANGLE_STEP;
    if (min_deg < ANGLE_STEP) min_deg = ANGLE_STEP;

    // User-configured max, snapped down to a step boundary.
    int max_deg = (int)g_cal_meta.max_angle_deg;
    max_deg = (max_deg / ANGLE_STEP) * ANGLE_STEP;
    if (max_deg > HIGHEST_ANGLE) max_deg = HIGHEST_ANGLE;
    if (max_deg < min_deg) max_deg = min_deg;

    int count = 0;
    for (int a = min_deg; a <= max_deg && count < MAX_ANGLES; a += ANGLE_STEP) {
        g_target_angles[count] = (float)a;
        g_target_ticks[count] = 0;
        count++;
    }
    // Pad unused slots with the last angle so out-of-bounds accesses are harmless.
    float last = (count > 0) ? g_target_angles[count - 1] : (float)min_deg;
    for (int i = count; i < MAX_ANGLES; ++i) {
        g_target_angles[i] = last;
        g_target_ticks[i] = 0;
    }

    g_min_target_idx = 0;
    g_max_target_idx = (count > 0) ? count - 1 : 0;
    g_target_idx = 0;

    wallter::control::update_limits(g_min_target_idx, g_max_target_idx);

    ESP_LOGI(TAG, "Targets: home=%.1f° min=%d° max=%d° count=%d",
             (double)home_deg, min_deg, max_deg, count);
    display.update_target_view(g_target_angles[0], 0);
}

// Motor control state machine lives in wallter::control


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Init");

    ESP_LOGI(TAG, "Firmware: %s", VERSION_STRING);

    // NVS for calibration persistence
    wallter::calibration::init_nvs();
    load_calibration_all();

    setup();

    wallter::modes::Services svc;
    svc.display = &display;
    svc.motors = motors;
    svc.num_motors = NUM_MOTORS;
    svc.master_motor_index = MASTER_MOTOR;
    svc.max_angles = MAX_ANGLES;
    svc.target_idx = &g_target_idx;
    svc.cal_meta = &g_cal_meta;
    svc.read_extend_pressed = &wallter::inputs::read_extend_pressed;
    svc.read_retract_pressed = &wallter::inputs::read_retract_pressed;
    svc.set_current_command = &wallter::control::set_command;
    svc.motor_control_iteration = &wallter::control::iterate;
    svc.run_homing_blocking = &wallter::control::run_homing_blocking;
    svc.reset_tick_counters = &wallter::control::reset_tick_counters;
    svc.panicf = &wallter::control::panicf;
    svc.now_ms = &now_ms;

    // Boot behavior (legacy-compatible):
    // - Hold EXTEND (only) at boot -> boot menu
    // - Hold BOTH at boot -> skip auto self-test
    // - Otherwise -> auto self-test then home
    bool boot_menu = wallter::inputs::boot_menu_requested();
    bool skip_self_test = wallter::inputs::boot_skip_self_test_requested();

    if (boot_menu) {
        // Clear any latched interrupt flags before entering menu.
        wallter::inputs::clear_button_events();

        wallter::modes::BootMenuChoice choice = wallter::modes::run_boot_menu(svc);
        if (choice == wallter::modes::MENU_SET_MAX_ANGLE) {
            wallter::modes::run_set_max_angle_mode(svc);
        } else if (choice == wallter::modes::MENU_SET_MIN_ANGLE) {
            wallter::modes::run_set_min_angle_mode(svc);
        } else if (choice == wallter::modes::MENU_SET_OFFSET) {
            wallter::modes::run_set_angle_offset_mode(svc);
        } else if (choice == wallter::modes::MENU_SELF_TEST) {
            wallter::modes::run_self_test_sequence(svc);
        } else if (choice == wallter::modes::MENU_HAL_TEST) {
            wallter::modes::run_hal_feedback_test(svc);
        } else if (choice == wallter::modes::MENU_RESET_CAL) {
            wallter::modes::run_reset_calibration_data(svc);
        } else {
            display.print("Jog mode", "Starting...");
            wallter::modes::run_jog_mode(svc);
        }

        // Re-load calibration after a calibration run (ensures interpolation is applied consistently)
        load_calibration_all();
        wallter::control::update_limits(g_min_target_idx, g_max_target_idx);
        wallter::inputs::clear_button_events();
        wallter::inputs::clear_boot_menu_requested();
        wallter::inputs::clear_skip_self_test_requested();
    } else {
        if (skip_self_test) {
            ESP_LOGI(TAG, "Normal boot: skipping self-test (both buttons held)");
        } else {
            ESP_LOGI(TAG, "Normal boot: running self-test");
            wallter::modes::run_self_test_sequence(svc);
            // Self-test drives motors directly, but encoder ISRs still maintain MotorDriver
            // position. Homing should establish the reference; do not zero positions here.
        }

        wallter::inputs::clear_button_events();
        wallter::inputs::clear_skip_self_test_requested();
    }

    // Self testing done or skipped, let's go home.
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);

    // Make sure we reset idle timer since we've been using the motors.
    wallter::control::reset_idle_timer();
    wallter::control::set_command(wallter::CMD_HOME);

    // this is the main loop
    int prev_cmd = wallter::CMD_HOME;
    while (1) {
        // Handle buttons in a dedicated function (Arduino-style)
        handle_buttons();

        wallter::control::iterate();

        // Rebuild target table whenever homing completes.
        int cur_cmd = wallter::control::command();
        if (prev_cmd == wallter::CMD_HOME && cur_cmd == wallter::CMD_STOP) {
            rebuild_target_table();

            // Auto-extend to the calibrated minimum if we're below it.
            float home_angle = wallter::control::master_angle_deg();
            float first_target = g_target_angles[0];
            if (home_angle < first_target - 0.3f) {
                ESP_LOGI(TAG, "Auto-extending from %.1f° to min target %.1f°",
                         (double)home_angle, (double)first_target);
                g_target_idx = 0;
                wallter::control::set_command(wallter::CMD_EXTEND);
            } else {
                // Already at or above minimum; just show current angle.
                display.update_current_view(home_angle);
            }
        }

        // Periodic logging and stall checks
        wallter::control::print_state(500);
        if (cur_cmd == wallter::CMD_HOME) {
            // Don't let target view override the homing view.
            display.update_homing_view();
        } else {
            float cur_angle = wallter::control::master_angle_deg();
            if (cur_cmd == wallter::CMD_STOP) {
                // Snap the displayed angle once when we enter STOP,
                // then keep it frozen so it doesn't jitter while idle.
                if (prev_cmd != wallter::CMD_STOP) {
                    display.update_current_view(cur_angle);
                }
            } else {
                float tgt_angle   = g_target_angles[g_target_idx];
                float start_angle = wallter::control::progress_start_angle_deg();
                uint8_t pct = wallter::control::compute_progress_percent(
                    (int32_t)(cur_angle * 10.0f),
                    (int32_t)(start_angle * 10.0f),
                    (int32_t)(tgt_angle * 10.0f));
                display.update_target_view(tgt_angle, pct);
            }
            wallter::ble_ota::set_current_angle_deg(cur_angle);
            wallter::control::error_check_motor_positions();
        }
        prev_cmd = cur_cmd;
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Extracted setup routine (Arduino-style)
static void setup() {
#if CONFIG_WALLTER_FACTORY_TEST
    wallter::factory_test::run();
#endif
    // Configure GPIO early so the boot-menu hold can be latched quickly.
    wallter::inputs::init(motors, NUM_MOTORS);

    // Init accelerometer (angle source) before motors/display.
    if (wallter::accel::init() != ESP_OK) {
        ESP_LOGE("main", "BMA400 init failed — angle readings unavailable");
    }

    // Init motors and display
    for (int i = 0; i < NUM_MOTORS; ++i) motors[i].init();
    vTaskDelay(pdMS_TO_TICKS(500));
    display.init();
    vTaskDelay(pdMS_TO_TICKS(500));
    display.set_refresh_rate(1.0f);
    // Run startup animation like old display
    display.startup_animation();

    wallter::control::Context ctx;
    ctx.display = &display;
    ctx.motors = motors;
    ctx.num_motors = NUM_MOTORS;
    ctx.master_motor_index = MASTER_MOTOR;
    ctx.target_ticks = g_target_ticks;
    ctx.target_angles = g_target_angles;
    ctx.max_angles = MAX_ANGLES;
    ctx.target_idx = &g_target_idx;
    ctx.min_target_idx = g_min_target_idx;
    ctx.max_target_idx = g_max_target_idx;
    wallter::control::init(ctx);

    // Reset counters and positions like Arduino setup().
    // Note: position will be re-zeroed by a successful homing sequence.
    wallter::control::reset_tick_counters(0, true);
    wallter::control::reset_motor_positions();

    // (button filters + initial button latching handled before animation)

    // Configure follower PID motors
    for (int p = 1; p < NUM_MOTORS; p++) {
        motors[p].pidBegin(8, 0.001, 0);
        motors[p].pidConfigureLimits(MINSPEED, MAXSPEED);
        motors[p].pidSetSampleTime(50);
        motors[p].pidStart();
    }

    g_target_idx = (uint32_t)g_min_target_idx;

    // Leave command STOP here. Boot flow will run self-test/menu first, then
    // explicitly enter homing once those routines complete.
    wallter::control::set_command(wallter::CMD_STOP);

    // Button ISRs configured in configure_gpio().

    // Initial display view — real targets built after first homing.
    display.update_target_view(g_target_angles[0], 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);

    // BLE OTA service (runs in background tasks when enabled).
    wallter::ble_ota::set_settings_callbacks(
        []() -> wallter::ble_ota::Settings {
            return {
                .min_angle_deg = g_cal_meta.min_angle_deg,
                .max_angle_deg = g_cal_meta.max_angle_deg,
                .angle_offset_tenths = g_cal_meta.angle_offset_tenths,
            };
        },
        [](const wallter::ble_ota::Settings &s) -> bool {
            g_cal_meta.min_angle_deg = s.min_angle_deg;
            g_cal_meta.max_angle_deg = s.max_angle_deg;
            g_cal_meta.angle_offset_tenths = s.angle_offset_tenths;
            wallter::accel::set_angle_offset((float)s.angle_offset_tenths * 0.1f);
            return wallter::calibration::save_meta(g_cal_meta) == ESP_OK;
        }
    );
    wallter::ble_ota::set_version_string(VERSION_STRING);
    wallter::ble_ota::init();
}

// Replicate Arduino handle_buttons() logic without blocking loops
static void handle_buttons() {
    auto e = wallter::inputs::poll_button_events();

    auto ble = wallter::ble_ota::poll_button_events();
    if (ble.stop) {
        wallter::control::set_command(wallter::CMD_STOP);
    }
    if (ble.home) {
        wallter::control::set_command(wallter::CMD_HOME);
    }

    wallter::control::handle_buttons(e.extend || ble.extend, e.retract || ble.retract);
}
