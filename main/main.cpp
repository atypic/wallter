#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.hpp"
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
    MotorDriver(HAL_PWM[0], HAL_DIR[0]),
    MotorDriver(HAL_PWM[1], HAL_DIR[1])
#if NUM_MOTORS > 2
    , MotorDriver(HAL_PWM[2], HAL_DIR[2])
#endif
#if NUM_MOTORS > 3
    , MotorDriver(HAL_PWM[3], HAL_DIR[3])
#endif
};

// Master motor index (tune as needed)
#define MASTER_MOTOR 0

// Idle helpers and master speed computation
// Target positions (ticks) for angles (match Arduino flow)
static constexpr int MAX_ANGLES = wallter::kMaxAngles;
static uint32_t g_target_ticks[MAX_ANGLES] = {0};
static uint32_t g_target_idx = 0;

static wallter::calibration::CalMeta g_cal_meta = { (uint8_t)LOWEST_ANGLE, (uint8_t)HIGHEST_ANGLE };
static int g_min_target_idx = 0;
static int g_max_target_idx = 0;
static uint32_t g_home_offset_raw_ticks = 0;

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void apply_calibration_meta_and_shift() {
    // Validate and compute indices.
    int min_idx = wallter::angle_to_index((int)g_cal_meta.min_angle_deg);
    int max_idx = wallter::angle_to_index((int)g_cal_meta.max_angle_deg);
    if (min_idx < 0) min_idx = 0;
    if (max_idx < 0) max_idx = MAX_ANGLES - 1;
    if (min_idx > max_idx) {
        min_idx = 0;
        max_idx = MAX_ANGLES - 1;
        g_cal_meta.min_angle_deg = (uint8_t)LOWEST_ANGLE;
        g_cal_meta.max_angle_deg = (uint8_t)HIGHEST_ANGLE;
    }

    // Compute raw offset (in the unshifted table) for the chosen minimum.
    g_home_offset_raw_ticks = g_target_ticks[min_idx];

    // Shift in-place so that the min angle is tick 0. Angles below min become 0.
    for (int i = 0; i < MAX_ANGLES; ++i) {
        if (i < min_idx) {
            g_target_ticks[i] = 0;
            continue;
        }
        uint32_t t = g_target_ticks[i];
        g_target_ticks[i] = (t >= g_home_offset_raw_ticks) ? (t - g_home_offset_raw_ticks) : 0;
    }

    g_min_target_idx = min_idx;
    g_max_target_idx = max_idx;

    if ((int)g_target_idx < g_min_target_idx) g_target_idx = (uint32_t)g_min_target_idx;
    if ((int)g_target_idx > g_max_target_idx) g_target_idx = (uint32_t)g_max_target_idx;
}

static void load_calibration_all() {
    wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);
    (void)wallter::calibration::load_meta_or_default(g_cal_meta);
    apply_calibration_meta_and_shift();

    // Always start at the configured minimum angle on boot/reload.
    // (Calibration mode may leave target_idx pointing at the last visited step.)
    g_target_idx = (uint32_t)g_min_target_idx;
}

// Motor control state machine lives in wallter::control


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Init");

    // NVS for calibration persistence
    wallter::calibration::init_nvs();
    load_calibration_all();

    setup();

    wallter::modes::Services svc;
    svc.display = &display;
    svc.motors = motors;
    svc.num_motors = NUM_MOTORS;
    svc.master_motor_index = MASTER_MOTOR;
    svc.target_ticks = g_target_ticks;
    svc.max_angles = MAX_ANGLES;
    svc.target_idx = &g_target_idx;
    svc.cal_meta = &g_cal_meta;
    svc.home_offset_raw_ticks = &g_home_offset_raw_ticks;
    svc.read_extend_pressed = &wallter::inputs::read_extend_pressed;
    svc.read_retract_pressed = &wallter::inputs::read_retract_pressed;
    svc.set_current_command = &wallter::control::set_command;
    svc.motor_control_iteration = &wallter::control::iterate;
    svc.run_homing_blocking = &wallter::control::run_homing_blocking;
    svc.reset_tick_counters = &wallter::control::reset_tick_counters;
    svc.panicf = &wallter::control::panicf;
    svc.now_ms = &now_ms;

    // Boot menu: hold EXTEND while starting (latched during setup after GPIO config)
    bool boot_extend = wallter::inputs::boot_menu_requested();
    bool boot_retract = wallter::inputs::skip_self_test_requested() || wallter::inputs::read_retract_pressed();
    if (boot_extend) {
        // Clear any latched interrupt flags before entering menu.
        wallter::inputs::clear_button_events();

        wallter::modes::BootMenuChoice choice = wallter::modes::run_boot_menu(svc);
        if (choice == wallter::modes::MENU_CALIBRATE) {
            display.print("Calibration", "Starting...");
            wallter::modes::run_calibration_mode(svc);
        } else if (choice == wallter::modes::MENU_SELF_TEST) {
            wallter::modes::run_self_test_sequence(svc);
        } else {
            display.print("Jog mode", "Starting...");
            wallter::modes::run_jog_mode(svc);
        }

        // Re-load calibration after a calibration run (ensures interpolation is applied consistently)
        load_calibration_all();
        wallter::control::update_limits(g_min_target_idx, g_max_target_idx, g_home_offset_raw_ticks);
        wallter::inputs::clear_button_events();
        wallter::inputs::clear_boot_menu_requested();
        wallter::inputs::clear_skip_self_test_requested();
    } else {
        if (boot_retract) {
            ESP_LOGI(TAG, "Normal boot: skipping self-test (retract held at boot)");
        } else {
            ESP_LOGI(TAG, "Normal boot: running self-test (hold retract at boot to skip)");
            wallter::modes::run_self_test_sequence(svc);
        }

        wallter::inputs::clear_button_events();
        wallter::inputs::clear_skip_self_test_requested();
    }

    // Self testing done or skipped, let's go home.
    // Post-setup initial view
    float initial_angle = (float)(LOWEST_ANGLE + (int)g_target_idx * ANGLE_STEP);
    display.update_target_view(initial_angle, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);

    // Make sure we reset idle timer since we've been using the motors.
    wallter::control::reset_idle_timer();
    wallter::control::set_command(wallter::CMD_HOME);

    // this is the main loop
    while (1) {
        // Handle buttons in a dedicated function (Arduino-style)
        handle_buttons();

        wallter::control::iterate();

        // Periodic logging and stall checks, update target view
        wallter::control::print_state(500);
        uint32_t target_ticks = g_target_ticks[g_target_idx];
        int32_t pos           = wallter::control::master_position();
        int32_t start_pos     = wallter::control::progress_start_ticks();
        uint8_t pct = wallter::control::compute_progress_percent(pos, start_pos, (int32_t)target_ticks);
        float tgt_angle = (g_target_idx == 0) ? LOWEST_ANGLE : (g_target_idx * ANGLE_STEP + LOWEST_ANGLE);
        display.update_target_view(tgt_angle, pct);
        wallter::ble_ota::set_current_angle_deg(tgt_angle);
        wallter::control::error_check_motor_positions();
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
    ctx.max_angles = MAX_ANGLES;
    ctx.target_idx = &g_target_idx;
    ctx.min_target_idx = g_min_target_idx;
    ctx.max_target_idx = g_max_target_idx;
    ctx.home_offset_raw_ticks = g_home_offset_raw_ticks;
    wallter::control::init(ctx);

    // Reset counters and positions like Arduino setup()
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
    wallter::control::set_command(wallter::CMD_HOME);

    // Button ISRs configured in configure_gpio().

    // Initial display view matching Arduino setup
    float initial_angle = (float)(LOWEST_ANGLE + (int)g_target_idx * ANGLE_STEP);
    display.update_target_view(initial_angle, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);

    // BLE OTA service (runs in background tasks when enabled).
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
