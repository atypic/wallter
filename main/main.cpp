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

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

// Motor control state machine lives in wallter::control


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Init");

    // NVS for calibration persistence
    wallter::calibration::init_nvs();
    wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);

    setup();

    wallter::modes::Services svc;
    svc.display = &display;
    svc.motors = motors;
    svc.num_motors = NUM_MOTORS;
    svc.master_motor_index = MASTER_MOTOR;
    svc.target_ticks = g_target_ticks;
    svc.max_angles = MAX_ANGLES;
    svc.target_idx = &g_target_idx;
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
    bool boot_retract = wallter::inputs::read_retract_pressed();
    if (boot_extend) {
        // Clear any latched interrupt flags before entering menu.
        wallter::inputs::clear_button_events();

        wallter::modes::BootMenuChoice choice = wallter::modes::run_boot_menu(svc);
        if (choice == wallter::modes::MENU_CALIBRATE) {
            display.print("Calibration", "Starting...");
            wallter::modes::run_calibration_mode(svc);
        } else {
            wallter::modes::run_self_test_sequence(svc);
        }

        // Re-load calibration after a calibration run (ensures interpolation is applied consistently)
        wallter::calibration::load_or_default(g_target_ticks, MAX_ANGLES);
        wallter::inputs::clear_button_events();
        wallter::inputs::clear_boot_menu_requested();
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
        wallter::control::error_check_motor_positions();
        display.refresh();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Extracted setup routine (Arduino-style)
static void setup() {
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

    g_target_idx = 0;
    wallter::control::set_command(wallter::CMD_HOME);

    // Button ISRs configured in configure_gpio().

    // Initial display view matching Arduino setup
    display.update_target_view(LOWEST_ANGLE, 0);
    display.update_homing_view();
    display.set_view(LCD_HOMING_VIEW);
}

// Replicate Arduino handle_buttons() logic without blocking loops
static void handle_buttons() {
    auto e = wallter::inputs::poll_button_events();
    wallter::control::handle_buttons(e.extend, e.retract);
}
