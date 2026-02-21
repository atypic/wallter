#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "hal/gpio_ll.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.hpp"
#include "angle_utils.hpp"
#include "calibration_store.hpp"
#include "commands.hpp"
#include "modes.hpp"
#include "motor_control.hpp"
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

// Master motor index (tune as needed)
#define MASTER_MOTOR 0

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

static inline uint64_t now_ms() {
    return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static inline bool read_extend_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}
static inline bool read_retract_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}

// Motor control state machine lives in wallter::control

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
    svc.read_extend_pressed = &read_extend_pressed;
    svc.read_retract_pressed = &read_retract_pressed;
    svc.set_current_command = &wallter::control::set_command;
    svc.motor_control_iteration = &wallter::control::iterate;
    svc.run_homing_blocking = &wallter::control::run_homing_blocking;
    svc.reset_tick_counters = &wallter::control::reset_tick_counters;
    svc.panicf = &wallter::control::panicf;
    svc.now_ms = &now_ms;

    // Boot menu: hold EXTEND while starting (latched during setup after GPIO config)
    bool boot_extend = g_boot_menu_requested;
    bool boot_retract = read_retract_pressed();
    if (boot_extend) {
        // Clear any latched interrupt flags before entering menu.
        g_extend_pressed = false;
        g_retract_pressed = false;

        wallter::modes::BootMenuChoice choice = wallter::modes::run_boot_menu(svc);
        if (choice == wallter::modes::MENU_CALIBRATE) {
            display.print("Calibration", "Starting...");
            wallter::modes::run_calibration_mode(svc);
        } else {
            wallter::modes::run_self_test_sequence(svc);
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
    bool extend  = g_extend_pressed;
    bool retract = g_retract_pressed;
    g_extend_pressed  = false;
    g_retract_pressed = false;

    wallter::control::handle_buttons(extend, retract);
}
