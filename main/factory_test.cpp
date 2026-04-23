#include "factory_test.hpp"

#include "sdkconfig.h"

#include "app_config.hpp"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_WALLTER_FACTORY_TEST

static const char *TAG = "factory_test";

static constexpr int DIR_LEVEL_EXTEND = 0;  // matches CytronMD PWM_DIR: speed>=0 -> DIR=0
static constexpr int DIR_LEVEL_RETRACT = 1; // matches CytronMD PWM_DIR: speed<0  -> DIR=1

static void set_dir(gpio_num_t pin, int level) {
    gpio_set_level(pin, level ? 1 : 0);
}

static esp_err_t ledc_init_timer() {
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.freq_hz = 500; // multimeter-friendly, still above visible flicker for most setups
    timer.clk_cfg = LEDC_AUTO_CLK;
    return ledc_timer_config(&timer);
}

static esp_err_t ledc_init_channel(int channel_index, gpio_num_t pwm_gpio) {
    ledc_channel_config_t ch = {};
    ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = static_cast<ledc_channel_t>(channel_index);
    ch.timer_sel = LEDC_TIMER_0;
    ch.intr_type = LEDC_INTR_DISABLE;
    ch.gpio_num = pwm_gpio;
    ch.duty = 0;
    ch.hpoint = 0;
    return ledc_channel_config(&ch);
}

static void ledc_set_duty_u8(int channel_index, uint8_t duty) {
    auto channel = static_cast<ledc_channel_t>(channel_index);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void detach_ledc_channel_from_pin(int channel_index, gpio_num_t pin) {
    // Stop PWM output and reset the pin mux back to GPIO.
    auto channel = static_cast<ledc_channel_t>(channel_index);
    (void)ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0);
    gpio_reset_pin(pin);
}

static esp_err_t configure_gpio_output(gpio_num_t pin) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&cfg);
}

static esp_err_t configure_gpio_output_opendrain_pullup(gpio_num_t pin) {
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << pin;
    io.mode = GPIO_MODE_OUTPUT_OD;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&io);
}

static esp_err_t configure_gpio_input_pullup(gpio_num_t pin) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&cfg);
}

static esp_err_t configure_gpio_input_pulldown(gpio_num_t pin) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    return gpio_config(&cfg);
}

static void wait_for_extend_press(const char *why) {
    gpio_num_t extend = static_cast<gpio_num_t>(BUTTON_EXTEND_PIN);
    ESP_LOGI(TAG, "Waiting for EXTEND press (%s) on GPIO%d...", why ? why : "continue", (int)extend);

    // Ensure we start from released state.
    while (gpio_get_level(extend) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Wait for press (active-low), with a small debounce.
    while (gpio_get_level(extend) != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(30));
    while (gpio_get_level(extend) != 0) {
        // Bounce; restart.
        while (gpio_get_level(extend) == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        while (gpio_get_level(extend) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    // Wait for release so one press == one advance.
    while (gpio_get_level(extend) == 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelay(pdMS_TO_TICKS(30));
}

static int stable_read_level(gpio_num_t pin, int samples, int sample_delay_ms) {
    int ones = 0;
    for (int i = 0; i < samples; ++i) {
        ones += (gpio_get_level(pin) ? 1 : 0);
        if (sample_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }
    return (ones > (samples / 2)) ? 1 : 0;
}

static uint32_t lfsr_next(uint32_t &state) {
    // xorshift32 (simple, deterministic pseudo-random)
    state ^= (state << 13);
    state ^= (state >> 17);
    state ^= (state << 5);
    return state;
}

static void log_drive_read(const char *tag, gpio_num_t drive, gpio_num_t sense) {
    gpio_set_level(drive, 0);
    vTaskDelay(pdMS_TO_TICKS(30));
    int r0 = stable_read_level(sense, 9, 0);
    gpio_set_level(drive, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    int r1 = stable_read_level(sense, 9, 0);
    ESP_LOGI(TAG, "%s: drive GPIO%d 0->sense GPIO%d=%d, 1->%d", tag, (int)drive, (int)sense, r0, r1);
}

static void diagnose_pin_coupling(gpio_num_t a, gpio_num_t b, const char *a_name, const char *b_name) {
    if (!GPIO_IS_VALID_OUTPUT_GPIO(a) || !GPIO_IS_VALID_GPIO(b)) {
        ESP_LOGW(TAG, "Coupling diag skipped (%s GPIO%d or %s GPIO%d not valid)", a_name, (int)a, b_name, (int)b);
        return;
    }

    ESP_LOGI(TAG, "Coupling diag: %s(GPIO%d) -> %s(GPIO%d)", a_name, (int)a, b_name, (int)b);

    ESP_ERROR_CHECK(configure_gpio_output(a));

    // Sense with pulldown
    ESP_ERROR_CHECK(configure_gpio_input_pulldown(b));
    log_drive_read("  sense pulldown", a, b);

    // Sense with pullup
    ESP_ERROR_CHECK(configure_gpio_input_pullup(b));
    log_drive_read("  sense pullup", a, b);
}

static void slow_toggle_opendrain(gpio_num_t pin, int cycles, int half_period_ms) {
    // Open-drain: level=1 releases (hi-Z), level=0 drives low.
    // Log readback so we can detect direction-dependent external clamping.
    for (int i = 0; i < cycles; ++i) {
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
        int r_low = stable_read_level(pin, 7, 0);

        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(half_period_ms));
        int r_rel = stable_read_level(pin, 7, 0);

        ESP_LOGI(TAG, "    PWM(GPIO%d) cycle %d/%d readback: drive-low=%d, released=%d",
                 (int)pin,
                 i + 1,
                 cycles,
                 r_low,
                 r_rel);
    }
}

// Bare-bones continuity check between two pins.
// Configures pin_a as push-pull OUTPUT and pin_b as INPUT (no pulls).
// Drives pin_a high then low, reads pin_b each time.
static bool continuity_check(gpio_num_t pin_a, gpio_num_t pin_b) {
    ESP_LOGI(TAG, "  Configuring GPIO%d as OUTPUT, GPIO%d as INPUT (no pull)", (int)pin_a, (int)pin_b);

    // Reset both pins first to clear any prior configuration
    gpio_reset_pin(pin_a);
    gpio_reset_pin(pin_b);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_direction(pin_a, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_b, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin_b, GPIO_FLOATING);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Drive HIGH
    gpio_set_level(pin_a, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    int r1 = gpio_get_level(pin_a);
    int s1 = gpio_get_level(pin_b);
    ESP_LOGI(TAG, "  GPIO%d=1 (readback=%d), GPIO%d reads %d", (int)pin_a, r1, (int)pin_b, s1);

    // Drive LOW
    gpio_set_level(pin_a, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    int r0 = gpio_get_level(pin_a);
    int s0 = gpio_get_level(pin_b);
    ESP_LOGI(TAG, "  GPIO%d=0 (readback=%d), GPIO%d reads %d", (int)pin_a, r0, (int)pin_b, s0);

    // Clean up
    gpio_reset_pin(pin_a);
    gpio_reset_pin(pin_b);

    bool pass = (s1 == 1 && s0 == 0);
    if (!pass) {
        // Try the other direction
        ESP_LOGI(TAG, "  Trying reverse: GPIO%d as OUTPUT, GPIO%d as INPUT", (int)pin_b, (int)pin_a);
        gpio_reset_pin(pin_a);
        gpio_reset_pin(pin_b);
        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_direction(pin_b, GPIO_MODE_OUTPUT);
        gpio_set_direction(pin_a, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin_a, GPIO_FLOATING);
        vTaskDelay(pdMS_TO_TICKS(10));

        gpio_set_level(pin_b, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        r1 = gpio_get_level(pin_b);
        s1 = gpio_get_level(pin_a);
        ESP_LOGI(TAG, "  GPIO%d=1 (readback=%d), GPIO%d reads %d", (int)pin_b, r1, (int)pin_a, s1);

        gpio_set_level(pin_b, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        r0 = gpio_get_level(pin_b);
        s0 = gpio_get_level(pin_a);
        ESP_LOGI(TAG, "  GPIO%d=0 (readback=%d), GPIO%d reads %d", (int)pin_b, r0, (int)pin_a, s0);

        gpio_reset_pin(pin_a);
        gpio_reset_pin(pin_b);

        pass = (s1 == 1 && s0 == 0);
    }
    return pass;
}

static void step_continuity_check() {
    ESP_LOGI(TAG, "=== STEP: Encoder pin continuity check ===");
    ESP_LOGI(TAG, "Install 2 jumpers:");
    ESP_LOGI(TAG, "  M1 CLK (GPIO%d) <-> M2 CLK (GPIO%d)", (int)HAL_CLK[0], (int)HAL_CLK[1]);
    ESP_LOGI(TAG, "  M1 CNT (GPIO%d) <-> M2 CNT (GPIO%d)", (int)HAL_CNT[0], (int)HAL_CNT[1]);
    ESP_LOGI(TAG, "Press EXTEND when jumpers are installed.");
    wait_for_extend_press("jumpers installed");

    gpio_num_t clk0 = static_cast<gpio_num_t>(HAL_CLK[0]);
    gpio_num_t clk1 = static_cast<gpio_num_t>(HAL_CLK[1]);
    gpio_num_t cnt0 = static_cast<gpio_num_t>(HAL_CNT[0]);
    gpio_num_t cnt1 = static_cast<gpio_num_t>(HAL_CNT[1]);

    ESP_LOGI(TAG, "Testing CLK path: GPIO%d <-> GPIO%d", (int)clk0, (int)clk1);
    bool clk_ok = continuity_check(clk0, clk1);
    ESP_LOGW(TAG, "CLK continuity: %s", clk_ok ? "PASS" : "FAIL");

    ESP_LOGI(TAG, "Testing CNT path: GPIO%d <-> GPIO%d", (int)cnt0, (int)cnt1);
    bool cnt_ok = continuity_check(cnt0, cnt1);
    ESP_LOGW(TAG, "CNT continuity: %s", cnt_ok ? "PASS" : "FAIL");

    ESP_LOGW(TAG, "Continuity result: %s", (clk_ok && cnt_ok) ? "ALL PASS" : "FAIL");
}

static void set_all_pwm_off() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        ledc_set_duty_u8(i, 0);
    }
}

static void set_all_dir_safe() {
    for (int i = 0; i < NUM_MOTORS; ++i) {
        gpio_set_level(static_cast<gpio_num_t>(HAL_DIR[i]), DIR_LEVEL_EXTEND);
    }
}


static void step_output_multimeter(int motor_index, int pwm_channel) {
    int label = motor_index + 1;
    gpio_num_t dir_pin = static_cast<gpio_num_t>(HAL_DIR[motor_index]);
    gpio_num_t pwm_pin = static_cast<gpio_num_t>(HAL_PWM[motor_index]);

    ESP_LOGI(TAG, "=== STEP: Motor %d output (measure PWM%d/DIR%d) ===", label, label, label);
    ESP_LOGI(TAG, "Probe pins: PWM GPIO%d, DIR GPIO%d", (int)pwm_pin, (int)dir_pin);
    ESP_LOGI(TAG, "Direction mapping: DIR LOW => EXTEND, DIR HIGH => RETRACT");

    // Ensure only this motor is active.
    set_all_pwm_off();
    set_all_dir_safe();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Motor %d: DIR=EXTEND/LOW (2s)", label);
    set_dir(dir_pin, DIR_LEVEL_EXTEND);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: DIR=RETRACT/HIGH (2s)", label);
    set_dir(dir_pin, DIR_LEVEL_RETRACT);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // PWM duty steps in BOTH directions so LED indicators show PWM intensity either way.
    // Note: if a motor/actuator is connected, this will actively drive it in both directions.

    ESP_LOGI(TAG, "Motor %d: DIR=EXTEND/LOW + PWM 25%% (2s)", label);
    set_dir(dir_pin, DIR_LEVEL_EXTEND);
    ledc_set_duty_u8(pwm_channel, 64);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: DIR=EXTEND/LOW + PWM 75%% (2s)", label);
    ledc_set_duty_u8(pwm_channel, 192);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: DIR=EXTEND/LOW + PWM 100%% (2s)", label);
    ledc_set_duty_u8(pwm_channel, 255);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: PWM off (0.5s)", label);
    ledc_set_duty_u8(pwm_channel, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Motor %d: DIR=RETRACT/HIGH + PWM 25%% (2s)", label);
    set_dir(dir_pin, DIR_LEVEL_RETRACT);
    ledc_set_duty_u8(pwm_channel, 64);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: DIR=RETRACT/HIGH + PWM 75%% (2s)", label);
    ledc_set_duty_u8(pwm_channel, 192);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: DIR=RETRACT/HIGH + PWM 100%% (2s)", label);
    ledc_set_duty_u8(pwm_channel, 255);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Motor %d: PWM off", label);
    ledc_set_duty_u8(pwm_channel, 0);
    // Return to safe direction baseline.
    set_dir(dir_pin, DIR_LEVEL_EXTEND);
    vTaskDelay(pdMS_TO_TICKS(300));

    // Diagnostic: check if PWM and DIR appear electrically coupled when powered.
    // This helps distinguish a true short from a powered current path through external circuitry.
    ESP_LOGI(TAG, "Diagnostic: checking PWM/DIR coupling for Motor %d...", label);
    set_all_pwm_off();
    vTaskDelay(pdMS_TO_TICKS(50));
    detach_ledc_channel_from_pin(pwm_channel, pwm_pin);
    diagnose_pin_coupling(pwm_pin, dir_pin, "PWM", "DIR");
    diagnose_pin_coupling(dir_pin, pwm_pin, "DIR", "PWM");

    // Additional diagnostic: test whether PWM pin is being clamped depending on DIR state.
    // Use open-drain + internal pull-up so we never hard-drive high into an external low.
    ESP_LOGI(TAG, "Diagnostic: PWM open-drain slow toggle under DIR states (measure PWM pin voltage)");
    ESP_ERROR_CHECK(configure_gpio_output(dir_pin));
    ESP_ERROR_CHECK(configure_gpio_output_opendrain_pullup(pwm_pin));
    
    ESP_LOGI(TAG, "  DIR=EXTEND/LOW, PWM toggle (open-drain) 2Hz for 3s");
    set_dir(dir_pin, DIR_LEVEL_EXTEND);
    slow_toggle_opendrain(pwm_pin, /*cycles*/6, /*half_period_ms*/250);

    ESP_LOGI(TAG, "  DIR=RETRACT/HIGH, PWM toggle (open-drain) 2Hz for 3s");
    set_dir(dir_pin, DIR_LEVEL_RETRACT);
    slow_toggle_opendrain(pwm_pin, /*cycles*/6, /*half_period_ms*/250);

    // Restore LEDC on PWM pin
    ESP_ERROR_CHECK(ledc_init_channel(pwm_channel, pwm_pin));
    ledc_set_duty_u8(pwm_channel, 0);
    ESP_ERROR_CHECK(configure_gpio_output(dir_pin));
    set_dir(dir_pin, DIR_LEVEL_EXTEND);
    vTaskDelay(pdMS_TO_TICKS(100));
}
[[noreturn]] void wallter::factory_test::run() {
    ESP_LOGW(TAG, "FACTORY TEST MODE ENABLED (CONFIG_WALLTER_FACTORY_TEST=y)");
    ESP_LOGW(TAG, "Normal application will NOT run.");

    // Print mapping table
    for (int i = 0; i < NUM_MOTORS; ++i) {
        ESP_LOGI(TAG, "Motor %d mapping: PWM=GPIO%d DIR=GPIO%d HAL_CLK=GPIO%d HAL_CNT=GPIO%d",
                 i,
                 (int)HAL_PWM[i],
                 (int)HAL_DIR[i],
                 (int)HAL_CLK[i],
                 (int)HAL_CNT[i]);
    }

    ESP_LOGI(TAG, "Instructions:");
    ESP_LOGI(TAG, "  - Cross-motor encoder loopback: tests all 4 HAL pins with 2 jumpers.");
    ESP_LOGI(TAG, "  - Then PWM/DIR output test for each motor (probe with multimeter).");

    // Configure EXTEND/RETRACT buttons as inputs (active-low).
    ESP_ERROR_CHECK(configure_gpio_input_pullup(static_cast<gpio_num_t>(BUTTON_EXTEND_PIN)));
    ESP_ERROR_CHECK(configure_gpio_input_pullup(static_cast<gpio_num_t>(BUTTON_RETRACT_PIN)));

    // Basic operator check: verify RETRACT button wiring.
    ESP_LOGI(TAG, "Button check: press RETRACT to confirm it works.");
    while (gpio_get_level(static_cast<gpio_num_t>(BUTTON_RETRACT_PIN)) != 0) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    ESP_LOGI(TAG, "RETRACT button detected.");
    // Debounce / wait for release
    while (gpio_get_level(static_cast<gpio_num_t>(BUTTON_RETRACT_PIN)) == 0) {
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    vTaskDelay(pdMS_TO_TICKS(150));

    // Configure DIR pins as outputs.
    for (int i = 0; i < NUM_MOTORS; ++i) {
        ESP_ERROR_CHECK(configure_gpio_output(static_cast<gpio_num_t>(HAL_DIR[i])));
        set_dir(static_cast<gpio_num_t>(HAL_DIR[i]), 0);
    }

    // LEDC setup for PWM
    ESP_ERROR_CHECK(ledc_init_timer());
    for (int i = 0; i < NUM_MOTORS; ++i) {
        // LEDC has 8 channels per speed mode; keep factory test simple.
        ESP_ERROR_CHECK(ledc_init_channel(i, static_cast<gpio_num_t>(HAL_PWM[i])));
        ledc_set_duty_u8(i, 0);
    }

    set_all_dir_safe();

    while (true) {
        // Encoder pin continuity check (all 4 HAL pins with 2 jumpers)
        step_continuity_check();

        // Motor 0 (Motor 1 in operator labeling)
        wait_for_extend_press("next: Motor 1 PWM/DIR output");
        step_output_multimeter(0, 0);

        // Motor 1 (Motor 2 in operator labeling)
        wait_for_extend_press("next: Motor 2 PWM/DIR output");
        step_output_multimeter(1, 1);

        ESP_LOGI(TAG, "Factory test complete.");
        wait_for_extend_press("repeat from start");
    }
}

#endif // CONFIG_WALLTER_FACTORY_TEST
