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

static bool loopback_drive_and_sense(gpio_num_t drive, gpio_num_t sense, int toggles, int settle_ms) {
    esp_err_t err;

    err = configure_gpio_output(drive);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d cannot be OUTPUT (err=%d)", (int)drive, (int)err);
        return false;
    }
    err = configure_gpio_input_pulldown(sense);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPIO%d cannot be INPUT pulldown (err=%d)", (int)sense, (int)err);
        return false;
    }

    // Baseline check: if sense follows drive even with just DC levels,
    // it's almost certainly hard-connected (or pulled strongly).
    gpio_set_level(drive, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    int base0 = stable_read_level(sense, /*samples*/9, /*sample_delay_ms*/0);
    gpio_set_level(drive, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    int base1 = stable_read_level(sense, /*samples*/9, /*sample_delay_ms*/0);
    ESP_LOGI(TAG, "Baseline: drive GPIO%d=0 -> sense GPIO%d=%d, drive=1 -> sense=%d", (int)drive, (int)sense, base0, base1);

    int matches = 0;
    int total = 0;
    bool saw0 = false;
    bool saw1 = false;

    uint32_t prng = 0xA5A5F00Du ^ (uint32_t)drive ^ ((uint32_t)sense << 8);

    for (int i = 0; i < toggles; ++i) {
        int level = (int)(lfsr_next(prng) & 1u);
        gpio_set_level(drive, level);
        vTaskDelay(pdMS_TO_TICKS(settle_ms));

        int read = stable_read_level(sense, /*samples*/5, /*sample_delay_ms*/0);
        saw0 |= (read == 0);
        saw1 |= (read == 1);
        if (read == level) {
            matches++;
        }
        total++;
    }

    ESP_LOGI(TAG,
             "Loopback drive GPIO%d -> sense GPIO%d: %d/%d matches (saw0=%d saw1=%d)",
             (int)drive,
             (int)sense,
             matches,
             total,
             saw0 ? 1 : 0,
             saw1 ? 1 : 0);

    // Require near-perfect match AND that we observed both levels on the sensed pin.
    return (matches >= (total - 1)) && saw0 && saw1;
}

static bool loopback_hal_clk_cnt(int motor_index, int toggles, int settle_ms) {
    gpio_num_t clk = static_cast<gpio_num_t>(HAL_CLK[motor_index]);
    gpio_num_t cnt = static_cast<gpio_num_t>(HAL_CNT[motor_index]);

    // Drive whichever side is actually output-capable.
    gpio_num_t drive = GPIO_IS_VALID_OUTPUT_GPIO(clk) ? clk : cnt;
    gpio_num_t sense = (drive == clk) ? cnt : clk;

    if (!GPIO_IS_VALID_OUTPUT_GPIO(drive)) {
        ESP_LOGE(TAG,
                 "Motor %d loopback: neither HAL_CLK(GPIO%d) nor HAL_CNT(GPIO%d) is output-capable; cannot test",
                 motor_index + 1,
                 (int)clk,
                 (int)cnt);
        return false;
    }

    ESP_LOGI(TAG,
             "Motor %d loopback: driving GPIO%d (output-capable), sensing GPIO%d",
             motor_index + 1,
             (int)drive,
             (int)sense);

    bool ok = loopback_drive_and_sense(drive, sense, toggles, settle_ms);

    // If both pins are output-capable, try the reverse direction too (helpful if one side is loaded).
    if (!ok && GPIO_IS_VALID_OUTPUT_GPIO(clk) && GPIO_IS_VALID_OUTPUT_GPIO(cnt)) {
        ESP_LOGW(TAG, "Motor %d loopback: first direction failed; trying reverse", motor_index + 1);
        ok = loopback_drive_and_sense(sense, drive, toggles, settle_ms);
    }

    ESP_LOGI(TAG, "Motor %d loopback pins: HAL_CLK(GPIO%d) <-> HAL_CNT(GPIO%d) result=%s",
             motor_index + 1,
             (int)clk,
             (int)cnt,
             ok ? "PASS" : "FAIL");
    return ok;
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

static void step_loopback_auto(int motor_index) {
    int label = motor_index + 1;
    ESP_LOGI(TAG, "=== STEP: Motor %d encoder loopback (HAL_CLK%d <-> HAL_CNT%d) ===", label, label, label);
    ESP_LOGI(TAG, "Install jumper: HAL_CLK(GPIO%d) <-> HAL_CNT(GPIO%d) for Motor %d",
             (int)HAL_CLK[motor_index],
             (int)HAL_CNT[motor_index],
             label);

    bool ok = loopback_hal_clk_cnt(motor_index, /*toggles*/40, /*settle_ms*/30);
    ESP_LOGW(TAG, "Motor %d loopback result: %s", label, ok ? "PASS" : "FAIL");
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
    ESP_LOGI(TAG, "  - Test always starts at Motor 1 (index 0). Press EXTEND to advance steps.");
    ESP_LOGI(TAG, "  - STEP A: automatic HAL_CLK/HAL_CNT loopback check for that motor.");
    ESP_LOGI(TAG, "  - STEP B: output PWM/DIR for that motor so you can probe with a multimeter.");

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
        // Motor 0 (Motor 1 in operator labeling)
        step_loopback_auto(0);
        wait_for_extend_press("next: Motor 1 PWM/DIR output");
        step_output_multimeter(0, 0);

        // Motor 1 (Motor 2 in operator labeling)
        wait_for_extend_press("next: Motor 2 loopback auto-check");
        step_loopback_auto(1);
        wait_for_extend_press("next: Motor 2 PWM/DIR output");
        step_output_multimeter(1, 1);

        ESP_LOGI(TAG, "Factory test complete.");
        wait_for_extend_press("repeat from Motor 1");
    }
}

#endif // CONFIG_WALLTER_FACTORY_TEST
