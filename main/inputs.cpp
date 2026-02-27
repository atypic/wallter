#include "inputs.hpp"

#include "app_config.hpp"
#include "motordriver.hpp"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "soc/soc_caps.h"

#include <stdint.h>

#include "boards.h"

namespace wallter::inputs {

static const char *TAG = "inputs";

static MotorDriver *g_motors = nullptr;
static int g_num_motors = 0;

static gpio_glitch_filter_handle_t g_btn_extend_filter = nullptr;
static gpio_glitch_filter_handle_t g_btn_retract_filter = nullptr;

#if CONFIG_WALLTER_HAL_SIM
static esp_timer_handle_t g_hal_sim_timer = nullptr;
#endif

static volatile bool g_extend_event = false;
static volatile bool g_retract_event = false;
static bool g_boot_menu_requested = false;
static bool g_boot_skip_self_test_requested = false;

// Arduino-style hall tick filter (ms gate).
// We keep the knob in one place by deriving from HAL_CLK_MIN_PULSE_US.
// NOTE: With PCNT we don't do a software min-gap gate here; the peripheral counts
// pulses directly and samples the direction control at the pulse edge.

static pcnt_unit_handle_t g_pcnt_units[NUM_MOTORS] = {nullptr};
static pcnt_channel_handle_t g_pcnt_channels[NUM_MOTORS] = {nullptr};
static int32_t g_pcnt_last_count[NUM_MOTORS] = {0};

static void IRAM_ATTR isr_button_extend(void *arg) {
    (void)arg;
    g_extend_event = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}

static void IRAM_ATTR isr_button_retract(void *arg) {
    (void)arg;
    g_retract_event = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}

static void configure_hal_pcnt() {
#if SOC_PCNT_SUPPORTED
    // Create one PCNT unit per motor.
    // HAL_CLK is the pulse input (we count rising edges only).
    // HAL_CNT is the level input (direction control sampled at the pulse edge).
    for (int i = 0; i < g_num_motors; ++i) {
        if (g_pcnt_units[i]) {
            continue;
        }

        pcnt_unit_config_t unit_cfg = {};
        unit_cfg.low_limit = -32768;
        unit_cfg.high_limit = 32767;
    // Enable driver-managed accumulation on overflow so the effective count
    // range extends beyond the 16-bit hardware counter.
    unit_cfg.flags.accum_count = 1;
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &g_pcnt_units[i]));

    // Enable watch points at the low/high limits. These are required for
    // the driver to detect overflows and maintain the accumulated value.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_units[i], unit_cfg.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(g_pcnt_units[i], unit_cfg.high_limit));

        pcnt_chan_config_t chan_cfg = {};
        chan_cfg.edge_gpio_num = (gpio_num_t)HAL_CLK[i];
        chan_cfg.level_gpio_num = (gpio_num_t)HAL_CNT[i];
        ESP_ERROR_CHECK(pcnt_new_channel(g_pcnt_units[i], &chan_cfg, &g_pcnt_channels[i]));

        // Count on rising edge only.
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(g_pcnt_channels[i],
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_HOLD));

        // Map HAL_CNT to direction:
        // - HAL_CNT==0 -> step IN  (position--)
        // - HAL_CNT==1 -> step OUT (position++)
        // Base edge action is INCREASE; invert it when HAL_CNT is low.
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(g_pcnt_channels[i],
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

        // Optional glitch filter: keep conservative (short spikes only).
        // If you want stronger filtering, tune this value.
        pcnt_glitch_filter_config_t filter_cfg = {};
        filter_cfg.max_glitch_ns = 1000; // 1us
        (void)pcnt_unit_set_glitch_filter(g_pcnt_units[i], &filter_cfg);

        ESP_ERROR_CHECK(pcnt_unit_enable(g_pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(g_pcnt_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(g_pcnt_units[i]));
        g_pcnt_last_count[i] = 0;

        ESP_LOGI(TAG, "HAL PCNT m%d: CLK=%u CNT=%u", i, (unsigned)HAL_CLK[i], (unsigned)HAL_CNT[i]);
    }
#else
    ESP_LOGW(TAG, "PCNT not supported on this target; encoder counting disabled");
#endif
}

static void configure_buttons_gpio() {
    gpio_config_t io = {};
    io.mode = GPIO_MODE_INPUT;
    io.intr_type = GPIO_INTR_NEGEDGE;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pin_bit_mask = (1ULL << BUTTON_EXTEND_PIN) | (1ULL << BUTTON_RETRACT_PIN);
    gpio_config(&io);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)BUTTON_EXTEND_PIN, isr_button_extend, nullptr);
    gpio_isr_handler_add((gpio_num_t)BUTTON_RETRACT_PIN, isr_button_retract, nullptr);

    gpio_pin_glitch_filter_config_t bcfg = {};
    bcfg.gpio_num = (gpio_num_t)BUTTON_EXTEND_PIN;
    gpio_new_pin_glitch_filter(&bcfg, &g_btn_extend_filter);
    bcfg.gpio_num = (gpio_num_t)BUTTON_RETRACT_PIN;
    gpio_new_pin_glitch_filter(&bcfg, &g_btn_retract_filter);
}

static void configure_hal_encoder_gpio_and_isrs() {
    gpio_config_t clk = {};
    clk.mode = GPIO_MODE_INPUT;
    clk.intr_type = GPIO_INTR_DISABLE;
    clk.pull_up_en = GPIO_PULLUP_ENABLE;
    clk.pull_down_en = GPIO_PULLDOWN_DISABLE;
    uint64_t mask = 0;
    for (int i = 0; i < g_num_motors; ++i) {
        unsigned int clk_pin = HAL_CLK[i];
        mask |= (1ULL << clk_pin);
    }
    clk.pin_bit_mask = mask;
    gpio_config(&clk);

    gpio_config_t cnt = {};
    cnt.mode = GPIO_MODE_INPUT;
    cnt.intr_type = GPIO_INTR_DISABLE;
    cnt.pull_up_en = GPIO_PULLUP_ENABLE;
    cnt.pull_down_en = GPIO_PULLDOWN_DISABLE;
    mask = 0;
    for (int i = 0; i < g_num_motors; ++i) {
        // Configure the pin we sample for direction.
        unsigned int sample_pin = HAL_CNT[i];
        mask |= (1ULL << sample_pin);
    }
    cnt.pin_bit_mask = mask;
    gpio_config(&cnt);

    // Use PCNT for counting instead of GPIO ISRs.
    configure_hal_pcnt();
}

void poll_encoder_counts() {
#if SOC_PCNT_SUPPORTED
    if (!g_motors || g_num_motors <= 0) {
        return;
    }
    for (int i = 0; i < g_num_motors; ++i) {
        if (!g_pcnt_units[i]) {
            continue;
        }
        int count = 0;
        if (pcnt_unit_get_count(g_pcnt_units[i], &count) != ESP_OK) {
            continue;
        }
        // With accum_count enabled, pcnt_unit_get_count() returns an extended count
        // that won't wrap at 16-bit limits.
        int32_t cur = (int32_t)count;
        int32_t delta = (int32_t)(cur - g_pcnt_last_count[i]);
        if (delta > 0) {
            for (int s = 0; s < delta; ++s) {
                g_motors[i].incrementStepOut();
            }
        } else if (delta < 0) {
            for (int s = 0; s < -delta; ++s) {
                g_motors[i].incrementStepIn();
            }
        }
        g_pcnt_last_count[i] = cur;
    }
#endif
}

void reset_encoder_counts() {
#if SOC_PCNT_SUPPORTED
    for (int i = 0; i < g_num_motors; ++i) {
        g_pcnt_last_count[i] = 0;
        if (g_pcnt_units[i]) {
            (void)pcnt_unit_clear_count(g_pcnt_units[i]);
        }
    }
#endif
}

#if CONFIG_WALLTER_HAL_SIM
static void hal_sim_timer_cb(void *arg) {
    (void)arg;
    if (!g_motors || g_num_motors <= 0) {
        return;
    }

    // Scale steps based on commanded speed.
    // Notes:
    // - EXTEND (positive speed) -> steps OUT -> position increases
    // - RETRACT (negative speed) -> steps IN  -> position decreases
    // - Home-stop simulation: once position <= 0, retract produces no steps
    const uint32_t period_ms = (uint32_t)CONFIG_WALLTER_HAL_SIM_PERIOD_MS;
    const uint32_t ticks_per_sec_full = (uint32_t)CONFIG_WALLTER_HAL_SIM_TICKS_PER_SEC;
    const int32_t max_pos = (int32_t)CONFIG_WALLTER_HAL_SIM_MAX_POS;

    for (int i = 0; i < g_num_motors; ++i) {
        int32_t spd = g_motors[i].getSpeed();
        if (spd == 0) {
            continue;
        }

        MotorDirection dir = g_motors[i].getDirection();
        int32_t pos = g_motors[i].getPosition();

        if (dir == DIR_RETRACT && pos <= 0) {
            // At home stop: no edges, so control can detect idle and finish homing.
            continue;
        }
        if (dir == DIR_EXTEND && pos >= max_pos) {
            // Artificial max travel.
            continue;
        }

        uint32_t abs_spd = (uint32_t)((spd < 0) ? -spd : spd);
        // MotorDriver speed is conceptually 0..MAXSPEED; clamp to avoid absurd sim rates.
        if (abs_spd > (uint32_t)MAXSPEED) {
            abs_spd = (uint32_t)MAXSPEED;
        }
        // steps = ticks_per_sec_full * (period_ms/1000) * (abs_spd/255)
        uint32_t steps = (ticks_per_sec_full * period_ms * abs_spd + (255U * 1000U / 2U)) / (255U * 1000U);
        if (steps == 0) {
            steps = 1;
        }

        for (uint32_t s = 0; s < steps; ++s) {
            if (dir == DIR_EXTEND) {
                g_motors[i].incrementStepOut();
            } else if (dir == DIR_RETRACT) {
                g_motors[i].incrementStepIn();
            }
        }
    }
}

static void start_hal_sim_timer() {
    if (g_hal_sim_timer) {
        return;
    }

    esp_timer_create_args_t args = {};
    args.callback = &hal_sim_timer_cb;
    args.arg = nullptr;
    args.dispatch_method = ESP_TIMER_TASK;
    args.name = "hal_sim";

    ESP_ERROR_CHECK(esp_timer_create(&args, &g_hal_sim_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_hal_sim_timer, (uint64_t)CONFIG_WALLTER_HAL_SIM_PERIOD_MS * 1000ULL));
}
#endif

void init(MotorDriver *motors, int num_motors) {
    g_motors = motors;
    g_num_motors = num_motors;

    configure_buttons_gpio();

#if CONFIG_WALLTER_HAL_SIM
    // In simulation mode we do not attach encoder GPIO ISRs.
    start_hal_sim_timer();
#else
    configure_hal_encoder_gpio_and_isrs();
#endif

    if (g_btn_extend_filter) {
        gpio_glitch_filter_enable(g_btn_extend_filter);
    }
    if (g_btn_retract_filter) {
        gpio_glitch_filter_enable(g_btn_retract_filter);
    }

    bool boot_extend_pressed = (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
    bool boot_retract_pressed = (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0);

    // Legacy behavior alignment:
    // - EXTEND-only at boot enters the boot menu.
    // - BOTH at boot skips the automatic self-test.
    g_boot_skip_self_test_requested = boot_extend_pressed && boot_retract_pressed;
    g_boot_menu_requested = boot_extend_pressed && !boot_retract_pressed;

    // Seed event latches so first poll sees the boot-held buttons.
    if (boot_extend_pressed) g_extend_event = true;
    if (boot_retract_pressed) g_retract_event = true;
}

Events poll_button_events() {
    Events e;
    e.extend = g_extend_event;
    e.retract = g_retract_event;
    g_extend_event = false;
    g_retract_event = false;
    return e;
}

void clear_button_events() {
    g_extend_event = false;
    g_retract_event = false;
}

bool read_extend_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}

bool read_retract_pressed() {
    return (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}

bool boot_menu_requested() {
    return g_boot_menu_requested;
}

bool boot_skip_self_test_requested() {
    return g_boot_skip_self_test_requested;
}

void clear_boot_menu_requested() {
    g_boot_menu_requested = false;
    g_boot_skip_self_test_requested = false;
}

bool skip_self_test_requested() {
    // Backwards-compatible alias.
    return g_boot_skip_self_test_requested;
}

void clear_skip_self_test_requested() {
    // Backwards-compatible alias.
    g_boot_skip_self_test_requested = false;
}

} // namespace wallter::inputs
