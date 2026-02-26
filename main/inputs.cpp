#include "inputs.hpp"

#include "app_config.hpp"
#include "motordriver.hpp"

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"
#include "soc/soc_caps.h"

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
static constexpr uint32_t HALL_FILTER_MS = (uint32_t)((HAL_CLK_MIN_PULSE_US + 999U) / 1000U);
static uint32_t g_last_clk_ms[NUM_MOTORS] = {0};

static void IRAM_ATTR isr_button_extend(void *arg) {
    (void)arg;
    g_extend_event = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_EXTEND_PIN) == 0);
}

static void IRAM_ATTR isr_button_retract(void *arg) {
    (void)arg;
    g_retract_event = (gpio_ll_get_level(&GPIO, (gpio_num_t)BUTTON_RETRACT_PIN) == 0);
}

static void IRAM_ATTR isr_hal_clk(void *arg) {
    int idx = (int)(intptr_t)arg;
    uint32_t now_ms = (uint32_t)((uint64_t)esp_timer_get_time() / 1000ULL);

    if ((unsigned)idx >= (unsigned)g_num_motors) {
        return;
    }

    // Arduino logic:
    // if (millis() - lastClk[idx] > HALL_FILTER_MS) {
    //   if (digitalRead(HAL_CNT[idx]) == 0) steps_in++, pos-- else steps_out++, pos++;
    //   lastClk[idx] = millis();
    // }
    uint32_t last_ms = g_last_clk_ms[idx];
    uint32_t delta_ms = now_ms - last_ms;
    if (last_ms != 0 && delta_ms <= HALL_FILTER_MS) {
        return;
    }

    // Direction is sampled directly from HAL_CNT.
    int cnt = gpio_ll_get_level(&GPIO, (gpio_num_t)(HAL_CNT[idx]));
    if (cnt == 0) {
        g_motors[idx].incrementStepIn();
    } else {
        g_motors[idx].incrementStepOut();
    }
    g_last_clk_ms[idx] = now_ms;
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
    // Reset software edge filter baselines.
    for (int i = 0; i < g_num_motors; ++i) {
        g_last_clk_ms[i] = 0;
    }

    gpio_config_t clk = {};
    clk.mode = GPIO_MODE_INPUT;
    clk.intr_type = GPIO_INTR_POSEDGE;
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

    for (int i = 0; i < g_num_motors; ++i) {
        unsigned int clk_pin = HAL_CLK[i];
        unsigned int sample_pin = HAL_CNT[i];
        ESP_LOGI(TAG,
                 "HAL m%d: CLK=%u SAMPLE=%u hall_filter_ms=%u",
                 i,
                 clk_pin,
                 sample_pin,
                 (unsigned)HALL_FILTER_MS);
    }


    for (int i = 0; i < g_num_motors; ++i) {
        unsigned int clk_pin = HAL_CLK[i];
        gpio_isr_handler_add((gpio_num_t)clk_pin, isr_hal_clk, (void *)(intptr_t)i);
    }
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
