#include "inputs.hpp"

#include "app_config.hpp"
#include "motordriver.hpp"

#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "hal/gpio_ll.h"

namespace wallter::inputs {

static MotorDriver *g_motors = nullptr;
static int g_num_motors = 0;

static gpio_glitch_filter_handle_t g_btn_extend_filter = nullptr;
static gpio_glitch_filter_handle_t g_btn_retract_filter = nullptr;
static gpio_glitch_filter_handle_t g_hal_clk_filter[NUM_MOTORS] [[maybe_unused]] = {};

static volatile bool g_extend_event = false;
static volatile bool g_retract_event = false;
static bool g_boot_menu_requested = false;

// Software edge filter for HAL clocks (aggressive debounce)
static uint64_t g_last_clk_us[NUM_MOTORS] = {0};
static const uint32_t HAL_CLK_FILTER_US = 80; // ignore edges within 80us

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
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    uint64_t last_us = g_last_clk_us[idx];
    if (last_us != 0 && (now_us - last_us) < HAL_CLK_FILTER_US) {
        return;
    }
    g_last_clk_us[idx] = now_us;

    int cnt = gpio_ll_get_level(&GPIO, (gpio_num_t)HAL_CNT[idx]);
    if (cnt == 0) {
        g_motors[idx].incrementStepIn();
    } else {
        g_motors[idx].incrementStepOut();
    }
}

static void configure_gpio() {
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

    gpio_config_t clk = {};
    clk.mode = GPIO_MODE_INPUT;
    clk.intr_type = GPIO_INTR_POSEDGE;
    clk.pull_up_en = GPIO_PULLUP_ENABLE;
    clk.pull_down_en = GPIO_PULLDOWN_DISABLE;
    uint64_t mask = 0;
    for (int i = 0; i < g_num_motors; ++i) {
        mask |= (1ULL << HAL_CLK[i]);
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
        mask |= (1ULL << HAL_CNT[i]);
    }
    cnt.pin_bit_mask = mask;
    gpio_config(&cnt);

    for (int i = 0; i < g_num_motors; ++i) {
        gpio_isr_handler_add((gpio_num_t)HAL_CLK[i], isr_hal_clk, (void *)(intptr_t)i);
    }
}

void init(MotorDriver *motors, int num_motors) {
    g_motors = motors;
    g_num_motors = num_motors;

    configure_gpio();

    if (g_btn_extend_filter) {
        gpio_glitch_filter_enable(g_btn_extend_filter);
    }
    if (g_btn_retract_filter) {
        gpio_glitch_filter_enable(g_btn_retract_filter);
    }

    g_boot_menu_requested = (gpio_get_level((gpio_num_t)BUTTON_EXTEND_PIN) == 0);
    if (g_boot_menu_requested) {
        g_extend_event = true;
    }
    if (gpio_get_level((gpio_num_t)BUTTON_RETRACT_PIN) == 0) {
        g_retract_event = true;
    }
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

void clear_boot_menu_requested() {
    g_boot_menu_requested = false;
}

} // namespace wallter::inputs
