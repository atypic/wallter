#include "display.hpp"
#include "esp_timer.h"

Display::Display()
    : lcd(),
    current_view(LCD_HOMING_VIEW),
    next_view(LCD_HOMING_VIEW),
    last_update_ms(0),
    refresh_interval_ms(REFRESH_INTERVAL),
    short_view_time_ms(0),
    pending_refresh(false) {}

void Display::init(void) {
    // Basic I2C init on default pins (adjust as needed)
    lcd.init_i2c(I2C_NUM_0, GPIO_NUM_13, GPIO_NUM_14, 100000);
    lcd.begin(16, 2, LCD_5x8DOTS);
    lcd.setRGB(LCD_BACKLIGHT_R, LCD_BACKLIGHT_G, LCD_BACKLIGHT_B);
    lcd.clear();
}

void Display::print(const char *message, const char *line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message ? message : "");
    lcd.setCursor(0, 1);
    lcd.print(line2 ? line2 : "");
}

void Display::clear(void) { lcd.clear(); }

void Display::set_view(display_views_t view) {
    current_view = view;
    pending_refresh = true;
}

void Display::set_next_view(display_views_t view) { next_view = view; }

void Display::refresh() {
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (last_update_ms == 0) last_update_ms = now;
    if (!pending_refresh && (now - last_update_ms) < refresh_interval_ms) return;
    pending_refresh = false;
    last_update_ms = now;

    // Render current_view from buffers if present
    const char *l1 = view_buffers[current_view][0];
    const char *l2 = view_buffers[current_view][1];
    print(l1, l2);
}

void Display::set_refresh_rate(float seconds) { refresh_interval_ms = (uint32_t)(seconds * 1000.0f); }
void Display::trigger_refresh() { pending_refresh = true; }

void Display::update_new_pin_view(char *new_pin) {
    snprintf(view_buffers[LCD_NEW_PIN_VIEW][0], 16, "New PIN:");
    snprintf(view_buffers[LCD_NEW_PIN_VIEW][1], 16, "%s", new_pin);
    set_view(LCD_NEW_PIN_VIEW);
}

void Display::update_target_view(float target_angle, uint8_t percent) {
    snprintf(view_buffers[LCD_TARGET_VIEW][0], 16, "Target:%3.0fdeg", target_angle);
    snprintf(view_buffers[LCD_TARGET_VIEW][1], 16, "Pct:%3u%%", (unsigned)percent);
    set_view(LCD_TARGET_VIEW);
}

void Display::update_lockscreen_view(char *current_pin) {
    snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][0], 16, "LOCKED");
    snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][1], 16, "PIN:%s", current_pin);
    set_view(LCD_LOCKSCREEN_VIEW);
}

void Display::update_homing_view() {
    snprintf(view_buffers[LCD_HOMING_VIEW][0], 16, "Homing...");
    snprintf(view_buffers[LCD_HOMING_VIEW][1], 16, "Please wait");
    set_view(LCD_HOMING_VIEW);
}

void Display::update_manual_view(long pos_ticks) {
    snprintf(view_buffers[LCD_MANUAL_VIEW][0], 16, "Manual");
    snprintf(view_buffers[LCD_MANUAL_VIEW][1], 16, "Pos:%ld", pos_ticks);
    set_view(LCD_MANUAL_VIEW);
}

void Display::show_short_message(char *line1, char *line2, uint32_t time_ms) {
    snprintf(view_buffers[LCD_SHORT_MESSAGE_VIEW][0], 16, "%s", line1);
    snprintf(view_buffers[LCD_SHORT_MESSAGE_VIEW][1], 16, "%s", line2);
    short_view_time_ms = (int32_t)time_ms;
    set_view(LCD_SHORT_MESSAGE_VIEW);
}

void Display::cycle_colors() {
    static uint8_t r = LCD_BACKLIGHT_R, g = LCD_BACKLIGHT_G, b = LCD_BACKLIGHT_B;
    r = (uint8_t)((r + 32) % 256);
    g = (uint8_t)((g + 32) % 256);
    b = (uint8_t)((b + 32) % 256);
    lcd.setRGB(r, g, b);
}
