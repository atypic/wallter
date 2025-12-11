#define BOARD_TYPE BOARD_TYPE_ARCTIC_CYTRON
#include "display.hpp"
#include "boards.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Display::Display()
    : lcd(),
    current_view(LCD_HOMING_VIEW),
    next_view(LCD_HOMING_VIEW),
    previous_view(LCD_HOMING_VIEW),
    last_update_ms(0),
    refresh_interval_ms(REFRESH_INTERVAL),
    short_view_time_ms(0),
    short_view_start_ms(0),
    pending_refresh(false) {}

void Display::init(void) {
    // Mirror the known-working demo exactly: pins, timing, params
    lcd.setRGB(LCD_BACKLIGHT_R, LCD_BACKLIGHT_G, LCD_BACKLIGHT_B);
    vTaskDelay(pdMS_TO_TICKS(1000));
    const gpio_num_t SDA = GPIO_NUM_7;
    const gpio_num_t SCL = GPIO_NUM_6;
    if (lcd.init_i2c(I2C_NUM_0, SDA, SCL, 100000) != ESP_OK) {
        ESP_LOGE("Display", "I2C init failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (lcd.begin(16, 2, 0) != ESP_OK) {
        ESP_LOGE("Display", "LCD begin failed");
        return;
    }
    lcd.clear();
}

void Display::print(const char *message, const char *line2) {
    // Clear may NACK if bus is busy; retry briefly
    esp_err_t err = lcd.clear();
    if (err != ESP_OK) {
        ESP_LOGW("Display", "lcd.clear failed (%d), retrying", (int)err);
        vTaskDelay(pdMS_TO_TICKS(10));
        err = lcd.clear();
        if (err != ESP_OK) {
            ESP_LOGW("Display", "lcd.clear second attempt failed (%d)", (int)err);
        }
    }
    lcd.setCursor(0, 0);
    // Ensure strings are limited to 16 characters
    char l1[17];
    char l2[17];
    snprintf(l1, sizeof(l1), "%.*s", 16, message ? message : "");
    snprintf(l2, sizeof(l2), "%.*s", 16, line2 ? line2 : "");
    lcd.print(l1);
    lcd.setCursor(0, 1);
    lcd.print(l2);
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
    // Respect refresh interval unless explicitly triggered
    if (!pending_refresh && (now - last_update_ms) < refresh_interval_ms) return;
    pending_refresh = false;
    last_update_ms = now;

    // Render current_view from buffers if present
    // Handle short message countdown like original Arduino code
    if (current_view == LCD_SHORT_MESSAGE_VIEW) {
        if (short_view_time_ms <= 0) {
            current_view = next_view;
        } else {
            short_view_time_ms -= (int32_t)refresh_interval_ms;
        }
    }

    // Switch to next_view if queued
    if (current_view != next_view) {
        current_view = next_view;
    }
    print(view_buffers[current_view][0], view_buffers[current_view][1]);
}

void Display::set_refresh_rate(float seconds) { refresh_interval_ms = (uint32_t)(seconds * 1000.0f); }
void Display::trigger_refresh() { pending_refresh = true; }

void Display::update_new_pin_view(char *new_pin) {
    snprintf(view_buffers[LCD_NEW_PIN_VIEW][0], 17, "%.*s", 16, "New PIN:");
    snprintf(view_buffers[LCD_NEW_PIN_VIEW][1], 17, "%.*s", 16, new_pin);
    // Do not force immediate refresh; respect refresh interval
    next_view = LCD_NEW_PIN_VIEW;
}

void Display::update_target_view(float target_angle, uint8_t percent) {
    // Top row: target angle, formatted similar to Arduino version
    snprintf(view_buffers[LCD_TARGET_VIEW][0], 17, "Target:  %.1f", target_angle);
    // Bottom row: progress bar [########------] width 16 (brackets + 14 cells)
    const int innerWidth = 14;
    uint8_t clamped = percent > 100 ? 100 : percent;
    uint8_t filledCells = (clamped == 100) ? innerWidth : (uint8_t)((clamped * innerWidth + 99) / 100);
    // Build bar in-place
    view_buffers[LCD_TARGET_VIEW][1][0] = '[';
    for (int i = 1; i <= innerWidth; ++i) {
        view_buffers[LCD_TARGET_VIEW][1][i] = (i <= filledCells) ? '#' : ' ';
    }
    view_buffers[LCD_TARGET_VIEW][1][15] = ']';
    view_buffers[LCD_TARGET_VIEW][1][16] = '\0';
    // Do not set_view here to avoid forcing refresh each loop
    next_view = LCD_TARGET_VIEW;
}

void Display::update_lockscreen_view(char *current_pin) {
    snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][0], 17, "%.*s", 16, "LOCKED");
    snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][1], 17, "PIN:%.*s", 12, current_pin);
    next_view = LCD_LOCKSCREEN_VIEW;
}

void Display::update_homing_view() {
    snprintf(view_buffers[LCD_HOMING_VIEW][0], 17, "%.*s", 16, "Homing...");
    snprintf(view_buffers[LCD_HOMING_VIEW][1], 17, "%.*s", 16, "Please wait");
    next_view = LCD_HOMING_VIEW;
}

void Display::update_manual_view(long pos_ticks) {
    snprintf(view_buffers[LCD_MANUAL_VIEW][0], 17, "%.*s", 16, "Manual");
    snprintf(view_buffers[LCD_MANUAL_VIEW][1], 17, "Pos:%ld", pos_ticks);
    next_view = LCD_MANUAL_VIEW;
}

void Display::show_short_message(char *line1, char *line2, uint32_t time_ms) {
    snprintf(view_buffers[LCD_SHORT_MESSAGE_VIEW][0], 17, "%.*s", 16, line1);
    snprintf(view_buffers[LCD_SHORT_MESSAGE_VIEW][1], 17, "%.*s", 16, line2);
    short_view_time_ms = (int32_t)time_ms;
    // Remember current view to return after short message
    previous_view = current_view;
    set_next_view(previous_view);
    set_view(LCD_SHORT_MESSAGE_VIEW);
}

void Display::cycle_colors() {
    static uint8_t r = LCD_BACKLIGHT_R, g = LCD_BACKLIGHT_G, b = LCD_BACKLIGHT_B;
    r = (uint8_t)((r + 32) % 256);
    g = (uint8_t)((g + 32) % 256);
    b = (uint8_t)((b + 32) % 256);
    lcd.setRGB(r, g, b);
}

void Display::startup_animation() {
    const char *line1 = "ARCTIC";
    const char *line2 = "GRIPS";
    const int len1 = (int)strlen(line1);
    const int len2 = (int)strlen(line2);
    const uint32_t total_ms = 5000; // 5 seconds
    const uint32_t frame_ms = 60;   // ~60ms per frame
    const int frames = (int)(total_ms / frame_ms);

    // Ensure clean screen
    lcd.clear();
    // Backlight progression setup
    uint8_t targetR = LCD_BACKLIGHT_R;
    uint8_t targetG = LCD_BACKLIGHT_G;
    uint8_t targetB = LCD_BACKLIGHT_B;
    uint8_t curR = 0, curG = 0, curB = 0;
    lcd.setRGB(0, 0, 0);

    for (int f = 0; f < frames; ++f) {
        float p = (float)f / (float)frames;
        if (p < 0.25f) {
            curR = (uint8_t)(255.0f * (p / 0.25f));
            curG = 0;
            curB = 0;
        } else {
            float t = (p - 0.25f) / 0.75f;
            curR = (uint8_t)(curR + (int)((targetR - curR) * t));
            curG = (uint8_t)(curG + (int)((targetG - curG) * t));
            curB = (uint8_t)(curB + (int)((targetB - curB) * t));
        }
        lcd.setRGB(curR, curG, curB);

        // Compute scrolling positions
        int col1 = -len1 + (int)((16 + len1) * p); // from -len1 to 16
        int col2 = 16 - (int)((16 + len2) * p);    // from 16 to -len2

        // Draw line 1
        lcd.setCursor(0, 0);
        for (int i = 0; i < 16; ++i) {
            char sp[2] = {' ', '\0'};
            lcd.print(sp);
        }
        if (col1 < 16 && col1 > -len1) {
            int start = col1 < 0 ? 0 : col1;
            int offset = col1 < 0 ? -col1 : 0;
            int available = 16 - start;
            int count = (len1 - offset) < available ? (len1 - offset) : available;
            if (count > 0) {
                lcd.setCursor(start, 0);
                for (int i = 0; i < count; ++i) {
                    char ch[2] = { line1[offset + i], '\0' };
                    lcd.print(ch);
                }
            }
        }

        // Draw line 2
        lcd.setCursor(0, 1);
        for (int i = 0; i < 16; ++i) {
            char sp[2] = {' ', '\0'};
            lcd.print(sp);
        }
        if (col2 < 16 && col2 > -len2) {
            int start = col2 < 0 ? 0 : col2;
            int offset = col2 < 0 ? -col2 : 0;
            int available = 16 - start;
            int count = (len2 - offset) < available ? (len2 - offset) : available;
            if (count > 0) {
                lcd.setCursor(start, 1);
                for (int i = 0; i < count; ++i) {
                    char ch[2] = { line2[offset + i], '\0' };
                    lcd.print(ch);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(frame_ms));
    }
    lcd.clear();
}
