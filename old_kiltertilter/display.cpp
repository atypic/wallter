#include "display.hpp"
#include "Wire.h"
#include "util.hpp"

Display::Display() : lcd() {
    last_update_ms      = 0;
    short_view_time_ms  = 0;
    current_view        = LCD_TARGET_VIEW;
    next_view           = LCD_TARGET_VIEW;
    pending_refresh     = false;
    refresh_interval_ms = REFRESH_INTERVAL;
}

void Display::init() {
    lcd.begin(16, 2);
    Wire.setTimeout(3000, false);
    lcd.setRGB(LCD_BACKLIGHT_R, LCD_BACKLIGHT_G, LCD_BACKLIGHT_B);
    lcd.clear();
}
void Display::clear(void) {
    lcd.clear();
}

void Display::print(const char *message, const char *line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void Display::set_next_view(display_views_t view) {
    next_view = view;
}

void Display::set_view(display_views_t view) {
    // Do not reinitialize or reset backlight on every view switch to avoid flicker.
    current_view = view;
    // Request immediate refresh when view changes
    pending_refresh = true;
}

void Display::show_short_message(char *line1, char *line2, uint32_t time_ms) {
    // If we are currently showing a short message,
    // do not interrupt it.
    if (current_view != LCD_SHORT_MESSAGE_VIEW) {
        next_view = current_view;
        set_view(LCD_SHORT_MESSAGE_VIEW);
        (void)strncpy(view_buffers[LCD_SHORT_MESSAGE_VIEW][0], line1, 16);
        (void)strncpy(view_buffers[LCD_SHORT_MESSAGE_VIEW][1], line2, 16);
        short_view_time_ms = time_ms;
    }
    // Force a refresh soon
    pending_refresh = true;
}

void Display::update_target_view(float target_angle, uint8_t percent) {
    // Top row: target angle
    snprintf(view_buffers[LCD_TARGET_VIEW][0], 16, "Target:  %.1f", target_angle);

    // Bottom row: progress bar [########------]
    // 16 chars total: '[' at 0, ']' at 15, 14 inner cells
    const int innerWidth = 14;
    uint8_t clamped      = percent > 100 ? 100 : percent;
    // Round up slightly so near-100 values visibly fill the bar; force full on 100
    uint8_t filledCells =
        (clamped == 100) ? innerWidth : (uint8_t)((clamped * innerWidth + 99) / 100);
    view_buffers[LCD_TARGET_VIEW][1][0] = '[';
    for (int i = 1; i <= innerWidth; ++i) {
        view_buffers[LCD_TARGET_VIEW][1][i] = (i <= filledCells) ? '#' : ' ';
    }
    view_buffers[LCD_TARGET_VIEW][1][15] = ']';
}

void Display::update_homing_view() {
    snprintf(view_buffers[LCD_HOMING_VIEW][0], 16, "Wall going up.");
    snprintf(view_buffers[LCD_HOMING_VIEW][1], 16, "Please take care.");
}

void Display::update_manual_view(long pos_ticks) {
    snprintf(view_buffers[LCD_MANUAL_VIEW][0], 16, "Manual mode");
    snprintf(view_buffers[LCD_MANUAL_VIEW][1], 16, "Pos:%ld", pos_ticks);
}
void Display::update_lockscreen_view(char *current_pin) {
    (void)snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][0], 16, "ARCTIC GRIPS ");
    (void)snprintf(view_buffers[LCD_LOCKSCREEN_VIEW][1], 16, "PIN? -> %s", current_pin);
}

void Display::refresh() {
    unsigned long now = millis();

    if (pending_refresh || throttle(last_update_ms, refresh_interval_ms)) {
        // Check if we are showing the short message.
        // If we are done, go to next view.
        if ((short_view_time_ms <= 0) && (current_view == LCD_SHORT_MESSAGE_VIEW)) {
            current_view = next_view;
        } else {
            short_view_time_ms -= (int32_t)refresh_interval_ms;
        }

        //lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(view_buffers[current_view][0]);
        lcd.setCursor(0, 1);
        lcd.print(view_buffers[current_view][1]);
        // last_update_ms updated by throttle()
        pending_refresh = false;
    }
}

void Display::update_new_pin_view(char *new_pin) {
    (void)snprintf(view_buffers[LCD_NEW_PIN_VIEW][0], 16, "New PIN: %s", new_pin);
}

void Display::trigger_refresh() {
    pending_refresh = true;
}

void Display::cycle_colors() {
    // Avoid forcing backlight changes during normal operation; leave as no-op.
}

void Display::startup_animation() {
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_SLUPPEN_PROTO)
    // No RGB backlight; simple scroll only.
#endif
    const char *line1      = "ARCTIC";
    const char *line2      = "GRIPS";
    const int len1         = (int)strlen(line1);
    const int len2         = (int)strlen(line2);
    const uint32_t total_ms = 5000; // 5 seconds
    const uint32_t frame_ms = 60;   // ~60ms per frame
    const int frames        = (int)(total_ms / frame_ms);

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

        // Compute scrolling positions:
        // Top line scrolls left -> right, starting off-screen left to off-screen right
        // Bottom line scrolls right -> left, starting off-screen right to off-screen left
        int col1 = -len1 + (int)((16 + len1) * p);       // from -len1 to 16
        int col2 = 16 - (int)((16 + len2) * p);          // from 16 to -len2

        // Draw line 1
        lcd.setCursor(0, 0);
        for (int i = 0; i < 16; ++i) {
            lcd.print(' ');
        }
        if (col1 < 16 && col1 > -len1) {
            int start = max(0, col1);
            int offset = max(0, -col1);
            int available = 16 - start;
            int count = min(len1 - offset, available);
            if (count > 0) {
                lcd.setCursor(start, 0);
                for (int i = 0; i < count; ++i) {
                    lcd.print(line1[offset + i]);
                }
            }
        }

        // Draw line 2
        lcd.setCursor(0, 1);
        for (int i = 0; i < 16; ++i) {
            lcd.print(' ');
        }
        if (col2 < 16 && col2 > -len2) {
            int start = max(0, col2);
            int offset = max(0, -col2);
            int available = 16 - start;
            int count = min(len2 - offset, available);
            if (count > 0) {
                lcd.setCursor(start, 1);
                for (int i = 0; i < count; ++i) {
                    lcd.print(line2[offset + i]);
                }
            }
        }
        delay(frame_ms);
    }
    lcd.clear();
}

void Display::set_refresh_rate(float seconds) {
    if (seconds <= 0.0f) {
        // Minimum interval 50ms to avoid busy redraws
        refresh_interval_ms = 200;
    } else {
        refresh_interval_ms = (uint32_t)(seconds * 1000.0f);
    }
}

void Display::show_panic(const char *line2) {
    // Force the short message view with a persistent PANIC banner
    next_view = LCD_SHORT_MESSAGE_VIEW;
    set_view(LCD_SHORT_MESSAGE_VIEW);
    (void)strncpy(view_buffers[LCD_SHORT_MESSAGE_VIEW][0], "PANIC", 16);
    (void)strncpy(view_buffers[LCD_SHORT_MESSAGE_VIEW][1], line2, 16);
    // Hold effectively forever; refresh() decrements by refresh_interval_ms
    short_view_time_ms = 0x3fffffff; // large positive
    pending_refresh    = true;
}