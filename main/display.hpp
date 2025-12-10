#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include "boards.h"
#include <stdint.h>

#include "rgb_lcd_idf.hpp"  // from espidf_lcd component

#define REFRESH_INTERVAL 5000

#define LCD_BACKLIGHT_R 0
#define LCD_BACKLIGHT_G 128
#define LCD_BACKLIGHT_B 255

typedef enum display_views {
    LCD_HOMING_VIEW,
    LCD_TARGET_VIEW,
    LCD_SHORT_MESSAGE_VIEW,
    LCD_LOCKSCREEN_VIEW,
    LCD_MANUAL_VIEW,
    LCD_NEW_PIN_VIEW,
    LCD_NUM_VIEWS
} display_views_t;

class Display {
  public:
    Display();
    void init(void);
    void print(const char *message, const char *line2);
    void clear(void);
    void set_view(display_views_t view);
    void set_next_view(display_views_t view);

    void refresh();
    void set_refresh_rate(float seconds);
    void trigger_refresh();

    void update_new_pin_view(char *new_pin);
    void update_target_view(float target_angle, uint8_t percent);
    void update_lockscreen_view(char *current_pin);
    void update_homing_view();
    void update_manual_view(long pos_ticks);
    void show_short_message(char *line1, char *line2, uint32_t time_ms);
    void cycle_colors();

  private:
    rgb_lcd_idf lcd;
    display_views_t current_view;
    display_views_t next_view;
    uint32_t last_update_ms;
    uint32_t refresh_interval_ms;
    int32_t short_view_time_ms;
    bool pending_refresh;
    char view_buffers[LCD_NUM_VIEWS][2][16];
};

#endif  // DISPLAY_HPP
