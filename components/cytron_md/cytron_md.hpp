#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

enum CytronMode { CYTRON_PWM_DIR, CYTRON_PWM_PWM };

class CytronMD {
public:
  // Constructor: provide mode and pins
  CytronMD(CytronMode mode,
           gpio_num_t pin1,
           gpio_num_t pin2,
           ledc_channel_t ch1 = LEDC_CHANNEL_0,
           ledc_channel_t ch2 = LEDC_CHANNEL_1);

  // Initialize GPIO + LEDC. Provide LEDC timer, frequency, resolution (bits).
  esp_err_t init(ledc_timer_t timer = LEDC_TIMER_0,
                 ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE,
                 uint32_t freq_hz = 20000,
                 ledc_timer_bit_t duty_resolution = LEDC_TIMER_8_BIT);

  // Set speed in range [-255, 255]
  esp_err_t setSpeed(int16_t speed);

private:
  CytronMode mode_;
  gpio_num_t pin1_;
  gpio_num_t pin2_;

  ledc_mode_t speed_mode_ = LEDC_LOW_SPEED_MODE;
  ledc_timer_t timer_ = LEDC_TIMER_0;
  ledc_timer_bit_t resolution_ = LEDC_TIMER_8_BIT;
  uint32_t freq_hz_ = 20000;

  // LEDC channels used (only ch1 used for PWM_DIR; both for PWM_PWM)
  ledc_channel_t ch1_;
  ledc_channel_t ch2_;

  bool initialized_ = false;

  esp_err_t setup_gpio();
  esp_err_t setup_ledc_channels();
  uint32_t clampDuty(int16_t speed) const;
};
