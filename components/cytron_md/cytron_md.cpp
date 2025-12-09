#include "cytron_md.hpp"
#include "esp_check.h"
#include "esp_log.h"

CytronMD::CytronMD(CytronMode mode, gpio_num_t pin1, gpio_num_t pin2)
  : mode_(mode), pin1_(pin1), pin2_(pin2) {}

esp_err_t CytronMD::init(ledc_timer_t timer,
                         ledc_mode_t speed_mode,
                         uint32_t freq_hz,
                         ledc_timer_bit_t duty_resolution) {
  timer_ = timer;
  speed_mode_ = speed_mode;
  freq_hz_ = freq_hz;
  resolution_ = duty_resolution;

  // Configure LEDC timer
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode = speed_mode_;
  tcfg.timer_num = timer_;
  tcfg.freq_hz = freq_hz_;
  tcfg.clk_cfg = LEDC_AUTO_CLK;
  tcfg.duty_resolution = resolution_;
  ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), "CytronMD", "timer_config");

  ESP_RETURN_ON_ERROR(setup_gpio(), "CytronMD", "gpio_setup");
  ESP_RETURN_ON_ERROR(setup_ledc_channels(), "CytronMD", "ledc_channels");

  initialized_ = true;
  // Default outputs low
  gpio_set_level(pin1_, 0);
  gpio_set_level(pin2_, 0);
  return ESP_OK;
}

esp_err_t CytronMD::setup_gpio() {
  gpio_config_t io = {};
  io.mode = GPIO_MODE_OUTPUT;
  io.intr_type = GPIO_INTR_DISABLE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;

  io.pin_bit_mask = (1ULL << pin1_) | (1ULL << pin2_);
  return gpio_config(&io);
}

esp_err_t CytronMD::setup_ledc_channels() {
  // In PWM_DIR, pin1 is PWM (LEDC), pin2 is direction (GPIO)
  // In PWM_PWM, both pins are PWM channels.
  ledc_channel_config_t c1 = {};
  c1.gpio_num = pin1_;
  c1.speed_mode = speed_mode_;
  c1.channel = ch1_;
  c1.intr_type = LEDC_INTR_DISABLE;
  c1.timer_sel = timer_;
  c1.duty = 0;
  c1.hpoint = 0;
  ESP_RETURN_ON_ERROR(ledc_channel_config(&c1), "CytronMD", "ch1_config");

  if (mode_ == CYTRON_PWM_PWM) {
    ledc_channel_config_t c2 = {};
    c2.gpio_num = pin2_;
    c2.speed_mode = speed_mode_;
    c2.channel = ch2_;
    c2.intr_type = LEDC_INTR_DISABLE;
    c2.timer_sel = timer_;
    c2.duty = 0;
    c2.hpoint = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&c2), "CytronMD", "ch2_config");
  }
  return ESP_OK;
}

uint32_t CytronMD::clampDuty(int16_t speed) const {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;
  return (uint32_t)(speed >= 0 ? speed : -speed);
}

esp_err_t CytronMD::setSpeed(int16_t speed) {
  if (!initialized_) return ESP_ERR_INVALID_STATE;
  uint32_t duty = clampDuty(speed);

  switch (mode_) {
    case CYTRON_PWM_DIR:
      // pin1: PWM duty; pin2: direction level
      ESP_RETURN_ON_ERROR(ledc_set_duty(speed_mode_, ch1_, duty), "CytronMD", "set_duty");
      ESP_RETURN_ON_ERROR(ledc_update_duty(speed_mode_, ch1_), "CytronMD", "update_duty");
      gpio_set_level(pin2_, (speed >= 0) ? 0 : 1);
      break;

    case CYTRON_PWM_PWM:
      if (speed >= 0) {
        ESP_RETURN_ON_ERROR(ledc_set_duty(speed_mode_, ch1_, duty), "CytronMD", "set_duty_ch1");
        ESP_RETURN_ON_ERROR(ledc_update_duty(speed_mode_, ch1_), "CytronMD", "update_duty_ch1");
        ESP_RETURN_ON_ERROR(ledc_set_duty(speed_mode_, ch2_, 0), "CytronMD", "set_duty_ch2_zero");
        ESP_RETURN_ON_ERROR(ledc_update_duty(speed_mode_, ch2_), "CytronMD", "update_duty_ch2_zero");
      } else {
        ESP_RETURN_ON_ERROR(ledc_set_duty(speed_mode_, ch1_, 0), "CytronMD", "set_duty_ch1_zero");
        ESP_RETURN_ON_ERROR(ledc_update_duty(speed_mode_, ch1_), "CytronMD", "update_duty_ch1_zero");
        ESP_RETURN_ON_ERROR(ledc_set_duty(speed_mode_, ch2_, duty), "CytronMD", "set_duty_ch2");
        ESP_RETURN_ON_ERROR(ledc_update_duty(speed_mode_, ch2_), "CytronMD", "update_duty_ch2");
      }
      break;
  }

  return ESP_OK;
}
