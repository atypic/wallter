#include "demo.hpp"
#include "cytron_md.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "cytron_demo";

extern "C" void cytron_md_demo(void) {
  // Update pins to your board wiring
  const gpio_num_t PWM_PIN = GPIO_NUM_3;
  const gpio_num_t DIR_PIN = GPIO_NUM_8;

  CytronMD md(CYTRON_PWM_DIR, PWM_PIN, DIR_PIN);
  if (md.init(LEDC_TIMER_0, LEDC_LOW_SPEED_MODE, 20000, LEDC_TIMER_8_BIT) != ESP_OK) {
    ESP_LOGE(TAG, "Motor init failed");
    return;
  }

  // Ramp forward
  for (int s = 0; s <= 255; s += 51) {
    md.setSpeed(s);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  // Hold
  vTaskDelay(pdMS_TO_TICKS(500));
  // Ramp reverse
  for (int s = 0; s >= -255; s -= 51) {
    md.setSpeed(s);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  // Stop
  md.setSpeed(0);
}
