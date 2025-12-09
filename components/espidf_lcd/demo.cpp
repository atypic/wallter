#include "rgb_lcd_idf.hpp"
#include "demo.hpp"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lcd_demo";

extern "C" void espidf_lcd_demo(void) {
  // Configure I2C pins (update these to your board)
  const gpio_num_t SDA = GPIO_NUM_13;
  const gpio_num_t SCL = GPIO_NUM_14;

  rgb_lcd_idf lcd;
  if (lcd.init_i2c(I2C_NUM_0, SDA, SCL, 100000) != ESP_OK) {
    ESP_LOGE(TAG, "I2C init failed");
    return;
  }
  if (lcd.begin(16, 2, 0) != ESP_OK) {
    ESP_LOGE(TAG, "LCD begin failed");
    return;
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  const char *msg1 = "00Hello, Wallter!";
  for (const char *p = msg1; *p; ++p) lcd.write((uint8_t)*p);

  lcd.setCursor(0, 1);
  const char *msg2 = "RGB LCD demo";
  for (const char *p = msg2; *p; ++p) lcd.write((uint8_t)*p);

  lcd.setRGB(0, 128, 255); // soft blue backlight

  // Keep task alive briefly to show output
  vTaskDelay(pdMS_TO_TICKS(2000));
}
