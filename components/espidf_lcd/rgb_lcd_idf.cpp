#include "rgb_lcd_idf.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rgb_lcd_idf";

static esp_err_t probe_i2c_address(i2c_port_t port, uint8_t addr, TickType_t timeout_ticks) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  esp_err_t err = i2c_master_start(cmd);
  if (err == ESP_OK) err = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  if (err == ESP_OK) err = i2c_master_stop(cmd);
  if (err == ESP_OK) err = i2c_master_cmd_begin(port, cmd, timeout_ticks);
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t rgb_lcd_idf::init_i2c(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz) {
  port_ = port;
  i2c_config_t cfg{};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = sda;
  cfg.scl_io_num = scl;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.master.clk_speed = clk_hz;
#if SOC_I2C_SUPPORT_XTAL
  cfg.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
#endif
  ESP_RETURN_ON_ERROR(i2c_param_config(port_, &cfg), TAG, "param_config");
  ESP_RETURN_ON_ERROR(i2c_driver_install(port_, cfg.mode, 0, 0, 0), TAG, "driver_install");
  initialized_ = true;
  return ESP_OK;
}

esp_err_t rgb_lcd_idf::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (!initialized_) return ESP_ERR_INVALID_STATE;

  if (lines > 1) _displayfunction |= LCD_2LINE;
  _numlines = lines;
  _currline = 0;

  if ((dotsize != 0) && (lines == 1)) _displayfunction |= LCD_5x10DOTS;

  // wait 50ms after power-up
  vTaskDelay(pdMS_TO_TICKS(50));

  // Function set sequence (per HD44780)
  ESP_RETURN_ON_ERROR(command(LCD_FUNCTIONSET | _displayfunction), TAG, "funcset1");
  vTaskDelay(pdMS_TO_TICKS(5));
  ESP_RETURN_ON_ERROR(command(LCD_FUNCTIONSET | _displayfunction), TAG, "funcset2");
  vTaskDelay(pdMS_TO_TICKS(2));
  ESP_RETURN_ON_ERROR(command(LCD_FUNCTIONSET | _displayfunction), TAG, "funcset3");
  ESP_RETURN_ON_ERROR(command(LCD_FUNCTIONSET | _displayfunction), TAG, "funcset4");

  _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  ESP_RETURN_ON_ERROR(display(), TAG, "display_on");
  ESP_RETURN_ON_ERROR(clear(), TAG, "clear");

  _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  ESP_RETURN_ON_ERROR(command(LCD_ENTRYMODESET | _displaymode), TAG, "entrymode");

  // Probe optional RGB backlight chip (some boards omit it)
  (void)probe_rgb_chip();

  // Init backlight depending on chip variant (if present)
  if (rgb_addr_ == 0) {
    ESP_LOGW(TAG, "RGB backlight chip not found; skipping backlight init");
  } else if (rgb_addr_ == RGB_ADDRESS_V5) {
    // reset chip
    ESP_RETURN_ON_ERROR(setReg(0x00, 0x07), TAG, "v5 reset");
    vTaskDelay(pdMS_TO_TICKS(1)); // ~200us
    // set all led always on
    ESP_RETURN_ON_ERROR(setReg(0x04, 0x15), TAG, "v5 mode");
  } else {
    ESP_RETURN_ON_ERROR(setReg(REG_MODE1, 0), TAG, "mode1");
    ESP_RETURN_ON_ERROR(setReg(REG_OUTPUT, 0xFF), TAG, "output");
    ESP_RETURN_ON_ERROR(setReg(REG_MODE2, 0x20), TAG, "mode2");
  }
  ESP_RETURN_ON_ERROR(setRGB(255, 255, 255), TAG, "setWhite");

  vTaskDelay(pdMS_TO_TICKS(10));
  return ESP_OK;
}

// High-level API
esp_err_t rgb_lcd_idf::clear() {
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_RETURN_ON_ERROR(command(LCD_CLEARDISPLAY), TAG, "clear");
  vTaskDelay(pdMS_TO_TICKS(20));
  return ESP_OK;
}
esp_err_t rgb_lcd_idf::home() {
  ESP_RETURN_ON_ERROR(command(LCD_RETURNHOME), TAG, "home");
  vTaskDelay(pdMS_TO_TICKS(2));
  return ESP_OK;
}
esp_err_t rgb_lcd_idf::setCursor(uint8_t col, uint8_t row) {
  uint8_t addr = (row == 0) ? (0x80 | col) : (0xC0 | col);
  uint8_t dta[2] = {0x80, addr};
  return i2c_send_bytes(LCD_ADDRESS, dta, sizeof(dta));
}
esp_err_t rgb_lcd_idf::noDisplay() {
  _displaycontrol &= ~LCD_DISPLAYON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::display() {
  _displaycontrol |= LCD_DISPLAYON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::noCursor() {
  _displaycontrol &= ~LCD_CURSORON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::cursor() {
  _displaycontrol |= LCD_CURSORON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::noBlink() {
  _displaycontrol &= ~LCD_BLINKON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::blink() {
  _displaycontrol |= LCD_BLINKON;
  return command(LCD_DISPLAYCONTROL | _displaycontrol);
}
esp_err_t rgb_lcd_idf::scrollDisplayLeft() {
  return command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
esp_err_t rgb_lcd_idf::scrollDisplayRight() {
  return command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}
esp_err_t rgb_lcd_idf::leftToRight() {
  _displaymode |= LCD_ENTRYLEFT;
  return command(LCD_ENTRYMODESET | _displaymode);
}
esp_err_t rgb_lcd_idf::rightToLeft() {
  _displaymode &= ~LCD_ENTRYLEFT;
  return command(LCD_ENTRYMODESET | _displaymode);
}
esp_err_t rgb_lcd_idf::autoscroll() {
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  return command(LCD_ENTRYMODESET | _displaymode);
}
esp_err_t rgb_lcd_idf::noAutoscroll() {
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  return command(LCD_ENTRYMODESET | _displaymode);
}
esp_err_t rgb_lcd_idf::createChar(uint8_t location, const uint8_t charmap[8]) {
  location &= 0x7;
  ESP_RETURN_ON_ERROR(command(LCD_SETCGRAMADDR | (location << 3)), TAG, "setCGRAM");
  uint8_t dta[9];
  dta[0] = 0x40;
  for (int i = 0; i < 8; ++i) dta[i + 1] = charmap[i];
  return i2c_send_bytes(LCD_ADDRESS, dta, sizeof(dta));
}

// Mid-level
esp_err_t rgb_lcd_idf::command(uint8_t value) {
  uint8_t dta[2] = {0x80, value};
  esp_err_t err = i2c_send_bytes(LCD_ADDRESS, dta, sizeof(dta));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "I2C cmd 0x%02X -> %s (%d)", value, esp_err_to_name(err), (int)err);
  }
  return err;
}
esp_err_t rgb_lcd_idf::write(uint8_t value) {
  uint8_t dta[2] = {0x40, value};
  return i2c_send_bytes(LCD_ADDRESS, dta, sizeof(dta));
}

esp_err_t rgb_lcd_idf::print(const char *str) {
  if (!str) return ESP_OK;
  while (*str) {
    ESP_RETURN_ON_ERROR(write((uint8_t)*str), TAG, "write");
    ++str;
  }
  return ESP_OK;
}

// RGB control
esp_err_t rgb_lcd_idf::setReg(uint8_t reg, uint8_t dat) {
  if (rgb_addr_ == 0) return ESP_OK;
  uint8_t dta[2] = {reg, dat};
  return i2c_send_bytes(rgb_addr_, dta, sizeof(dta));
}
esp_err_t rgb_lcd_idf::setRGB(uint8_t r, uint8_t g, uint8_t b) {
  if (rgb_addr_ == 0) return ESP_OK;
  if (rgb_addr_ == RGB_ADDRESS_V5) {
    ESP_RETURN_ON_ERROR(setReg(0x06, r), TAG, "v5 R");
    ESP_RETURN_ON_ERROR(setReg(0x07, g), TAG, "v5 G");
    ESP_RETURN_ON_ERROR(setReg(0x08, b), TAG, "v5 B");
  } else {
    ESP_RETURN_ON_ERROR(setReg(0x04, r), TAG, "R");
    ESP_RETURN_ON_ERROR(setReg(0x03, g), TAG, "G");
    ESP_RETURN_ON_ERROR(setReg(0x02, b), TAG, "B");
  }
  return ESP_OK;
}
esp_err_t rgb_lcd_idf::setPWM(Color c, uint8_t pwm) {
  switch (c) {
    case WHITE: return setRGB(pwm, pwm, pwm);
    case RED:   return setRGB(pwm, 0, 0);
    case GREEN: return setRGB(0, pwm, 0);
    case BLUE:  return setRGB(0, 0, pwm);
    default:    return ESP_OK;
  }
}
esp_err_t rgb_lcd_idf::setColor(Color c) {
  static const uint8_t colors[4][3] = {
    {255,255,255}, {255,0,0}, {0,255,0}, {0,0,255}
  };
  if (c > 3) return ESP_OK;
  return setRGB(colors[c][0], colors[c][1], colors[c][2]);
}

esp_err_t rgb_lcd_idf::blinkLED() {
  if (rgb_addr_ == RGB_ADDRESS_V5) {
    ESP_RETURN_ON_ERROR(setReg(0x04, 0x2a), TAG, "v5 attach pwm1");
    ESP_RETURN_ON_ERROR(setReg(0x01, 0x06), TAG, "v5 period ~1s");
    ESP_RETURN_ON_ERROR(setReg(0x02, 0x7f), TAG, "v5 50% duty");
  } else {
    ESP_RETURN_ON_ERROR(setReg(0x07, 0x17), TAG, "period ~1s");
    ESP_RETURN_ON_ERROR(setReg(0x06, 0x7f), TAG, "50% duty");
  }
  return ESP_OK;
}
esp_err_t rgb_lcd_idf::noBlinkLED() {
  if (rgb_addr_ == RGB_ADDRESS_V5) {
    return setReg(0x04, 0x15);
  } else {
    ESP_RETURN_ON_ERROR(setReg(0x07, 0x00), TAG, "stop blink");
    return setReg(0x06, 0xff);
  }
}

// Low-level helpers
esp_err_t rgb_lcd_idf::i2c_send_bytes(uint8_t addr, const uint8_t *data, size_t len) {
  if (!initialized_) return ESP_ERR_INVALID_STATE;

  // Be a bit forgiving: long wires / weak pullups can cause transient I2C errors.
  constexpr int kMaxAttempts = 3;
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    esp_err_t err = i2c_master_write_to_device(port_, addr, data, len, pdMS_TO_TICKS(200));
    if (err == ESP_OK) return ESP_OK;
    if (attempt < kMaxAttempts) vTaskDelay(pdMS_TO_TICKS(10));
    else return err;
  }
  return ESP_FAIL;
}

esp_err_t rgb_lcd_idf::probe_rgb_chip() {
  // Common addresses seen in Grove/clone RGB backlight controllers.
  static constexpr uint8_t candidates[] = {
    RGB_ADDRESS_V5,    // 0x30 v5 variant
    RGB_ADDRESS,       // 0x62 classic
    0x60, 0x61, 0x63,  // occasional clones around classic
    0x6A               // rare clone variant
  };

  for (uint8_t addr : candidates) {
    if (probe_i2c_address(port_, addr, pdMS_TO_TICKS(50)) == ESP_OK) {
      rgb_addr_ = addr;
      ESP_LOGI(TAG, "RGB backlight chip found at 0x%02X", rgb_addr_);
      return ESP_OK;
    }
  }

  rgb_addr_ = 0; // mark as absent
  ESP_LOGW(TAG, "RGB backlight chip not found on I2C bus");
  return ESP_ERR_NOT_FOUND;
}
