#pragma once
#include "driver/i2c.h"
#include "esp_check.h"
#include <cstdint>
#include <cstring>

// I2C device addresses
constexpr uint8_t LCD_ADDRESS      = 0x3E;
constexpr uint8_t RGB_ADDRESS      = 0x62; // classic
constexpr uint8_t RGB_ADDRESS_V5   = 0x6A; // newer chip variant

// RGB registers (classic)
constexpr uint8_t REG_MODE1  = 0x00;
constexpr uint8_t REG_MODE2  = 0x01;
constexpr uint8_t REG_OUTPUT = 0x08;

// LCD commands
constexpr uint8_t LCD_CLEARDISPLAY   = 0x01;
constexpr uint8_t LCD_RETURNHOME     = 0x02;
constexpr uint8_t LCD_ENTRYMODESET   = 0x04;
constexpr uint8_t LCD_DISPLAYCONTROL = 0x08;
constexpr uint8_t LCD_CURSORSHIFT    = 0x10;
constexpr uint8_t LCD_FUNCTIONSET    = 0x20;
constexpr uint8_t LCD_SETCGRAMADDR   = 0x40;

// Flags for display entry mode
constexpr uint8_t LCD_ENTRYRIGHT          = 0x00;
constexpr uint8_t LCD_ENTRYLEFT           = 0x02;
constexpr uint8_t LCD_ENTRYSHIFTINCREMENT = 0x01;
constexpr uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00;

// Flags for display on/off control
constexpr uint8_t LCD_DISPLAYON  = 0x04;
constexpr uint8_t LCD_DISPLAYOFF = 0x00;
constexpr uint8_t LCD_CURSORON   = 0x02;
constexpr uint8_t LCD_CURSOROFF  = 0x00;
constexpr uint8_t LCD_BLINKON    = 0x01;
constexpr uint8_t LCD_BLINKOFF   = 0x00;

// Flags for display/cursor shift
constexpr uint8_t LCD_DISPLAYMOVE = 0x08;
constexpr uint8_t LCD_CURSORMOVE  = 0x00;
constexpr uint8_t LCD_MOVERIGHT   = 0x04;
constexpr uint8_t LCD_MOVELEFT    = 0x00;

// Flags for function set
constexpr uint8_t LCD_8BITMODE = 0x10;
constexpr uint8_t LCD_4BITMODE = 0x00;
constexpr uint8_t LCD_2LINE    = 0x08;
constexpr uint8_t LCD_1LINE    = 0x00;
constexpr uint8_t LCD_5x10DOTS = 0x04;
constexpr uint8_t LCD_5x8DOTS  = 0x00;

enum Color { WHITE = 0, RED = 1, GREEN = 2, BLUE = 3 };

class rgb_lcd_idf {
public:
	esp_err_t init_i2c(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz);
	esp_err_t begin(uint8_t cols, uint8_t lines, uint8_t dotsize);

	// High-level text API
	esp_err_t clear();
	esp_err_t home();
	esp_err_t setCursor(uint8_t col, uint8_t row);
	esp_err_t noDisplay();
	esp_err_t display();
	esp_err_t noCursor();
	esp_err_t cursor();
	esp_err_t noBlink();
	esp_err_t blink();
	esp_err_t scrollDisplayLeft();
	esp_err_t scrollDisplayRight();
	esp_err_t leftToRight();
	esp_err_t rightToLeft();
	esp_err_t autoscroll();
	esp_err_t noAutoscroll();
	esp_err_t createChar(uint8_t location, const uint8_t charmap[8]);
	esp_err_t print(const char *str);

	// RGB backlight
	esp_err_t setReg(uint8_t reg, uint8_t dat);
	esp_err_t setRGB(uint8_t r, uint8_t g, uint8_t b);
	esp_err_t setPWM(Color c, uint8_t pwm);
	esp_err_t setColor(Color c);
	esp_err_t blinkLED();
	esp_err_t noBlinkLED();

private:
	// Mid-level
	esp_err_t command(uint8_t value);
	esp_err_t write(uint8_t value);

	// Low-level
	esp_err_t i2c_send_bytes(uint8_t addr, const uint8_t *data, size_t len);
	esp_err_t probe_rgb_chip();

	// State
	i2c_port_t port_ = I2C_NUM_0;
	bool initialized_ = false;

	uint8_t _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	uint8_t _numlines = 0;
	uint8_t _currline = 0;
	uint8_t _displaycontrol = 0;
	uint8_t _displaymode = 0;
	uint8_t rgb_addr_ = RGB_ADDRESS; // default classic
};

