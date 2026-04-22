#define BOARD_TYPE BOARD_TYPE_ARCTIC_CYTRON
#include "accel.hpp"
#include "boards.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

namespace wallter::accel {

static const char *TAG = "accel";

// BMA400 registers
static constexpr uint8_t REG_CHIPID      = 0x00;
static constexpr uint8_t REG_ACC_DATA_0  = 0x04;  // X_LSB..Z_MSB (6 bytes)
static constexpr uint8_t REG_ACC_CONFIG0 = 0x19;  // filter BW + power mode (bits[1:0]: 00=sleep 10=normal)
static constexpr uint8_t REG_ACC_CONFIG1 = 0x1A;  // osr[7:6] | range[5:4] | odr[3:0]

static constexpr uint8_t BMA400_CHIP_ID  = 0x90;

static constexpr i2c_port_t kPort = (i2c_port_t)BMA400_I2C_PORT;

static bool g_initialized = false;
static float g_filtered_angle = 0.0f;
static bool g_filter_seeded = false;
static int16_t g_last_ay = 0;
static int16_t g_last_az = 0;
static float g_angle_offset = 0.0f;

// EMA smoothing weight. Lower values = heavier smoothing. At ~5 Hz update rate
// 0.15 gives a ~1.3 s time constant, enough to reject single-sample noise spikes
// while still tracking the slow mechanical movement.
static constexpr float kAlpha = 0.15f;

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (BMA400_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, reg, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, val, true);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(kPort, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (BMA400_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, reg, true);
    if (err == ESP_OK) err = i2c_master_start(cmd);
    if (err == ESP_OK) err = i2c_master_write_byte(cmd, (BMA400_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (err == ESP_OK) err = i2c_master_read(cmd, buf, len, I2C_MASTER_LAST_NACK);
    if (err == ESP_OK) err = i2c_master_stop(cmd);
    if (err == ESP_OK) err = i2c_master_cmd_begin(kPort, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = BMA400_SDA_PIN;
    conf.scl_io_num = BMA400_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = BMA400_I2C_CLOCK_HZ;

    esp_err_t err = i2c_param_config(kPort, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(kPort, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // BMA400 needs a dummy read of CHIPID on first access to leave suspend mode
    uint8_t chip_id = 0;
    read_regs(REG_CHIPID, &chip_id, 1);
    vTaskDelay(pdMS_TO_TICKS(2));
    err = read_regs(REG_CHIPID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CHIPID read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (chip_id != BMA400_CHIP_ID) {
        ESP_LOGE(TAG, "CHIPID mismatch: got 0x%02X expected 0x%02X", chip_id, BMA400_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMA400 found (0x%02X)", chip_id);

    // ACC_CONFIG0: enter Normal power mode, filt1_bw=0.48*ODR, osr_lp=0
    // bits[1:0]=10(normal) bit[7]=0(0.48*ODR) → 0x02
    err = write_reg(REG_ACC_CONFIG0, 0x02);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(2));   // mode transition <1.5 ms

    // ACC_CONFIG1: acc_range=±4g, osr=0, acc_odr=100 Hz
    // bits[7:6]=01(±4g) bits[5:4]=00(osr=0) bits[3:0]=1000(100Hz) → 0x48
    err = write_reg(REG_ACC_CONFIG1, 0x48);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));  // wait 2/ODR = 20 ms for filter to settle

    g_initialized = true;
    ESP_LOGI(TAG, "BMA400 initialized: 100 Hz, ±4 g");
    return ESP_OK;
}

static float read_raw_angle() {
    // 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    uint8_t raw[6] = {};
    if (read_regs(REG_ACC_DATA_0, raw, 6) != ESP_OK) {
        ESP_LOGW(TAG, "ACC_DATA read failed");
        return g_filtered_angle;
    }

    // Each axis: 12-bit 2's-complement, right-aligned in [MSB[3:0] | LSB[7:0]]
    int16_t ay = (int16_t)(((uint16_t)(raw[3] & 0x0F) << 8) | raw[2]);
    int16_t az = (int16_t)(((uint16_t)(raw[5] & 0x0F) << 8) | raw[4]);
    if (ay & 0x0800) ay |= (int16_t)0xF000;
    if (az & 0x0800) az |= (int16_t)0xF000;
    g_last_ay = ay;
    g_last_az = az;

    // X is the axis of rotation → only Y and Z carry tilt information.
    // atan2(az, ay): at home (retracted, ~30°) gravity is mostly in +Y, giving a small
    // angle. As the board extends toward 60°, az grows and ay shrinks, increasing the angle.
    return atan2f((float)az, (float)ay) * (180.0f / (float)M_PI) + g_angle_offset;
}

void update(float direction_hint) {
    if (!g_initialized) return;

    float raw = read_raw_angle();

    if (!g_filter_seeded) {
        g_filtered_angle = raw;
        g_filter_seeded = true;
        return;
    }

    // Throttled raw debug log (~every 2 s at 5 Hz update rate).
    static uint32_t s_log_tick = 0;
    if (++s_log_tick >= 10) {
        s_log_tick = 0;
        ESP_LOGI(TAG, "ay=%d az=%d raw=%.1f filtered=%.1f",
                 (int)g_last_ay, (int)g_last_az, raw, g_filtered_angle);
    }

    // Stage 1: EMA smoothing.
    float ema = kAlpha * raw + (1.0f - kAlpha) * g_filtered_angle;

    g_filtered_angle = ema;
    (void)direction_hint;
}

float read_angle_deg() {
    return g_filtered_angle;
}

void set_angle_offset(float offset_deg) {
    g_angle_offset = offset_deg;
    ESP_LOGI(TAG, "Angle offset set to %.1f°", (double)offset_deg);
}

} // namespace wallter::accel
