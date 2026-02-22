#include "calibration_store.hpp"

#include "angle_utils.hpp"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <string.h>

namespace wallter::calibration {

static const char *TAG = "cal";

static constexpr const char *kNvsNamespace = "wallter";
static constexpr const char *kNvsKeyBlob   = "cal_ticks_v1";
static constexpr const char *kNvsKeyMeta   = "cal_meta_v1";
static constexpr uint32_t kBlobMagic = 0x57414C54; // 'WALT'

// Default table (index 0..10 corresponds to angles LOWEST_ANGLE..HIGHEST_ANGLE).
// This preserves the previous firmware behavior when no calibration is stored.
static const uint32_t kDefaultTargetTicks[] = {0, 3000, 6000, 9000, 12500, 16400, 20000, 24000, 29000, 33000, 37000};

struct CalBlobV1 {
    uint32_t magic;
    uint32_t version;
    uint32_t ticks[kCount];
};

struct CalMetaBlobV1 {
    uint32_t magic;
    uint32_t version;
    uint8_t min_angle_deg;
    uint8_t max_angle_deg;
    uint8_t reserved[2];
};

static bool is_valid_angle_setting(int angle_deg) {
    if (angle_deg < LOWEST_ANGLE || angle_deg > HIGHEST_ANGLE) {
        return false;
    }
    int delta = angle_deg - LOWEST_ANGLE;
    if ((delta % ANGLE_STEP) != 0) {
        return false;
    }
    return true;
}

static void apply_defaults(uint32_t *target_ticks, int target_len) {
    // If the build's angle table changes, fall back to best-effort copy.
    int n = target_len;
    int d = (int)(sizeof(kDefaultTargetTicks) / sizeof(kDefaultTargetTicks[0]));
    int copy_n = (n < d) ? n : d;
    for (int i = 0; i < copy_n; ++i) {
        target_ticks[i] = kDefaultTargetTicks[i];
    }
    for (int i = copy_n; i < n; ++i) {
        target_ticks[i] = 0;
    }
}

static void derive_entries(uint32_t *target_ticks, int target_len) {
    (void)target_len;
    // Derive 15° between 10° (0) and 20°.
    int idx20 = wallter::angle_to_index(20);
    int idx15 = wallter::angle_to_index(15);
    if (idx20 >= 0 && idx15 >= 0) {
        target_ticks[idx15] = target_ticks[idx20] / 2U;
    }
    // Ensure home is always zero.
    if (target_len > 0) {
        target_ticks[0] = 0;
    }
}

static void log_key_ticks(const uint32_t *target_ticks, int target_len, const char *prefix) {
    if (!target_ticks || target_len <= 0) {
        return;
    }
    int idx15 = wallter::angle_to_index(15);
    int idx20 = wallter::angle_to_index(20);
    int idx60 = wallter::angle_to_index(60);

    uint32_t t15 = (idx15 >= 0 && idx15 < target_len) ? target_ticks[idx15] : 0;
    uint32_t t20 = (idx20 >= 0 && idx20 < target_len) ? target_ticks[idx20] : 0;
    uint32_t t60 = (idx60 >= 0 && idx60 < target_len) ? target_ticks[idx60] : 0;
    ESP_LOGI(TAG, "%s ticks: 15=%u 20=%u 60=%u", prefix, (unsigned)t15, (unsigned)t20, (unsigned)t60);
}

esp_err_t init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS init: erasing flash (%d)", (int)err);
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %d", (int)err);
    }
    return err;
}

bool load_or_default(uint32_t *target_ticks, int target_len) {
    if (!target_ticks || target_len <= 0) {
        return false;
    }

    apply_defaults(target_ticks, target_len);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No calibration namespace yet (%d); using defaults", (int)err);
        derive_entries(target_ticks, target_len);
        return false;
    }

    CalBlobV1 blob = {};
    size_t len = sizeof(blob);
    err = nvs_get_blob(handle, kNvsKeyBlob, &blob, &len);
    nvs_close(handle);

    if (err != ESP_OK || len != sizeof(blob)) {
        ESP_LOGI(TAG, "No calibration blob (%d, len=%u); using defaults", (int)err, (unsigned)len);
        derive_entries(target_ticks, target_len);
        return false;
    }

    if (blob.magic != kBlobMagic || blob.version != 1U) {
        ESP_LOGW(TAG, "Calibration blob invalid (magic=0x%08lx ver=%lu)",
                 (unsigned long)blob.magic,
                 (unsigned long)blob.version);
        derive_entries(target_ticks, target_len);
        return false;
    }

    for (int i = 0; i < kCount; ++i) {
        int angle = kFirstAngle + i * kStepAngle;
        int idx = wallter::angle_to_index(angle);
        if (idx >= 0 && idx < target_len) {
            target_ticks[idx] = blob.ticks[i];
        }
    }

    derive_entries(target_ticks, target_len);
    ESP_LOGI(TAG, "Calibration loaded from NVS");
    log_key_ticks(target_ticks, target_len, "Loaded");
    return true;
}

esp_err_t save_from_target_ticks(const uint32_t *target_ticks, int target_len) {
    if (!target_ticks || target_len <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    CalBlobV1 blob = {};
    blob.magic = kBlobMagic;
    blob.version = 1U;

    for (int i = 0; i < kCount; ++i) {
        int angle = kFirstAngle + i * kStepAngle;
        int idx = wallter::angle_to_index(angle);
        blob.ticks[i] = (idx >= 0 && idx < target_len) ? target_ticks[idx] : 0;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %d", (int)err);
        return err;
    }

    err = nvs_set_blob(handle, kNvsKeyBlob, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: %d", (int)err);
    } else {
        ESP_LOGI(TAG, "Calibration saved to NVS");
        log_key_ticks(target_ticks, target_len, "Saved");
    }
    return err;
}

bool load_meta_or_default(CalMeta &out) {
    out.min_angle_deg = (uint8_t)LOWEST_ANGLE;
    out.max_angle_deg = (uint8_t)HIGHEST_ANGLE;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    CalMetaBlobV1 blob = {};
    size_t len = sizeof(blob);
    err = nvs_get_blob(handle, kNvsKeyMeta, &blob, &len);
    nvs_close(handle);
    if (err != ESP_OK || len != sizeof(blob)) {
        return false;
    }
    if (blob.magic != kBlobMagic || blob.version != 1U) {
        return false;
    }

    int min_deg = (int)blob.min_angle_deg;
    int max_deg = (int)blob.max_angle_deg;
    if (!is_valid_angle_setting(min_deg) || !is_valid_angle_setting(max_deg) || min_deg > max_deg) {
        ESP_LOGW(TAG, "Calibration meta invalid (min=%d max=%d); using defaults", min_deg, max_deg);
        out.min_angle_deg = (uint8_t)LOWEST_ANGLE;
        out.max_angle_deg = (uint8_t)HIGHEST_ANGLE;
        return false;
    }

    out.min_angle_deg = (uint8_t)min_deg;
    out.max_angle_deg = (uint8_t)max_deg;
    ESP_LOGI(TAG, "Calibration meta loaded: min=%u max=%u", (unsigned)out.min_angle_deg, (unsigned)out.max_angle_deg);
    return true;
}

esp_err_t save_meta(const CalMeta &meta) {
    int min_deg = (int)meta.min_angle_deg;
    int max_deg = (int)meta.max_angle_deg;
    if (!is_valid_angle_setting(min_deg) || !is_valid_angle_setting(max_deg) || min_deg > max_deg) {
        return ESP_ERR_INVALID_ARG;
    }

    CalMetaBlobV1 blob = {};
    blob.magic = kBlobMagic;
    blob.version = 1U;
    blob.min_angle_deg = meta.min_angle_deg;
    blob.max_angle_deg = meta.max_angle_deg;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for meta: %d", (int)err);
        return err;
    }

    err = nvs_set_blob(handle, kNvsKeyMeta, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS meta save failed: %d", (int)err);
    } else {
        ESP_LOGI(TAG, "Calibration meta saved: min=%u max=%u", (unsigned)meta.min_angle_deg, (unsigned)meta.max_angle_deg);
    }
    return err;
}

} // namespace wallter::calibration
