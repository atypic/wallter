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

struct DefaultTickEntry {
    int angle_deg;
    uint32_t ticks;
};

// Default table in absolute angles (degrees). This preserves the previous firmware
// behavior when no calibration is stored, while remaining correct even if the
// build's LOWEST_ANGLE changes.
#if defined(BOARD_TYPE) && (BOARD_TYPE == BOARD_TYPE_ARCTIC_CYTRON)
// On ARCTIC_CYTRON, mechanical home is 30deg (LOWEST_ANGLE), so 30deg must be 0 ticks.
// Values below are based on the legacy absolute table re-based so that 30deg is 0.
static constexpr DefaultTickEntry kDefaultTargetTicks[] = {
    {30, 0},
    {35, 3900},
    {40, 7500},
    {45, 11500},
    {50, 16500},
    {55, 20500},
    {60, 24500},
};
#else
static constexpr DefaultTickEntry kDefaultTargetTicks[] = {
    {10, 0},
    {15, 3000},
    {20, 6000},
    {25, 9000},
    {30, 12500},
    {35, 16400},
    {40, 20000},
    {45, 24000},
    {50, 29000},
    {55, 33000},
    {60, 37000},
    {65, 42000},
    {70, 47000},
};
#endif

static constexpr int kV1FirstAngle = 20;
static constexpr int kV1LastAngle  = 60;
static constexpr int kV1StepAngle  = 5;
static constexpr int kV1Count      = ((kV1LastAngle - kV1FirstAngle) / kV1StepAngle) + 1; // 9

struct CalBlobV1 {
    uint32_t magic;
    uint32_t version;
    uint32_t ticks[kV1Count];
};

struct CalBlobV2 {
    uint32_t magic;
    uint32_t version;
    uint32_t ticks[wallter::kMaxAngles];
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
    if (!target_ticks || target_len <= 0) {
        return;
    }

    // Start from 0s, then fill any entries that are valid for this build.
    for (int i = 0; i < target_len; ++i) {
        target_ticks[i] = 0;
    }

    for (const auto &e : kDefaultTargetTicks) {
        int idx = wallter::angle_to_index(e.angle_deg);
        if (idx >= 0 && idx < target_len) {
            target_ticks[idx] = e.ticks;
        }
    }

    // Clamp default usable table to the board's configured default calibration range.
    // (This keeps "reset calibration" behavior consistent with DEFAULT_CAL_MIN/MAX.)
    int min_idx = wallter::angle_to_index(DEFAULT_CAL_MIN_ANGLE);
    int max_idx = wallter::angle_to_index(DEFAULT_CAL_MAX_ANGLE);
    if (min_idx >= 0 && max_idx >= 0 && min_idx <= max_idx) {
        for (int i = 0; i < target_len; ++i) {
            if (i < min_idx || i > max_idx) {
                target_ticks[i] = 0;
            }
        }
    }
}

static void derive_entries(uint32_t *target_ticks, int target_len) {
    (void)target_ticks;
    (void)target_len;
    // No derived entries in v2: each 5-degree angle is calibrated explicitly (or skipped).
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

    size_t len = 0;
    err = nvs_get_blob(handle, kNvsKeyBlob, nullptr, &len);
    if (err != ESP_OK || len < (sizeof(uint32_t) * 2)) {
        nvs_close(handle);
        ESP_LOGI(TAG, "No calibration blob (%d, len=%u); using defaults", (int)err, (unsigned)len);
        derive_entries(target_ticks, target_len);
        return false;
    }

    // Read into a buffer large enough for the latest blob.
    uint8_t buf[sizeof(CalBlobV2)] = {0};
    if (len > sizeof(buf)) {
        nvs_close(handle);
        ESP_LOGW(TAG, "Calibration blob too large (len=%u); using defaults", (unsigned)len);
        derive_entries(target_ticks, target_len);
        return false;
    }
    err = nvs_get_blob(handle, kNvsKeyBlob, buf, &len);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Calibration blob read failed (%d); using defaults", (int)err);
        derive_entries(target_ticks, target_len);
        return false;
    }

    const uint32_t magic = ((const uint32_t *)buf)[0];
    const uint32_t ver = ((const uint32_t *)buf)[1];
    if (magic != kBlobMagic) {
        ESP_LOGW(TAG, "Calibration blob invalid magic (0x%08lx)", (unsigned long)magic);
        derive_entries(target_ticks, target_len);
        return false;
    }

    if (ver == 2U) {
        // V2 stores the full tick table sized to wallter::kMaxAngles.
        // If LOWEST_ANGLE/HIGHEST_ANGLE changed across firmware, the stored blob length
        // will no longer match and the index mapping becomes invalid. Treat as incompatible.
        if (len != sizeof(CalBlobV2)) {
            ESP_LOGW(TAG, "Calibration v2 blob incompatible (len=%u expected=%u); using defaults",
                     (unsigned)len,
                     (unsigned)sizeof(CalBlobV2));
            derive_entries(target_ticks, target_len);
            return false;
        }
        const CalBlobV2 *b2 = (const CalBlobV2 *)buf;
        for (int i = 0; i < target_len; ++i) {
            target_ticks[i] = b2->ticks[i];
        }
        derive_entries(target_ticks, target_len);
        ESP_LOGI(TAG, "Calibration loaded from NVS (v2)");
        log_key_ticks(target_ticks, target_len, "Loaded");
        return true;
    }

    if (ver == 1U && len == sizeof(CalBlobV1)) {
        const CalBlobV1 *b1 = (const CalBlobV1 *)buf;
        // Map legacy 20..60 slice into the current target table.
        for (int i = 0; i < kV1Count; ++i) {
            int angle = kV1FirstAngle + i * kV1StepAngle;
            int idx = wallter::angle_to_index(angle);
            if (idx >= 0 && idx < target_len) {
                target_ticks[idx] = b1->ticks[i];
            }
        }
        derive_entries(target_ticks, target_len);
        ESP_LOGI(TAG, "Calibration loaded from NVS (v1)");
        log_key_ticks(target_ticks, target_len, "Loaded");
        return true;
    }

    ESP_LOGW(TAG, "Calibration blob unexpected (ver=%lu len=%u); using defaults", (unsigned long)ver, (unsigned)len);
    derive_entries(target_ticks, target_len);
    return false;

    // unreachable
}

esp_err_t save_from_target_ticks(const uint32_t *target_ticks, int target_len) {
    if (!target_ticks || target_len <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    CalBlobV2 blob = {};
    blob.magic = kBlobMagic;
    blob.version = 2U;
    for (int i = 0; i < wallter::kMaxAngles; ++i) {
        blob.ticks[i] = (i >= 0 && i < target_len) ? target_ticks[i] : 0;
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
    out.min_angle_deg = (uint8_t)DEFAULT_CAL_MIN_ANGLE;
    out.max_angle_deg = (uint8_t)DEFAULT_CAL_MAX_ANGLE;

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
        out.min_angle_deg = (uint8_t)DEFAULT_CAL_MIN_ANGLE;
        out.max_angle_deg = (uint8_t)DEFAULT_CAL_MAX_ANGLE;
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

esp_err_t erase_calibration() {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed for erase: %d", (int)err);
        return err;
    }

    esp_err_t e1 = nvs_erase_key(handle, kNvsKeyBlob);
    if (e1 != ESP_OK && e1 != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS erase key failed (%s): %d", kNvsKeyBlob, (int)e1);
    }
    esp_err_t e2 = nvs_erase_key(handle, kNvsKeyMeta);
    if (e2 != ESP_OK && e2 != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "NVS erase key failed (%s): %d", kNvsKeyMeta, (int)e2);
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed for erase: %d", (int)err);
        return err;
    }

    ESP_LOGW(TAG, "Calibration erased (keys: %s, %s)", kNvsKeyBlob, kNvsKeyMeta);
    return ESP_OK;
}

} // namespace wallter::calibration
