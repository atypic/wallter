#include "ota_update.hpp"

#include <string.h>

#include "esp_log.h"
#include "esp_rom_crc.h"

namespace wallter::ota {

static const char *TAG = "ota";

bool sha256_equal(const uint8_t a[32], const uint8_t b[32]) {
    return memcmp(a, b, 32) == 0;
}

Writer::Writer() {
    mbedtls_sha256_init(&sha_ctx_);
}

Writer::~Writer() {
    abort();
    mbedtls_sha256_free(&sha_ctx_);
}

esp_err_t Writer::begin(const ExpectedDigest &expected) {
    if (in_progress_) {
        return ESP_ERR_INVALID_STATE;
    }

    expected_ = expected;

    update_partition_ = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition_) {
        ESP_LOGE(TAG, "No OTA update partition found (partition table likely missing ota_0/ota_1)");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Writing OTA image to partition: %s @ 0x%08x", update_partition_->label,
             (unsigned)update_partition_->address);

    bytes_written_ = 0;
    crc32_ = 0;
    memset(sha256_, 0, sizeof(sha256_));

    int mbed_err = mbedtls_sha256_starts(&sha_ctx_, 0 /* is224 */);
    if (mbed_err != 0) {
        ESP_LOGE(TAG, "mbedtls_sha256_starts failed: -0x%04X", (unsigned)(-mbed_err));
        update_partition_ = nullptr;
        ota_handle_ = 0;
        return ESP_FAIL;
    }
    sha_started_ = true;

    // Let ESP-IDF validate image header as we write.
    esp_err_t err = esp_ota_begin(update_partition_, expected_.image_size, &ota_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        sha_started_ = false;
        mbedtls_sha256_free(&sha_ctx_);
        mbedtls_sha256_init(&sha_ctx_);
        update_partition_ = nullptr;
        ota_handle_ = 0;
        return err;
    }

    in_progress_ = true;
    return ESP_OK;
}

esp_err_t Writer::write(const void *data, size_t len) {
    if (!in_progress_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || len == 0) {
        return ESP_OK;
    }

    esp_err_t err = esp_ota_write(ota_handle_, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed at %u bytes: %s", (unsigned)bytes_written_, esp_err_to_name(err));
        return err;
    }

    bytes_written_ += len;
    crc32_ = esp_rom_crc32_le(crc32_, (const uint8_t *)data, len);
    if (sha_started_) {
        int mbed_err = mbedtls_sha256_update(&sha_ctx_, (const unsigned char *)data, len);
        if (mbed_err != 0) {
            ESP_LOGE(TAG, "mbedtls_sha256_update failed: -0x%04X", (unsigned)(-mbed_err));
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

esp_err_t Writer::finalize_hash_and_checks_() {
    if (sha_started_) {
        int mbed_err = mbedtls_sha256_finish(&sha_ctx_, sha256_);
        if (mbed_err != 0) {
            ESP_LOGE(TAG, "mbedtls_sha256_finish failed: -0x%04X", (unsigned)(-mbed_err));
            return ESP_FAIL;
        }
        sha_started_ = false;
    }

    if (expected_.image_size != 0 && bytes_written_ != expected_.image_size) {
        ESP_LOGE(TAG, "Image size mismatch: wrote=%u expected=%u", (unsigned)bytes_written_,
                 (unsigned)expected_.image_size);
        return ESP_ERR_INVALID_SIZE;
    }

    if (expected_.sha256) {
        if (!sha256_equal(sha256_, expected_.sha256)) {
            ESP_LOGE(TAG, "SHA-256 mismatch");
            return ESP_ERR_INVALID_CRC;
        }
    }

    return ESP_OK;
}

esp_err_t Writer::finish(Result *out) {
    if (!in_progress_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = finalize_hash_and_checks_();
    if (err != ESP_OK) {
        // Abort and cleanup.
        esp_ota_abort(ota_handle_);
        in_progress_ = false;
        update_partition_ = nullptr;
        ota_handle_ = 0;
        return err;
    }

    err = esp_ota_end(ota_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        in_progress_ = false;
        update_partition_ = nullptr;
        ota_handle_ = 0;
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        in_progress_ = false;
        update_partition_ = nullptr;
        ota_handle_ = 0;
        return err;
    }

    ESP_LOGW(TAG, "OTA image accepted. Reboot required to boot into new firmware.");

    if (out) {
        out->bytes_written = bytes_written_;
        out->crc32 = crc32_;
        memcpy(out->sha256, sha256_, sizeof(out->sha256));
    }

    in_progress_ = false;
    update_partition_ = nullptr;
    ota_handle_ = 0;

    return ESP_OK;
}

void Writer::abort() {
    if (!in_progress_) {
        return;
    }

    (void)esp_ota_abort(ota_handle_);
    in_progress_ = false;
    update_partition_ = nullptr;
    ota_handle_ = 0;
    sha_started_ = false;
}

} // namespace wallter::ota
