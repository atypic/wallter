#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "mbedtls/sha256.h"

namespace wallter::ota {

struct ExpectedDigest {
    // If nullptr, SHA-256 check is skipped.
    const uint8_t *sha256 = nullptr; // 32 bytes
    size_t image_size = 0;           // if 0, size check is skipped
};

struct Result {
    size_t bytes_written = 0;
    uint32_t crc32 = 0;
    uint8_t sha256[32] = {0};
};

class Writer {
public:
    Writer();
    ~Writer();

    Writer(const Writer &) = delete;
    Writer &operator=(const Writer &) = delete;

    // Begin streaming an OTA image into the next update partition.
    // - expected.image_size: if non-zero, enforce exact byte count.
    // - expected.sha256: if non-null, enforce SHA-256 match.
    esp_err_t begin(const ExpectedDigest &expected);

    // Write the next chunk of the image.
    esp_err_t write(const void *data, size_t len);

    // Finish: verifies integrity (size/SHA if provided), validates image,
    // marks the new partition as bootable.
    // Does NOT reboot automatically.
    esp_err_t finish(Result *out);

    // Abort an in-progress update.
    void abort();

    bool in_progress() const { return in_progress_; }

private:
    esp_err_t finalize_hash_and_checks_();

    bool in_progress_ = false;

    // Expected
    ExpectedDigest expected_ = {};

    // Progress
    size_t bytes_written_ = 0;
    uint32_t crc32_ = 0;

    const esp_partition_t *update_partition_ = nullptr;
    esp_ota_handle_t ota_handle_ = 0;

    mbedtls_sha256_context sha_ctx_;
    bool sha_started_ = false;
    uint8_t sha256_[32] = {0};
};

// Utility: compare 2 SHA-256 digests.
bool sha256_equal(const uint8_t a[32], const uint8_t b[32]);

} // namespace wallter::ota
