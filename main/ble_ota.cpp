#include "ble_ota.hpp"

#include "ota_update.hpp"

#include "esp_log.h"
#include "esp_system.h"

#include "esp_bt.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <string.h>

namespace wallter::ble_ota {

static const char *TAG = "ble_ota";

// 128-bit UUIDs for Wallter OTA service.
// Note: BLE_UUID128_INIT expects little-endian byte order.
static const ble_uuid128_t kSvcUuid = BLE_UUID128_INIT(
    0x9a, 0x9b, 0x5a, 0x7d, 0x4d, 0x2a, 0x4d, 0x2f,
    0x9e, 0x2c, 0x8d, 0x7c, 0x34, 0x02, 0x01, 0x00);

static const ble_uuid128_t kCtrlUuid = BLE_UUID128_INIT(
    0x9a, 0x9b, 0x5a, 0x7d, 0x4d, 0x2a, 0x4d, 0x2f,
    0x9e, 0x2c, 0x8d, 0x7c, 0x34, 0x02, 0x02, 0x00);

static const ble_uuid128_t kDataUuid = BLE_UUID128_INIT(
    0x9a, 0x9b, 0x5a, 0x7d, 0x4d, 0x2a, 0x4d, 0x2f,
    0x9e, 0x2c, 0x8d, 0x7c, 0x34, 0x02, 0x03, 0x00);

static const ble_uuid128_t kStatusUuid = BLE_UUID128_INIT(
    0x9a, 0x9b, 0x5a, 0x7d, 0x4d, 0x2a, 0x4d, 0x2f,
    0x9e, 0x2c, 0x8d, 0x7c, 0x34, 0x02, 0x04, 0x00);

enum : uint8_t {
    kOpBegin = 0x01,
    kOpData = 0x02,
    kOpEnd = 0x03,
    kOpAbort = 0x04,
    kOpReboot = 0x05,
};

enum : uint8_t {
    kFlagHasSha256 = 0x01,
};

enum class State : uint8_t {
    Idle = 0,
    Receiving = 1,
    Ready = 2,
    Error = 3,
};

struct __attribute__((packed)) Status {
    uint8_t state;
    uint8_t last_err; // 0 = OK, nonzero = generic error
    uint16_t reserved;
    uint32_t bytes_written;
    uint32_t image_size;
    uint32_t crc32;
    uint8_t sha256[32];
};

static wallter::ota::Writer g_writer;
static Status g_status{};
static uint8_t g_expected_sha256[32] = {0};

static uint16_t g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_status_val_handle = 0;

static uint32_t read_u32_le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void status_set(State state, uint8_t last_err) {
    g_status.state = static_cast<uint8_t>(state);
    g_status.last_err = last_err;
}

static void status_notify() {
    if (g_conn_handle == BLE_HS_CONN_HANDLE_NONE || g_status_val_handle == 0) {
        return;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&g_status, sizeof(g_status));
    if (!om) {
        return;
    }
    int rc = ble_gatts_notify_custom(g_conn_handle, g_status_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify failed: %d", rc);
    }
}

static void reset_status_for_begin(uint32_t image_size) {
    memset(&g_status, 0, sizeof(g_status));
    g_status.image_size = image_size;
    status_set(State::Receiving, 0);
}

static int handle_begin(const uint8_t *buf, size_t len) {
    // Format:
    //   op:u8, flags:u8, rsv:u16, image_size:u32, [sha256:32]
    if (len < 8) {
        ESP_LOGW(TAG, "BEGIN: too short (%u)", (unsigned)len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t flags = buf[1];
    uint32_t image_size = read_u32_le(buf + 4);

    const uint8_t *sha = nullptr;
    if (flags & kFlagHasSha256) {
        if (len < 8 + 32) {
            ESP_LOGW(TAG, "BEGIN: missing sha256 (%u)", (unsigned)len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        memcpy(g_expected_sha256, buf + 8, 32);
        sha = g_expected_sha256;
    } else {
        memset(g_expected_sha256, 0, sizeof(g_expected_sha256));
    }

    reset_status_for_begin(image_size);

    wallter::ota::ExpectedDigest expected{};
    expected.sha256 = sha;
    expected.image_size = image_size;

    esp_err_t err = g_writer.begin(expected);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %d", (int)err);
        status_set(State::Error, 1);
        status_notify();
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "OTA begin: size=%u sha=%s", (unsigned)image_size, (sha ? "yes" : "no"));
    status_notify();
    return 0;
}

static int handle_data(const uint8_t *buf, size_t len) {
    if (g_status.state != static_cast<uint8_t>(State::Receiving) || !g_writer.in_progress()) {
        ESP_LOGW(TAG, "DATA: not receiving");
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (len == 0) {
        return 0;
    }

    esp_err_t err = g_writer.write(buf, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA write failed: %d", (int)err);
        status_set(State::Error, 1);
        status_notify();
        return BLE_ATT_ERR_UNLIKELY;
    }

    // Writer tracks bytes internally; mirror it here via monotonic accumulation.
    g_status.bytes_written += (uint32_t)len;
    return 0;
}

static int handle_end() {
    if (g_status.state != static_cast<uint8_t>(State::Receiving) || !g_writer.in_progress()) {
        ESP_LOGW(TAG, "END: not receiving");
        return BLE_ATT_ERR_UNLIKELY;
    }

    wallter::ota::Result res{};
    esp_err_t err = g_writer.finish(&res);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA finish failed: %d", (int)err);
        status_set(State::Error, 1);
        status_notify();
        return BLE_ATT_ERR_UNLIKELY;
    }

    g_status.bytes_written = (uint32_t)res.bytes_written;
    g_status.crc32 = res.crc32;
    memcpy(g_status.sha256, res.sha256, sizeof(g_status.sha256));

    status_set(State::Ready, 0);
    ESP_LOGI(TAG, "OTA ready: bytes=%u crc32=0x%08x", (unsigned)g_status.bytes_written, (unsigned)g_status.crc32);
    status_notify();
    return 0;
}

static int handle_abort() {
    g_writer.abort();
    memset(&g_status, 0, sizeof(g_status));
    status_set(State::Idle, 0);
    status_notify();
    ESP_LOGI(TAG, "OTA aborted");
    return 0;
}

static int gatt_access_ctrl(uint16_t /*conn_handle*/, uint16_t /*attr_handle*/,
                            struct ble_gatt_access_ctxt *ctxt, void * /*arg*/) {
    uint8_t buf[8 + 32] = {0};
    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (copied < 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    switch (buf[0]) {
        case kOpBegin:
            return handle_begin(buf, copied);
        case kOpEnd:
            return handle_end();
        case kOpAbort:
            return handle_abort();
        case kOpReboot:
            ESP_LOGW(TAG, "Reboot requested");
            esp_restart();
            return 0;
        default:
            ESP_LOGW(TAG, "CTRL: unknown op 0x%02x", buf[0]);
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static int gatt_access_data(uint16_t /*conn_handle*/, uint16_t /*attr_handle*/,
                            struct ble_gatt_access_ctxt *ctxt, void * /*arg*/) {
    // Data characteristic: raw chunk.
    // We cap per-write to 512 bytes to keep stack usage modest.
    uint8_t buf[512];
    uint16_t copied = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &copied);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return handle_data(buf, copied);
}

static int gatt_access_status(uint16_t /*conn_handle*/, uint16_t /*attr_handle*/,
                              struct ble_gatt_access_ctxt *ctxt, void * /*arg*/) {
    // Read current status.
    int rc = os_mbuf_append(ctxt->om, &g_status, sizeof(g_status));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kSvcUuid.u,
        .includes = nullptr,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &kCtrlUuid.u,
                .access_cb = gatt_access_ctrl,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_WRITE,
                .min_key_size = 0,
                .val_handle = nullptr,
                .cpfd = nullptr,
            },
            {
                .uuid = &kDataUuid.u,
                .access_cb = gatt_access_data,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_WRITE,
                .min_key_size = 0,
                .val_handle = nullptr,
                .cpfd = nullptr,
            },
            {
                .uuid = &kStatusUuid.u,
                .access_cb = gatt_access_status,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .min_key_size = 0,
                .val_handle = &g_status_val_handle,
                .cpfd = nullptr,
            },
            {
                .uuid = nullptr,
                .access_cb = nullptr,
                .arg = nullptr,
                .descriptors = nullptr,
                .flags = 0,
                .min_key_size = 0,
                .val_handle = nullptr,
                .cpfd = nullptr,
            },
        },
    },
    {
        .type = 0,
        .uuid = nullptr,
        .includes = nullptr,
        .characteristics = nullptr,
    },
};

static int gap_event(struct ble_gap_event *event, void * /*arg*/);
static void advertise();

static void on_reset(int reason) {
    ESP_LOGE(TAG, "reset: %d", reason);
}

static void on_sync() {
    advertise();
}

static void advertise() {
    ble_addr_t addr;
    uint8_t addr_type;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(addr_type, addr.val, nullptr);
    if (rc == 0) {
        ESP_LOGI(TAG, "addr: %02x:%02x:%02x:%02x:%02x:%02x",
                 addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
    }

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    const char *name = ble_svc_gap_device_name();
    fields.name = (const uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(addr_type, nullptr, BLE_HS_FOREVER, &adv_params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "advertising");
}

static int gap_event(struct ble_gap_event *event, void * /*arg*/) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                g_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "connected");
                status_notify();
            } else {
                ESP_LOGW(TAG, "connect failed; status=%d", event->connect.status);
                advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            advertise();
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "subscribe: attr=%u notify=%d", (unsigned)event->subscribe.attr_handle,
                     (int)event->subscribe.cur_notify);
            return 0;

        default:
            return 0;
    }
}

static void host_task(void * /*param*/) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void init() {
    // Safe to call multiple times; ignore subsequent attempts.
    static bool started = false;
    if (started) {
        return;
    }
    started = true;

    ESP_LOGI(TAG, "init");

    // Initialize controller + VHCI (NimBLE HCI transport) + NimBLE host.
    (void)esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %d", (int)err);
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %d", (int)err);
        return;
    }

    err = esp_nimble_hci_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_hci_init failed: %d", (int)err);
        return;
    }

    nimble_port_init();

    // Basic GAP/GATT.
    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set("wallter");

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;

    // No pairing/bonding for OTA transport.
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;

    // Prefer a larger MTU when the peer supports it.
    (void)ble_att_set_preferred_mtu(247);

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }

    status_set(State::Idle, 0);

    nimble_port_freertos_init(host_task);
}

} // namespace wallter::ble_ota
