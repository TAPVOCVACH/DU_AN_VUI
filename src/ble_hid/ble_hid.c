#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_sm.h"
#include "host/util/util.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

void ble_store_config_init(void);

#include "ble_hid/ble_hid.h"

static const char *TAG = "BLE_HID";

#define UUID_HID_SERVICE          0x1812
#define UUID_HID_INFORMATION      0x2A4A
#define UUID_HID_REPORT_MAP       0x2A4B
#define UUID_HID_CONTROL_POINT    0x2A4C
#define UUID_HID_REPORT           0x2A4D
#define UUID_HID_PROTOCOL_MODE    0x2A4E
#define UUID_BOOT_KEYBOARD_INPUT  0x2A22
#define UUID_BOOT_KEYBOARD_OUTPUT 0x2A32
#define UUID_REPORT_REFERENCE     0x2908

#define UUID_DEVICE_INFO_SERVICE  0x180A
#define UUID_MANUFACTURER_NAME    0x2A29
#define UUID_MODEL_NUMBER         0x2A24
#define UUID_SERIAL_NUMBER        0x2A25
#define UUID_PNP_ID               0x2A50

#define UUID_BATTERY_SERVICE      0x180F
#define UUID_BATTERY_LEVEL        0x2A19

/* BẢNG REPORT MAP CHUẨN APPLE CÓ REPORT ID 1 */
static const uint8_t s_hid_report_map[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   REPORT ID (1)

    // Modifiers (8 bits)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0xE0,       //   Usage Minimum (224)
    0x29, 0xE7,       //   Usage Maximum (231)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x08,       //   Report Count (8)
    0x81, 0x02,       //   Input (Data, Variable, Absolute)

    // Reserved (1 byte)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x01,       //   Input (Constant)

    // 6 Keycodes
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x06,       //   Report Count (6)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x65,       //   Logical Maximum (101)
    0x05, 0x07,       //   Usage Page (Key Codes)
    0x19, 0x00,       //   Usage Minimum (0)
    0x29, 0x65,       //   Usage Maximum (101)
    0x81, 0x00,       //   Input (Data, Array)

    // Output LEDs (Caps Lock, Num Lock)
    0x85, 0x01,       //   REPORT ID (1)
    0x05, 0x08,       //   Usage Page (LEDs)
    0x19, 0x01,       //   Usage Minimum (1)
    0x29, 0x05,       //   Usage Maximum (5)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x05,       //   Report Count (5)
    0x91, 0x02,       //   Output (Data, Variable, Absolute)

    // Padding (3 bits)
    0x75, 0x03,       //   Report Size (3)
    0x95, 0x01,       //   Report Count (1)
    0x91, 0x01,       //   Output (Constant)

    0xC0              // End Collection
};

// bcdHID = 0x0111, CountryCode = 0x00, Flags = 0x02 (Normally Connectable)
static const uint8_t s_hid_information[] = { 0x11, 0x01, 0x00, 0x02 };

// Report Reference: Byte 0 = Report ID (0x01), Byte 1 = Type (1: Input, 2: Output)
static const uint8_t s_input_report_reference[]  = { 0x01, 0x01 };
static const uint8_t s_output_report_reference[] = { 0x01, 0x02 };

// PnP ID chuẩn Apple
static const uint8_t s_pnp_id[] = { 0x01, 0x4C, 0x00, 0x01, 0x02, 0x10, 0x01 };
static const char *s_manufacturer = "Apple Inc.";
static const char *s_model_number = "A1644";
static const char *s_serial_number = "10001";

static uint8_t s_own_addr_type = 0;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static uint16_t s_input_val_handle = 0;
static uint16_t s_output_val_handle = 0;
static uint16_t s_boot_input_val_handle = 0;
static uint16_t s_boot_output_val_handle = 0;
static uint16_t s_battery_val_handle = 0;

static volatile bool s_ble_connected = false;
static volatile bool s_input_notify_enabled = false;
static volatile bool s_boot_input_notify_enabled = false;

static uint8_t s_protocol_mode = 0x01; // 1: Report Protocol, 0: Boot Protocol
static uint8_t s_hid_control_point = 0x00;
static uint8_t s_output_leds = 0x00;
static uint8_t s_battery_level = 100;

static ble_hid_conn_cb_t s_conn_cb = NULL;
static ble_hid_leds_cb_t s_leds_cb = NULL;

// Đổi tên mới để ép iOS xóa sạch cache cũ
static const char *s_device_name = "BT Keyboard";

static int ble_hid_gap_event(struct ble_gap_event *event, void *arg);

static int report_reference_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const uint8_t *ref = (const uint8_t *)arg;
    return os_mbuf_append(ctxt->om, ref, 2) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int battery_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return os_mbuf_append(ctxt->om, &s_battery_level, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int dis_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->chr->uuid->type == BLE_UUID_TYPE_16) {
        uint16_t uuid = BLE_UUID16(ctxt->chr->uuid)->value;
        if (uuid == UUID_MANUFACTURER_NAME) {
            return os_mbuf_append(ctxt->om, s_manufacturer, strlen(s_manufacturer)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (uuid == UUID_MODEL_NUMBER) {
            return os_mbuf_append(ctxt->om, s_model_number, strlen(s_model_number)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (uuid == UUID_SERIAL_NUMBER) {
            return os_mbuf_append(ctxt->om, s_serial_number, strlen(s_serial_number)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (uuid == UUID_PNP_ID) {
            return os_mbuf_append(ctxt->om, s_pnp_id, sizeof(s_pnp_id)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int hid_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (ctxt->chr->uuid->type == BLE_UUID_TYPE_16) {
            uint16_t uuid = BLE_UUID16(ctxt->chr->uuid)->value;
            switch (uuid) {
            case UUID_HID_INFORMATION:
                return os_mbuf_append(ctxt->om, s_hid_information, sizeof(s_hid_information)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            case UUID_HID_REPORT_MAP:
                return os_mbuf_append(ctxt->om, s_hid_report_map, sizeof(s_hid_report_map)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            case UUID_HID_PROTOCOL_MODE:
                return os_mbuf_append(ctxt->om, &s_protocol_mode, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            case UUID_HID_CONTROL_POINT:
                return os_mbuf_append(ctxt->om, &s_hid_control_point, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            case UUID_BOOT_KEYBOARD_INPUT:
            case UUID_HID_REPORT: {
                if (attr_handle == s_output_val_handle) {
                    return os_mbuf_append(ctxt->om, &s_output_leds, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                uint8_t empty[8] = {0};
                return os_mbuf_append(ctxt->om, empty, sizeof(empty)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            case UUID_BOOT_KEYBOARD_OUTPUT:
                return os_mbuf_append(ctxt->om, &s_output_leds, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            default:
                break;
            }
        }
        return BLE_ATT_ERR_UNLIKELY;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (ctxt->chr->uuid->type == BLE_UUID_TYPE_16) {
            uint16_t uuid = BLE_UUID16(ctxt->chr->uuid)->value;
            if (uuid == UUID_HID_PROTOCOL_MODE) {
                if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                    os_mbuf_copydata(ctxt->om, 0, 1, &s_protocol_mode);
                    ESP_LOGI(TAG, "Protocol Mode set: %u", s_protocol_mode);
                }
                return 0;
            }
            if (uuid == UUID_HID_CONTROL_POINT) {
                if (OS_MBUF_PKTLEN(ctxt->om) >= 1) os_mbuf_copydata(ctxt->om, 0, 1, &s_hid_control_point);
                return 0;
            }
            if (uuid == UUID_BOOT_KEYBOARD_OUTPUT || (uuid == UUID_HID_REPORT && attr_handle == s_output_val_handle)) {
                uint8_t leds = 0;
                if (OS_MBUF_PKTLEN(ctxt->om) >= 1) {
                    os_mbuf_copydata(ctxt->om, 0, 1, &leds);
                    s_output_leds = leds;
                    ESP_LOGI(TAG, "GATT RECEIVED LED = 0x%02X", leds);
                    if (s_leds_cb != NULL) {
                        s_leds_cb(leds);
                    }
                }
                return 0;
            }
        }
        return BLE_ATT_ERR_UNLIKELY;

    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    // 1. DEVICE INFORMATION
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_DEVICE_INFO_SERVICE),
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = BLE_UUID16_DECLARE(UUID_MANUFACTURER_NAME), .access_cb = dis_access_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(UUID_MODEL_NUMBER),      .access_cb = dis_access_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(UUID_SERIAL_NUMBER),     .access_cb = dis_access_cb, .flags = BLE_GATT_CHR_F_READ },
            { .uuid = BLE_UUID16_DECLARE(UUID_PNP_ID),            .access_cb = dis_access_cb, .flags = BLE_GATT_CHR_F_READ },
            { 0 }
        }
    },
    // 2. BATTERY SERVICE
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_BATTERY_SERVICE),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(UUID_BATTERY_LEVEL),
                .val_handle = &s_battery_val_handle,
                .access_cb = battery_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
            },
            { 0 }
        }
    },
    // 3. HID SERVICE (BẮT BUỘC ĐỦ BOOT + REPORT CHO APPLE)
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(UUID_HID_SERVICE),
        .characteristics = (struct ble_gatt_chr_def[]) {
            { 
                .uuid = BLE_UUID16_DECLARE(UUID_HID_INFORMATION), 
                .access_cb = hid_gatt_access_cb, 
                .flags = BLE_GATT_CHR_F_READ 
            },
            { 
                .uuid = BLE_UUID16_DECLARE(UUID_HID_REPORT_MAP), 
                .access_cb = hid_gatt_access_cb, 
                .flags = BLE_GATT_CHR_F_READ 
            },
            { 
                .uuid = BLE_UUID16_DECLARE(UUID_HID_CONTROL_POINT), 
                .access_cb = hid_gatt_access_cb, 
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP 
            },
            { 
                .uuid = BLE_UUID16_DECLARE(UUID_HID_PROTOCOL_MODE), 
                .access_cb = hid_gatt_access_cb, 
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP 
            },
            // BOOT KEYBOARD INPUT
            {
                .uuid = BLE_UUID16_DECLARE(UUID_BOOT_KEYBOARD_INPUT),
                .val_handle = &s_boot_input_val_handle,
                .access_cb = hid_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY
            },
            // BOOT KEYBOARD OUTPUT
            {
                .uuid = BLE_UUID16_DECLARE(UUID_BOOT_KEYBOARD_OUTPUT),
                .val_handle = &s_boot_output_val_handle,
                .access_cb = hid_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP
            },
            // REPORT KEYBOARD INPUT (ID 1)
            {
                .uuid = BLE_UUID16_DECLARE(UUID_HID_REPORT),
                .val_handle = &s_input_val_handle,
                .access_cb = hid_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    { 
                        .uuid = BLE_UUID16_DECLARE(UUID_REPORT_REFERENCE), 
                        .att_flags = BLE_ATT_F_READ, 
                        .access_cb = report_reference_access_cb, 
                        .arg = (void *)s_input_report_reference 
                    },
                    { 0 }
                }
            },
            // REPORT KEYBOARD OUTPUT (ID 1)
            {
                .uuid = BLE_UUID16_DECLARE(UUID_HID_REPORT),
                .val_handle = &s_output_val_handle,
                .access_cb = hid_gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    { 
                        .uuid = BLE_UUID16_DECLARE(UUID_REPORT_REFERENCE), 
                        .att_flags = BLE_ATT_F_READ, 
                        .access_cb = report_reference_access_cb, 
                        .arg = (void *)s_output_report_reference 
                    },
                    { 0 }
                }
            },
            { 0 }
        }
    },
    { 0 }
};

static void ble_hid_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;

    memset(&fields, 0, sizeof(fields));
    // Cờ kết hợp Apple Accessory
    fields.flags = BLE_HS_ADV_F_DISC_LTD | BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.appearance = 0x03C1; // Bàn phím HID
    fields.appearance_is_present = 1;
    fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(UUID_HID_SERVICE) };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.name = (uint8_t *)s_device_name;
    rsp_fields.name_len = strlen(s_device_name);
    rsp_fields.name_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp_fields);

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = 32;
    adv_params.itvl_max = 48;

    ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_hid_gap_event, NULL);
    ESP_LOGI(TAG, "BLE Advertising started: %s", s_device_name);
}

static int ble_hid_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_ble_connected = true;
            s_input_notify_enabled = false;
            s_boot_input_notify_enabled = false;
            ESP_LOGI(TAG, "BLE connected: handle=%u", s_conn_handle);

            // Tham số kết nối 15ms chuẩn Apple
            struct ble_gap_upd_params params = {
                .itvl_min = 12,     // 15ms
                .itvl_max = 12,     // 15ms
                .latency = 0,
                .supervision_timeout = 400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            ble_gap_update_params(s_conn_handle, &params);

            // Bắt buộc khởi tạo bảo mật
            ble_gap_security_initiate(s_conn_handle);

            if (s_conn_cb != NULL) s_conn_cb(false);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_ble_connected = false;
            ble_hid_advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE disconnected: reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_ble_connected = false;
        s_input_notify_enabled = false;
        s_boot_input_notify_enabled = false;
        if (s_conn_cb != NULL) s_conn_cb(false);
        ble_hid_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_input_val_handle) {
            s_input_notify_enabled = (event->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "REPORT INPUT NOTIFY = %s", s_input_notify_enabled ? "ON" : "OFF");
        }
        if (event->subscribe.attr_handle == s_boot_input_val_handle) {
            s_boot_input_notify_enabled = (event->subscribe.cur_notify != 0);
            ESP_LOGI(TAG, "BOOT INPUT NOTIFY = %s", s_boot_input_notify_enabled ? "ON" : "OFF");
        }
        if (s_conn_cb != NULL) {
            s_conn_cb(s_ble_connected && (s_input_notify_enabled || s_boot_input_notify_enabled));
        }
        break;

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        struct ble_sm_io pkey = {0};
        pkey.action = event->passkey.params.action;
        if (event->passkey.params.action == BLE_SM_IOACT_NUMCMP) {
            pkey.numcmp_accept = 1;
        } else if (event->passkey.params.action == BLE_SM_IOACT_DISP ||
                   event->passkey.params.action == BLE_SM_IOACT_INPUT) {
            pkey.passkey = 123456;
        } else {
            pkey.action = BLE_SM_IOACT_NONE;
        }
        ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    default:
        break;
    }
    return 0;
}

static void ble_hid_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: %d", rc);
        return;
    }
    ble_hid_advertise();
}

static void ble_hid_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE reset: %d", reason);
}

static void ble_hid_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================
 * GỬI GÓI PHÍM 8 BYTES CHUẨN
 * ============================================================ */

esp_err_t ble_hid_send_report(uint8_t modifier, const uint8_t keys[6])
{
    if (keys == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_ble_connected || s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t report[8];
    report[0] = modifier;
    report[1] = 0x00;
    memcpy(&report[2], keys, 6);

    // Gửi qua Report Input nếu iOS ở chế độ Report
    if (s_input_notify_enabled) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(report, sizeof(report));
        if (om != NULL) {
            ble_gatts_notify_custom(s_conn_handle, s_input_val_handle, om);
        }
    }

    // Gửi qua Boot Input nếu iOS ở chế độ Boot
    if (s_boot_input_notify_enabled) {
        struct os_mbuf *om2 = ble_hs_mbuf_from_flat(report, sizeof(report));
        if (om2 != NULL) {
            ble_gatts_notify_custom(s_conn_handle, s_boot_input_val_handle, om2);
        }
    }

    return ESP_OK;
}

bool ble_hid_is_connected(void)
{
    return s_ble_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE &&
           (s_input_notify_enabled || s_boot_input_notify_enabled);
}

esp_err_t ble_hid_send_key(uint8_t modifier, uint8_t keycode)
{
    uint8_t keys[6] = { keycode, 0, 0, 0, 0, 0 };
    esp_err_t ret = ble_hid_send_report(modifier, keys);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));
    return ble_hid_release_all();
}

esp_err_t ble_hid_release_all(void)
{
    uint8_t keys[6] = { 0 };
    return ble_hid_send_report(0, keys);
}

esp_err_t ble_hid_set_battery_level(uint8_t level)
{
    if (level > 100) level = 100;
    s_battery_level = level;
    if (s_ble_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_battery_val_handle != 0) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_battery_level, 1);
        if (om != NULL) ble_gatts_notify_custom(s_conn_handle, s_battery_val_handle, om);
    }
    return ESP_OK;
}

esp_err_t ble_hid_init(const ble_hid_config_t *config)
{
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    s_conn_cb = config->conn_cb;
    s_leds_cb = config->leds_cb;

    nimble_port_init();

    ble_hs_cfg.reset_cb = ble_hid_on_reset;
    ble_hs_cfg.sync_cb = ble_hid_on_sync;

    // Cấu hình mã hóa bảo mật bắt buộc của Apple
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set(s_device_name);

    ble_gatts_count_cfg(s_gatt_svcs);
    ble_gatts_add_svcs(s_gatt_svcs);

    ble_store_config_init();

    nimble_port_freertos_init(ble_hid_host_task);

    ESP_LOGI(TAG, "BLE HID Initialized: %s", s_device_name);
    return ESP_OK;
}