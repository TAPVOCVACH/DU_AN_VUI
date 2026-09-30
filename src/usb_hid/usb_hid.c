#include "usb_hid.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"

static const char *TAG = "USB_HID";

static usb_hid_report_cb_t s_report_cb = NULL;
static usb_hid_char_cb_t   s_char_cb = NULL;
static usb_hid_conn_cb_t   s_conn_cb = NULL;

static volatile bool s_keyboard_ready = false;
static hid_host_device_handle_t s_keyboard_handle = NULL;

static const char hid_keycode_to_ascii_table[2][0x68] = {
    {
        0, 0, 0, 0,
        'a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p',
        'q','r','s','t','u','v','w','x','y','z',
        '1','2','3','4','5','6','7','8','9','0',
        '\n', 0, '\b', '\t', ' ',
        '-', '=', '[', ']', '\\', 0, ';', '\'', '`', ',', '.', '/',
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        '/', '*', '-', '+', '\n',
        '1','2','3','4','5','6','7','8','9','0','.', 0,0,0,0
    },
    {
        0, 0, 0, 0,
        'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
        'Q','R','S','T','U','V','W','X','Y','Z',
        '!','@','#','$','%','^','&','*','(',')',
        '\n', 0, '\b', '\t', ' ',
        '_', '+', '{', '}', '|', 0, ':', '"', '~', '<', '>', '?',
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        '/', '*', '-', '+', '\n',
        '1','2','3','4','5','6','7','8','9','0','.', 0,0,0,0
    }
};

static bool enum_filter_cb(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue)
{
    if (dev_desc == NULL || bConfigurationValue == NULL) return false;
    *bConfigurationValue = 1;
    return true;
}

static void usb_lib_task(void *arg)
{
    while (1) {
        uint32_t event_flags = 0;
        esp_err_t ret = usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (ret != ESP_OK) continue;
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static void parse_keyboard_report(const uint8_t *data, size_t length)
{
    if (data == NULL || length < 8) return;

    size_t offset = 0;
    if (length >= 9 && data[0] != 0 && (data[1] & 0xE0) == 0) {
        offset = 1;
    }

    uint8_t modifier = data[offset];
    uint8_t keys[6];
    memcpy(keys, &data[offset + 2], sizeof(keys));
    // THÊM DÒNG LOG NÀY ĐỂ SOI MÃ PHÍM:
    ESP_LOGI(TAG, "KEY EVENT: Mod=0x%02X, Keys=[%02X %02X %02X %02X %02X %02X]",
             modifier, keys[0], keys[1], keys[2], keys[3], keys[4], keys[5]);

    if (s_report_cb != NULL) {
        s_report_cb(modifier, keys);
    }

    if (s_char_cb != NULL) {
        bool shift_pressed = (modifier & 0x22) != 0;
        for (int i = 0; i < 6; i++) {
            uint8_t keycode = keys[i];
            if (keycode < 0x04 || keycode > 0x67) continue;
            char ch = hid_keycode_to_ascii_table[shift_pressed ? 1 : 0][keycode];
            if (ch != 0) s_char_cb(ch, keycode);
        }
    }
}

static void hid_interface_cb(hid_host_device_handle_t hid_device_handle,
                              const hid_host_interface_event_t event, void *arg)
{
    if (hid_device_handle == NULL) return;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
        uint8_t data[64] = {0};
        size_t data_length = 0;
        esp_err_t ret = hid_host_device_get_raw_input_report_data(
            hid_device_handle, data, sizeof(data), &data_length
        );
        if (ret == ESP_OK && data_length > 0) {
            parse_keyboard_report(data, data_length);
        }
        break;
    }
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        s_keyboard_ready = false;
        if (s_keyboard_handle == hid_device_handle) s_keyboard_handle = NULL;
        hid_host_device_close(hid_device_handle);
        if (s_conn_cb != NULL) s_conn_cb(false);
        break;
    default:
        break;
    }
}

static void hid_device_event(hid_host_device_handle_t hid_device_handle,
                             const hid_host_driver_event_t event, void *arg)
{
    if (hid_device_handle == NULL || event != HID_HOST_DRIVER_EVENT_CONNECTED) return;

    hid_host_dev_params_t dev_params;
    if (hid_host_device_get_params(hid_device_handle, &dev_params) != ESP_OK) return;

    bool is_keyboard = (dev_params.proto == HID_PROTOCOL_KEYBOARD) ||
                       (dev_params.sub_class == HID_SUBCLASS_BOOT_INTERFACE);

    if (!is_keyboard) return;

    const hid_host_device_config_t dev_config = {
        .callback = hid_interface_cb,
        .callback_arg = NULL
    };

    if (hid_host_device_open(hid_device_handle, &dev_config) != ESP_OK) return;

    /* Cho bàn phím Fuhlen ổn định điện áp 50ms trước khi kích hoạt luồng */
    vTaskDelay(pdMS_TO_TICKS(50));

    if (hid_host_device_start(hid_device_handle) != ESP_OK) {
        hid_host_device_close(hid_device_handle);
        return;
    }

    s_keyboard_handle = hid_device_handle;
    s_keyboard_ready = true;
    ESP_LOGI(TAG, "USB HID KEYBOARD READY");

    if (s_conn_cb != NULL) s_conn_cb(true);
}

esp_err_t usb_hid_init(const usb_hid_config_t *config)
{
    if (config != NULL) {
        s_report_cb = config->report_cb;
        s_char_cb = config->char_cb;
        s_conn_cb = config->conn_cb;
    }

    s_keyboard_ready = false;
    s_keyboard_handle = NULL;

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = enum_filter_cb,
    };

    esp_err_t ret = usb_host_install(&host_config);
    if (ret != ESP_OK) return ret;

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        usb_lib_task, "usb_events", 4096, NULL, 2, NULL, 0
    );
    if (task_ret != pdPASS) return ESP_FAIL;

    const hid_host_driver_config_t hid_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 8192,
        .core_id = 0,
        .callback = hid_device_event,
        .callback_arg = NULL
    };

    return hid_host_install(&hid_driver_config);
}

bool usb_hid_is_connected(void)
{
    return s_keyboard_ready && (s_keyboard_handle != NULL);
}

char usb_hid_keycode_to_ascii(uint8_t keycode, bool shift_pressed)
{
    if (keycode < 0x04 || keycode > 0x67) return 0;
    return hid_keycode_to_ascii_table[shift_pressed ? 1 : 0][keycode];
}

esp_err_t usb_hid_set_leds(uint8_t leds)
{
    if (!usb_hid_is_connected() || s_keyboard_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t led_val = leds & 0x07;
    return hid_class_request_set_report(
        s_keyboard_handle,
        HID_REPORT_TYPE_OUTPUT,
        0,
        &led_val,
        1
    );
}