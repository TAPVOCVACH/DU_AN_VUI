#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "usb_hid/usb_hid.h"
#include "ble_hid/ble_hid.h"
#include "led_rgb/led_rgb.h"

static const char *TAG = "MAIN";

#define KEYBOARD_QUEUE_LENGTH   32
#define LED_QUEUE_LENGTH        1

typedef struct {
    uint8_t modifier;
    uint8_t keys[6];
} bridge_keyboard_report_t;

static QueueHandle_t s_keyboard_queue = NULL;
static QueueHandle_t s_led_queue = NULL;

static volatile uint32_t s_total_usb_reports = 0;
static volatile uint32_t s_forwarded_ble = 0;
static volatile uint32_t s_dropped_no_ble = 0;
static volatile uint32_t s_ble_send_errors = 0;

static uint8_t s_last_led_state = 0;

static void update_led_state(void)
{
    const bool usb_ok = usb_hid_is_connected();
    const bool ble_ok = ble_hid_is_connected();

    if (usb_ok && ble_ok) {
        led_rgb_set_state(LED_STATE_READY);
    } else if (!usb_ok && !ble_ok) {
        led_rgb_set_state(LED_STATE_WAIT_USB_AND_BLE);
    } else if (!usb_ok) {
        led_rgb_set_state(LED_STATE_WAIT_USB);
    } else {
        led_rgb_set_state(LED_STATE_WAIT_BLE);
    }
}

static bool report_has_key(const bridge_keyboard_report_t *report)
{
    if (report->modifier != 0) {
        return true;
    }
    for (int i = 0; i < 6; i++) {
        if (report->keys[i] != 0) {
            return true;
        }
    }
    return false;
}

static void bridge_keyboard_task(void *arg)
{
    bridge_keyboard_report_t report;
    ESP_LOGI(TAG, "Bridge keyboard task started");

    while (1) {
        if (xQueueReceive(s_keyboard_queue, &report, portMAX_DELAY) == pdTRUE) {
            if (!ble_hid_is_connected()) {
                s_dropped_no_ble++;
                continue;
            }

            esp_err_t ret = ble_hid_send_report(report.modifier, report.keys);
            if (ret == ESP_OK) {
                s_forwarded_ble++;
                if (report_has_key(&report)) {
                    led_rgb_blink_key();
                }
            } else {
                s_ble_send_errors++;
            }

            vTaskDelay(pdMS_TO_TICKS(8));
        }
    }
}

static void bridge_led_task(void *arg)
{
    uint8_t leds = 0;
    ESP_LOGI(TAG, "Bridge LED task started");

    while (1) {
        if (xQueueReceive(s_led_queue, &leds, portMAX_DELAY) == pdTRUE) {
            if (!usb_hid_is_connected()) {
                continue;
            }
            usb_hid_set_leds(leds);
        }
    }
}

static void on_usb_report(uint8_t modifier, const uint8_t keys[6])
{
    bridge_keyboard_report_t report;
    s_total_usb_reports++;
    report.modifier = modifier;
    memcpy(report.keys, keys, sizeof(report.keys));

    if (s_keyboard_queue == NULL) {
        s_dropped_no_ble++;
        return;
    }

    if (xQueueSend(s_keyboard_queue, &report, 0) != pdTRUE) {
        s_dropped_no_ble++;
    }
}

static void on_usb_connection(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "USB keyboard READY");
        if (s_led_queue != NULL) {
            xQueueOverwrite(s_led_queue, &s_last_led_state);
        }
    } else {
        ESP_LOGW(TAG, "USB keyboard DISCONNECTED");
    }
    update_led_state();
}

static void on_ble_connection(bool connected)
{
    if (connected) {
        ESP_LOGI(TAG, "BLE HID INPUT READY");
    } else {
        ESP_LOGW(TAG, "BLE HID INPUT NOT READY");
    }
    update_led_state();
}

static void on_ble_leds(uint8_t leds)
{
    s_last_led_state = leds;
    ESP_LOGI(TAG, "BLE LED OUTPUT = 0x%02X [Num=%d Caps=%d Scroll=%d]",
             leds, (leds & 0x01) ? 1 : 0, (leds & 0x02) ? 1 : 0, (leds & 0x04) ? 1 : 0);

    if (s_led_queue != NULL) {
        xQueueOverwrite(s_led_queue, &leds);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 USB -> BLE KEYBOARD BRIDGE V2");

    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    s_keyboard_queue = xQueueCreate(KEYBOARD_QUEUE_LENGTH, sizeof(bridge_keyboard_report_t));
    s_led_queue = xQueueCreate(LED_QUEUE_LENGTH, sizeof(uint8_t));

    const led_rgb_config_t led_cfg = { .gpio_num = 48, .brightness = 40 };
    ESP_ERROR_CHECK(led_rgb_init(&led_cfg));
    led_rgb_set_state(LED_STATE_BOOT);

    xTaskCreate(bridge_keyboard_task, "bridge_keyboard", 4096, NULL, 6, NULL);
    xTaskCreate(bridge_led_task, "bridge_led", 3072, NULL, 5, NULL);

    const ble_hid_config_t ble_cfg = {
        .device_name = "L411 BT",
        .conn_cb = on_ble_connection,
        .leds_cb = on_ble_leds,
    };
    ESP_ERROR_CHECK(ble_hid_init(&ble_cfg));

    const usb_hid_config_t usb_cfg = {
        .report_cb = on_usb_report,
        .char_cb = NULL,
        .conn_cb = on_usb_connection,
    };
    ESP_ERROR_CHECK(usb_hid_init(&usb_cfg));

    update_led_state();
    ESP_LOGI(TAG, "SYSTEM READY");
}