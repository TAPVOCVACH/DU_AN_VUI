#ifndef BLE_HID_H
#define BLE_HID_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ble_hid_conn_cb_t)(bool connected);
typedef void (*ble_hid_leds_cb_t)(uint8_t leds);

typedef struct {
    const char *device_name;
    ble_hid_conn_cb_t conn_cb;
    ble_hid_leds_cb_t leds_cb;
} ble_hid_config_t;

esp_err_t ble_hid_init(const ble_hid_config_t *config);
esp_err_t ble_hid_send_report(uint8_t modifier, const uint8_t keys[6]);
esp_err_t ble_hid_send_key(uint8_t modifier, uint8_t keycode);
esp_err_t ble_hid_release_all(void);
bool ble_hid_is_connected(void);
esp_err_t ble_hid_set_battery_level(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HID_H */