#ifndef USB_HID_H
#define USB_HID_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*usb_hid_report_cb_t)(uint8_t modifier, const uint8_t keys[6]);
typedef void (*usb_hid_char_cb_t)(char ch, uint8_t keycode);
typedef void (*usb_hid_conn_cb_t)(bool connected);

typedef struct {
    usb_hid_report_cb_t report_cb;
    usb_hid_char_cb_t   char_cb;
    usb_hid_conn_cb_t   conn_cb;
} usb_hid_config_t;

esp_err_t usb_hid_init(const usb_hid_config_t *config);
bool usb_hid_is_connected(void);
char usb_hid_keycode_to_ascii(uint8_t keycode, bool shift_pressed);
esp_err_t usb_hid_set_leds(uint8_t leds);

#ifdef __cplusplus
}
#endif

#endif /* USB_HID_H */