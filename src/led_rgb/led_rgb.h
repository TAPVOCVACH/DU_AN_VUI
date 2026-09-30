#ifndef LED_RGB_H
#define LED_RGB_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_STATE_BOOT,
    LED_STATE_WAIT_USB_AND_BLE,
    LED_STATE_WAIT_USB,
    LED_STATE_WAIT_BLE,
    LED_STATE_READY,
    LED_STATE_ERROR
} led_state_t;

typedef struct {
    int gpio_num;
    uint8_t brightness;
} led_rgb_config_t;

esp_err_t led_rgb_init(const led_rgb_config_t *config);
void led_rgb_set_state(led_state_t state);
void led_rgb_blink_key(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_RGB_H */