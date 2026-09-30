#include "led_rgb.h"

#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "led_strip.h"

static const char *TAG = "LED_RGB";

/* ============================================================
 * HARDWARE
 * ============================================================ */

static led_strip_handle_t s_strip = NULL;
static uint8_t s_brightness = 40;

/* ============================================================
 * STATE
 * ============================================================ */

static volatile led_state_t s_current_state = LED_STATE_BOOT;
static volatile bool s_key_blink_trigger = false; // Dùng cờ kích hoạt nhạy hơn biến đếm

static TaskHandle_t s_led_task_handle = NULL;

/* ============================================================
 * RGB
 * ============================================================ */

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static const rgb_t state_colors[] = {
    [LED_STATE_BOOT]             = { 0, 50, 100 },
    [LED_STATE_WAIT_USB_AND_BLE] = { 255, 80, 0 },
    [LED_STATE_WAIT_USB]         = { 255, 150, 0 },
    [LED_STATE_WAIT_BLE]         = { 255, 0, 0 },
    [LED_STATE_READY]            = { 0, 255, 0 },
    [LED_STATE_ERROR]            = { 255, 0, 0 }
};

/* ============================================================
 * SET COLOR & OFF
 * ============================================================ */

static void led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_strip == NULL) {
        return;
    }

    uint8_t rr = ((uint16_t)r * s_brightness) / 255;
    uint8_t gg = ((uint16_t)g * s_brightness) / 255;
    uint8_t bb = ((uint16_t)b * s_brightness) / 255;

    led_strip_set_pixel(s_strip, 0, rr, gg, bb);
    led_strip_refresh(s_strip);
}

static void led_off(void)
{
    if (s_strip == NULL) {
        return;
    }
    led_strip_clear(s_strip);
}

/* ============================================================
 * LED TASK
 * ============================================================ */

static void led_task(void *arg)
{
    led_state_t last_state = (led_state_t)-1;
    uint32_t error_tick = 0;

    while (1) {
        /* ----------------------------------------------------
         * PHẢN HỒI GÕ PHÍM (CHỚP XANH NHẸ RỒI TRẢ VỀ TRẠNG THÁI CŨ)
         * ---------------------------------------------------- */
        if (s_key_blink_trigger) {
            s_key_blink_trigger = false;

            // Chớp sáng tức thì báo hiệu đã nhận phím
            led_set_color(180, 255, 180);
            vTaskDelay(pdMS_TO_TICKS(25));

            // Ép vẽ lại trạng thái nền
            last_state = (led_state_t)-1;
        }

        /* ----------------------------------------------------
         * CẬP NHẬT TRẠNG THÁI HỆ THỐNG
         * ---------------------------------------------------- */
        led_state_t state = s_current_state;

        if (state != last_state) {
            last_state = state;

            if (state == LED_STATE_ERROR) {
                led_off();
            } else if (state >= 0 && state < (sizeof(state_colors) / sizeof(state_colors[0]))) {
                rgb_t c = state_colors[state];
                led_set_color(c.r, c.g, c.b);

                ESP_LOGD(TAG, "LED state=%d RGB=%u,%u,%u", state, c.r, c.g, c.b);
            }
        }

        /* ----------------------------------------------------
         * NHẤP NHÁY BÁO LỖI
         * ---------------------------------------------------- */
        if (state == LED_STATE_ERROR) {
            error_tick++;
            if ((error_tick & 1) == 0) {
                led_set_color(255, 0, 0);
            } else {
                led_off();
            }
            vTaskDelay(pdMS_TO_TICKS(250));
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

/* ============================================================
 * INIT
 * ============================================================ */

esp_err_t led_rgb_init(const led_rgb_config_t *config)
{
    int gpio = 48; // GPIO mặc định của LED RGB trên ESP32-S3-DevKitC-1

    if (config != NULL && config->gpio_num >= 0) {
        gpio = config->gpio_num;
    }

    if (config != NULL && config->brightness > 0) {
        s_brightness = config->brightness;
    }

    const led_strip_config_t strip_config = {
        .strip_gpio_num = gpio,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    esp_err_t ret = led_strip_new_rmt_device(
        &strip_config,
        &rmt_config,
        &s_strip
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: 0x%x", ret);
        return ret;
    }

    led_off();

    // SỬA LỖI 1: Tăng stack size lên 3072 bytes chống tràn bộ nhớ
    BaseType_t task_ret = xTaskCreate(
        led_task,
        "led_task",
        3072,
        NULL,
        2,
        &s_led_task_handle
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "LED task creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WS2812 initialized on GPIO %d (Brightness: %u)", gpio, s_brightness);
    return ESP_OK;
}

/* ============================================================
 * STATE & KEY BLINK
 * ============================================================ */

void led_rgb_set_state(led_state_t state)
{
    if (state < LED_STATE_BOOT || state > LED_STATE_ERROR) {
        return;
    }
    s_current_state = state;
}

void led_rgb_blink_key(void)
{
    // Kích hoạt cờ chớp sáng tức thì, không xếp hàng gây trễ
    s_key_blink_trigger = true;
}