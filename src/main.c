#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_strip.h"

static const char *TAG = "MAIN";

#define LED_STRIP_GPIO       38   // Data pin (WS2812_data / IO38)
#define LED_STRIP_NUM_LEDS   8    // 8 chained XL-2121 LEDs

static led_strip_handle_t init_led_strip(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = LED_STRIP_NUM_LEDS,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10 MHz
        .flags.with_dma = false,
    };

    led_strip_handle_t strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
    return strip;
}

// Helper function to set all LEDs across the strip to the same RGB color
static void set_all_leds(led_strip_handle_t strip, uint32_t red, uint32_t green, uint32_t blue)
{
    for (int i = 0; i < LED_STRIP_NUM_LEDS; i++) {
        led_strip_set_pixel(strip, i, red, green, blue);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 booted successfully");
    ESP_LOGI(TAG, "IDF Version: %s", esp_get_idf_version());

    // Initialize the XL-2121 LED strip on IO38
    led_strip_handle_t strip = init_led_strip();
    ESP_LOGI(TAG, "LED strip initialized on IO%d (8 LEDs)", LED_STRIP_GPIO);

    uint32_t counter = 0;

    while (1) {
        int64_t uptime_sec = esp_timer_get_time() / 1000000;
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_heap  = esp_get_minimum_free_heap_size();

        ESP_LOGI(TAG, "[#%" PRIu32 "] Uptime: %" PRId64 "s | Free Heap: %" PRIu32 " B (Min Free: %" PRIu32 " B)",
                 counter, uptime_sec, free_heap, min_heap);

        // Cycle all 8 LEDs through Red -> Green -> Blue -> White
        switch (counter % 4) {
            case 0:
                // Red
                set_all_leds(strip, 50, 0, 0);
                break;
            case 1:
                // Green
                set_all_leds(strip, 0, 50, 0);
                break;
            case 2:
                // Blue
                set_all_leds(strip, 0, 0, 50);
                break;
            case 3:
                // White (dimmed to avoid thermal load across all 8 package bodies)
                set_all_leds(strip, 25, 25, 25);
                break;
        }

        // Clock the entire 192-bit frame (8 LEDs * 24 bits) out over RMT
        ESP_ERROR_CHECK(led_strip_refresh(strip));

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}