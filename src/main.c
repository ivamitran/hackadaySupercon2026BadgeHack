#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "driver/uart.h"     
#include "driver/gpio.h"
#include "epd_driver.h"
#include "EPD_IMAGE.H"

static const char *TAG = "MAIN";

#define LED_STRIP_GPIO       38   
#define LED_STRIP_NUM_LEDS   8    

// DFPlayer UART Settings
#define DFPLAYER_UART_NUM    UART_NUM_1   
#define DFPLAYER_TX_GPIO     43           
#define DFPLAYER_RX_GPIO     44           
#define DFPLAYER_BAUD_RATE   9600

// DFPlayer Commands
#define DF_CMD_PLAY_TRACK    0x03
#define DF_CMD_SET_VOLUME    0x06

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
        .resolution_hz = 10 * 1000 * 1000, 
        .flags.with_dma = false,
    };

    led_strip_handle_t strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
    ESP_ERROR_CHECK(led_strip_clear(strip));
    return strip;
}

static void set_all_leds(led_strip_handle_t strip, uint32_t red, uint32_t green, uint32_t blue)
{
    for (int i = 0; i < LED_STRIP_NUM_LEDS; i++) {
        led_strip_set_pixel(strip, i, red, green, blue);
    }
}

// Background task: runs continuously regardless of what the display is doing
static void led_task(void *pvParameters)
{
    led_strip_handle_t strip = (led_strip_handle_t)pvParameters;
    uint32_t counter = 0;

    while (1) {
        switch (counter % 3) {
            case 0: set_all_leds(strip, 0, 0, 50); break;   // Blue
            case 1: set_all_leds(strip, 50, 0, 0); break;   // Red
            case 2: set_all_leds(strip, 50, 0, 50); break;  // Purple
        }
        led_strip_refresh(strip);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void init_dfplayer_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = DFPLAYER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(DFPLAYER_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(DFPLAYER_UART_NUM, DFPLAYER_TX_GPIO, DFPLAYER_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(DFPLAYER_UART_NUM, 256, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "DFPlayer UART initialized on TX:%d RX:%d", DFPLAYER_TX_GPIO, DFPLAYER_RX_GPIO);
}

static void dfplayer_send_cmd(uint8_t cmd, uint16_t argument)
{
    uint8_t buf[10];
    buf[0] = 0x7E;  
    buf[1] = 0xFF;  
    buf[2] = 0x06;  
    buf[3] = cmd;   
    buf[4] = 0x00;  
    buf[5] = (uint8_t)(argument >> 8);   
    buf[6] = (uint8_t)(argument & 0xFF); 
    
    uint16_t checksum = 0 - (buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6]);
    buf[7] = (uint8_t)(checksum >> 8);   
    buf[8] = (uint8_t)(checksum & 0xFF); 
    buf[9] = 0xEF;  
    
    uart_write_bytes(DFPLAYER_UART_NUM, (const char *)buf, sizeof(buf));
    ESP_LOGI(TAG, "Sent DFPlayer CMD: 0x%02X Arg: %d", cmd, argument);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 booted successfully");

    // 1. Initialize LEDs & start animating in background
    led_strip_handle_t strip = init_led_strip();
    xTaskCreate(led_task, "led_task", 2048, (void *)strip, 5, NULL);

    // 2. Initialize Audio & trigger playback immediately
    init_dfplayer_uart();
    vTaskDelay(pdMS_TO_TICKS(1500)); 
    dfplayer_send_cmd(DF_CMD_SET_VOLUME, 5);
    vTaskDelay(pdMS_TO_TICKS(100)); 
    dfplayer_send_cmd(DF_CMD_PLAY_TRACK, 1);

        // 3. Initialize E-Paper hardware
    epd_config_t epd_cfg = EPD_CONFIG_DEFAULT();   // set .reverse_bits = false for display_pn "84"
    ESP_ERROR_CHECK(epd_init(&epd_cfg));

    #define IMG_WIDTH        960
    #define IMG_HEIGHT       540
    #define IMG_ROW_BYTES    (IMG_WIDTH / 8)          // 120 bytes per row

    // Center 540 rows vertically on the 672-row canvas: (672 - 540) / 2 = 66
    #define OFFSET_Y         ((EPD_HEIGHT - IMG_HEIGHT) / 2)

    _Static_assert(IMG_WIDTH == EPD_WIDTH, "image width must match panel width");

    // 4. Start from a solid white canvas (0x00 = no pigment on both planes).
    //    The red plane is never written, so it stays blank.
    epd_clear();
    uint8_t *fb_black = epd_black_fb();

    // 5. Copy the image into the canvas, inverting so 1 = black ink.
    //    The driver does the left/right split and bit reversal when sending.
    ESP_LOGI(TAG, "Centering 960x540 image onto 960x672 canvas (Y-offset: %d)...", OFFSET_Y);
    for (int row = 0; row < IMG_HEIGHT; row++) {
        const uint8_t *src = &epd_image[row * IMG_ROW_BYTES];
        uint8_t *dst = &fb_black[(row + OFFSET_Y) * EPD_ROW_BYTES];

        for (int col = 0; col < IMG_ROW_BYTES; col++) {
            dst[col] = src[col] ^ 0xFF;
        }
    }

    // 6. Push image to display (blocks ~16-25 s)
    ESP_LOGI(TAG, "Pushing centered canvas to E-Paper...");
    esp_err_t err = epd_update();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "E-Paper update failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Display update complete.");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}