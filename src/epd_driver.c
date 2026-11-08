/*
 * E-Paper display driver for the Billboard Badge, C / ESP-IDF port.
 * See epd_driver.h for license and provenance notes.
 */
#include "epd_driver.h"

#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "EPD";

#define HALF_ROW_BYTES   (EPD_ROW_BYTES / 2)            /* 60 */
#define ROWS_PER_CHUNK   64
#define DMA_CHUNK_BYTES  (HALF_ROW_BYTES * ROWS_PER_CHUNK) /* 3840 */

#define BUSY_TIMEOUT_REFRESH_MS  35000  /* 3-color refresh takes ~16-25 s */
#define BUSY_TIMEOUT_SHORT_MS    5000

typedef enum {
    TARGET_BOTH,
    TARGET_LEFT,
    TARGET_RIGHT,
} target_t;

static spi_device_handle_t s_spi;
static SemaphoreHandle_t   s_lock;
static uint8_t *s_fb_black;
static uint8_t *s_fb_red;
static uint8_t *s_dma_buf;          /* internal, DMA-capable bounce buffer */
static uint8_t  s_rev_lut[256];
static bool     s_reverse_bits = true;
static bool     s_panel_monochrome = false;
static bool     s_inited = false;

/* ------------------------------------------------------------------ */
/* Timing helpers                                                      */
/* ------------------------------------------------------------------ */

/* vTaskDelay(pdMS_TO_TICKS(2)) is 0 ticks at a 100 Hz tick, so short delays
 * are done with a busy-wait and only longer ones yield to the scheduler. */
static void delay_ms(uint32_t ms)
{
    if (ms >= 2 * portTICK_PERIOD_MS) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        esp_rom_delay_us(ms * 1000);
    }
}

static void poll_sleep(void)
{
    TickType_t t = pdMS_TO_TICKS(20);
    vTaskDelay(t ? t : 1);
}

static bool busy_ready(void)
{
    int b1 = gpio_get_level(EPD_BUSY1_PIN);
    int b2 = gpio_get_level(EPD_BUSY2_PIN);
#if EPD_BUSY_REQUIRE_BOTH
    return b1 && b2;
#else
    return b1 || b2;
#endif
}

static esp_err_t wait_busy(uint32_t timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (!busy_ready()) {
        if (esp_timer_get_time() >= deadline) {
            ESP_LOGW(TAG, "BUSY wait timed out after %" PRIu32 " ms", timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        poll_sleep();
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Low-level SPI                                                       */
/* ------------------------------------------------------------------ */

static inline void cs_assert(target_t t)
{
    gpio_set_level(EPD_CS1_PIN, (t == TARGET_BOTH || t == TARGET_LEFT)  ? 0 : 1);
    gpio_set_level(EPD_CS2_PIN, (t == TARGET_BOTH || t == TARGET_RIGHT) ? 0 : 1);
}

static inline void cs_release(void)
{
    gpio_set_level(EPD_CS1_PIN, 1);
    gpio_set_level(EPD_CS2_PIN, 1);
}

/* Transmit len bytes that are already inside s_dma_buf. */
static esp_err_t spi_tx_dma(size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = s_dma_buf,
    };
    return spi_device_polling_transmit(s_spi, &t);
}

/* Copy arbitrary (possibly non-DMA) data through the bounce buffer. */
static esp_err_t spi_write_copy(const uint8_t *data, size_t len)
{
    while (len) {
        size_t n = len > DMA_CHUNK_BYTES ? DMA_CHUNK_BYTES : len;
        memcpy(s_dma_buf, data, n);
        esp_err_t err = spi_tx_dma(n);
        if (err != ESP_OK) {
            return err;
        }
        data += n;
        len  -= n;
    }
    return ESP_OK;
}

/* Index byte (DC low) then data bytes (DC high), CS toggled around each. */
static esp_err_t send_cmd(target_t t, uint8_t index, const uint8_t *data, size_t len)
{
    esp_err_t err;

    gpio_set_level(EPD_DC_PIN, 0);
    cs_assert(t);
    err = spi_write_copy(&index, 1);
    cs_release();
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(EPD_DC_PIN, 1);
    cs_assert(t);
    err = spi_write_copy(data, len);
    cs_release();
    return err;
}

static void cmd1(target_t t, uint8_t index, uint8_t a)
{
    send_cmd(t, index, &a, 1);
}

static void cmd2(target_t t, uint8_t index, uint8_t a, uint8_t b)
{
    uint8_t d[2] = { a, b };
    send_cmd(t, index, d, 2);
}

/* Send one half (left or right) of a full-width framebuffer plane, splitting
 * the 120-byte rows into 60-byte halves and optionally reversing bit order. */
static esp_err_t send_plane(target_t t, uint8_t index, const uint8_t *fb)
{
    const size_t col = (t == TARGET_RIGHT) ? HALF_ROW_BYTES : 0;
    esp_err_t err;

    gpio_set_level(EPD_DC_PIN, 0);
    cs_assert(t);
    err = spi_write_copy(&index, 1);
    cs_release();
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(EPD_DC_PIN, 1);
    cs_assert(t);

    size_t fill = 0;
    for (int r = 0; r < EPD_HEIGHT; r++) {
        const uint8_t *src = fb + (size_t)r * EPD_ROW_BYTES + col;
        uint8_t *dst = s_dma_buf + fill;
        if (s_reverse_bits) {
            for (int c = 0; c < HALF_ROW_BYTES; c++) {
                dst[c] = s_rev_lut[src[c]];
            }
        } else {
            memcpy(dst, src, HALF_ROW_BYTES);
        }
        fill += HALF_ROW_BYTES;

        if (fill + HALF_ROW_BYTES > DMA_CHUNK_BYTES || r == EPD_HEIGHT - 1) {
            err = spi_tx_dma(fill);
            fill = 0;
            if (err != ESP_OK) {
                break;
            }
        }
    }
    cs_release();
    return err;
}

/* ------------------------------------------------------------------ */
/* Panel sequences (ported 1:1 from the Python driver)                 */
/* ------------------------------------------------------------------ */

static void epd_reset(void)
{
    delay_ms(200);
    gpio_set_level(EPD_RST_PIN, 1);
    delay_ms(20);
    gpio_set_level(EPD_RST_PIN, 0);
    delay_ms(200);
    gpio_set_level(EPD_RST_PIN, 1);
    delay_ms(200);
    cs_release();
    delay_ms(5);
    cmd2(TARGET_BOTH, 0x70, 0x71, 0x36);   /* B1 booster config, itc97; Errata 9 */
}

static void epd_send_images(void)
{
    const uint8_t duw[]  = { 0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02 };
    const uint8_t drfw[] = { 0x00, 0x3B, 0x00, 0xA9 };
    const uint8_t ram_rw[] = { 0x3B, 0x00, 0x14 };

    send_cmd(TARGET_BOTH, 0x13, duw, sizeof(duw));
    send_cmd(TARGET_BOTH, 0x90, drfw, sizeof(drfw));

    send_cmd(TARGET_LEFT, 0x12, ram_rw, sizeof(ram_rw));
    send_plane(TARGET_LEFT, 0x10, s_fb_black);
    send_cmd(TARGET_LEFT, 0x12, ram_rw, sizeof(ram_rw));
    send_plane(TARGET_LEFT, 0x11, s_fb_red);

    send_cmd(TARGET_RIGHT, 0x12, ram_rw, sizeof(ram_rw));
    send_plane(TARGET_RIGHT, 0x10, s_fb_black);
    send_cmd(TARGET_RIGHT, 0x12, ram_rw, sizeof(ram_rw));
    send_plane(TARGET_RIGHT, 0x11, s_fb_red);
}

/* One temperature/OSC calibration pass for a single COG. */
static void cog_side_init(target_t t)
{
    cmd1(t, 0x44, 0x00);
    cmd1(t, 0x45, 0x80);
    cmd1(t, 0xA7, 0x10);
    delay_ms(100);
    cmd1(t, 0xA7, 0x00);
    delay_ms(100);
    cmd1(t, 0x44, 0x06);
    cmd1(t, 0x45, 0x82);        /* Temperature 0x82 @ 25C */
    cmd1(t, 0xA7, 0x10);
    delay_ms(100);
    cmd1(t, 0xA7, 0x00);
    delay_ms(100);
}

static void epd_cog_initial(void)
{
    cmd1(TARGET_BOTH, 0x05, 0x7D);
    delay_ms(200);
    cmd1(TARGET_BOTH, 0x05, 0x00);
    delay_ms(10);
    cmd1(TARGET_BOTH, 0xD8, 0x80);   /* MS_SYNC */
    cmd1(TARGET_BOTH, 0xD6, 0x00);   /* BVSS */
    cmd1(TARGET_BOTH, 0xA7, 0x10);
    delay_ms(100);
    cmd1(TARGET_BOTH, 0xA7, 0x00);
    delay_ms(100);
    cmd2(TARGET_BOTH, 0x03, 0x00, 0x11);   /* OSC */

    cog_side_init(TARGET_LEFT);
    cog_side_init(TARGET_RIGHT);

    cmd1(TARGET_BOTH, 0x60, 0x25);   /* TCON */
    cmd1(TARGET_LEFT, 0x61, 0x01);   /* STV_DIR for display 1 */
    cmd1(TARGET_BOTH, 0x02, 0x00);   /* VCOM */
}

static void epd_dcdc_soft_start(void)
{
    cmd2(TARGET_BOTH, 0x51, 0x50, 0x01);

    for (uint8_t v = 1; v < 5; v++) {
        cmd1(TARGET_BOTH, 0x09, 0x1F);
        cmd2(TARGET_BOTH, 0x51, 0x50, v);
        cmd1(TARGET_BOTH, 0x09, 0x9F);
        delay_ms(2);
    }
    for (uint8_t v = 1; v < 11; v++) {
        cmd1(TARGET_BOTH, 0x09, 0x1F);
        cmd2(TARGET_BOTH, 0x51, 0x0A, v);
        cmd1(TARGET_BOTH, 0x09, 0x9F);
        delay_ms(2);
    }
    for (uint8_t v = 3; v < 11; v++) {
        cmd1(TARGET_BOTH, 0x09, 0x7F);
        cmd2(TARGET_BOTH, 0x51, 0x0A, v);
        cmd1(TARGET_BOTH, 0x09, 0xFF);
        delay_ms(2);
    }
    for (uint8_t v = 9; v > 1; v--) {
        cmd1(TARGET_BOTH, 0x09, 0x7F);
        cmd2(TARGET_BOTH, 0x51, v, 0x0A);
        cmd1(TARGET_BOTH, 0x09, 0xFF);
        delay_ms(2);
    }
    cmd1(TARGET_BOTH, 0x09, 0xFF);
    delay_ms(10);
}

static esp_err_t epd_display_refresh(void)
{
    esp_err_t err = wait_busy(BUSY_TIMEOUT_SHORT_MS);
    cmd1(TARGET_BOTH, 0x15, 0x3C);   /* trigger refresh waveform */
    /* Python waits only 5 ms here. Give the controller time to actually pull
     * BUSY low before shutdown starts polling, or the shutdown wait can fall
     * straight through while the waveform is still running. */
    delay_ms(500);
    return err;
}

static esp_err_t epd_dcdc_soft_shutdown(void)
{
    esp_err_t err = wait_busy(BUSY_TIMEOUT_REFRESH_MS);
    cmd1(TARGET_BOTH, 0x09, 0x7F);
    cmd1(TARGET_BOTH, 0x05, 0x7D);
    cmd1(TARGET_BOTH, 0x09, 0x00);
    delay_ms(200);
    esp_err_t err2 = wait_busy(BUSY_TIMEOUT_SHORT_MS);

    gpio_set_level(EPD_DC_PIN, 0);
    gpio_set_level(EPD_CS1_PIN, 0);
    gpio_set_level(EPD_RST_PIN, 0);
    gpio_set_level(EPD_CS1_PIN, 1);
    return err != ESP_OK ? err : err2;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

static void gpio_out(int pin, int level)
{
    gpio_reset_pin(pin);
    gpio_set_level(pin, level);          /* set level before enabling the driver */
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
}

static void gpio_in(int pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_INPUT);
}

static uint8_t *alloc_fb(void)
{
    uint8_t *p = heap_caps_calloc(1, EPD_FB_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_calloc(1, EPD_FB_BYTES, MALLOC_CAP_8BIT);
    }
    return p;
}

esp_err_t epd_init(const epd_config_t *cfg)
{
    if (s_inited) {
        return ESP_OK;
    }
    s_reverse_bits = cfg ? cfg->reverse_bits : true;

    for (int i = 0; i < 256; i++) {
        uint8_t b = (uint8_t)i;
        b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
        b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
        b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
        s_rev_lut[i] = b;
    }

    s_lock = xSemaphoreCreateMutex();
    s_dma_buf = heap_caps_malloc(DMA_CHUNK_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_fb_black = alloc_fb();
    s_fb_red = alloc_fb();
    if (!s_lock || !s_dma_buf || !s_fb_black || !s_fb_red) {
        ESP_LOGE(TAG, "out of memory");
        return ESP_ERR_NO_MEM;
    }

    gpio_out(EPD_CS1_PIN, 1);
    gpio_out(EPD_CS2_PIN, 1);
    gpio_out(EPD_DC_PIN, 1);
    gpio_out(EPD_RST_PIN, 1);
    gpio_out(EPD_PWR_EN_PIN, 0);
    gpio_in(EPD_BUSY1_PIN);
    gpio_in(EPD_BUSY2_PIN);

    spi_bus_config_t buscfg = {
        .sclk_io_num = EPD_SCL_PIN,
        .mosi_io_num = EPD_SDA_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DMA_CHUNK_BYTES,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = EPD_SPI_HZ,
        .mode = 0,
        .spics_io_num = -1,          /* CS1/CS2 are driven manually */
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(EPD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(EPD_SPI_HOST, &devcfg, &s_spi));

    s_inited = true;
    ESP_LOGI(TAG, "init ok (reverse_bits=%d)", s_reverse_bits);
    return ESP_OK;
}

uint8_t *epd_black_fb(void) { return s_fb_black; }
uint8_t *epd_red_fb(void)   { return s_fb_red; }

void epd_clear(void)
{
    memset(s_fb_black, 0, EPD_FB_BYTES);
    memset(s_fb_red, 0, EPD_FB_BYTES);
}

void epd_fill(epd_color_t color)
{
    memset(s_fb_black, color == EPD_BLACK ? 0xFF : 0x00, EPD_FB_BYTES);
    memset(s_fb_red,   color == EPD_RED   ? 0xFF : 0x00, EPD_FB_BYTES);
}

void epd_draw_pixel(int x, int y, epd_color_t color)
{
    if (x < 0 || y < 0 || x >= EPD_WIDTH || y >= EPD_HEIGHT) {
        return;
    }
    size_t idx = (size_t)y * EPD_ROW_BYTES + (x >> 3);
    uint8_t mask = (uint8_t)(0x80 >> (x & 7));
    s_fb_black[idx] &= (uint8_t)~mask;
    s_fb_red[idx]   &= (uint8_t)~mask;
    if (color == EPD_BLACK) {
        s_fb_black[idx] |= mask;
    } else if (color == EPD_RED) {
        s_fb_red[idx] |= mask;
    }
}

bool epd_panel_is_monochrome(void) { return s_panel_monochrome; }

esp_err_t epd_update(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);

    bool has_red = false;
    const uint32_t *w = (const uint32_t *)s_fb_red;   /* calloc'd: 4-byte aligned */
    for (size_t i = 0; i < EPD_FB_BYTES / 4; i++) {
        if (w[i]) { has_red = true; break; }
    }
    s_panel_monochrome = false;

    int64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "Updating display to new image...");

    epd_reset();
    epd_send_images();
    ESP_LOGI(TAG, "image upload done (%lld ms)", (long long)((esp_timer_get_time() - t0) / 1000));

    epd_cog_initial();
    epd_dcdc_soft_start();
    esp_err_t err = epd_display_refresh();
    esp_err_t err2 = epd_dcdc_soft_shutdown();
    if (err == ESP_OK) {
        err = err2;
    }

    ESP_LOGI(TAG, "Done updating display (%lld ms, %s)",
             (long long)((esp_timer_get_time() - t0) / 1000), esp_err_to_name(err));

    if (err == ESP_OK) {
        s_panel_monochrome = !has_red;
    }
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t epd_poweroff(void)
{
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const uint8_t d[] = { 0x7F, 0x7D, 0x00 };
    send_cmd(TARGET_BOTH, 0x09, d, sizeof(d));
    delay_ms(200);
    esp_err_t err = wait_busy(BUSY_TIMEOUT_SHORT_MS);
    gpio_set_level(EPD_DC_PIN, 0);
    gpio_set_level(EPD_CS1_PIN, 0);
    delay_ms(150);
    gpio_set_level(EPD_RST_PIN, 0);
    xSemaphoreGive(s_lock);
    return err;
}