/*
 * E-Paper display driver for the Billboard Badge (9.69" 960x672, dual-COG B/W/R)
 *
 * C / ESP-IDF port of the MicroPython driver, which was itself ported from
 * https://github.com/PervasiveDisplays/EPD_Driver_GU_large
 *
 * Original Pervasive Displays copyright and BSD-style license apply to the
 * register sequences; keep their notice with this file when redistributing.
 * Copyright (c) 2012-2021 Pervasive Displays Inc. All rights reserved.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_WIDTH            960
#define EPD_HEIGHT           672
#define EPD_ROW_BYTES        (EPD_WIDTH / 8)            /* 120 */
#define EPD_FB_BYTES         (EPD_ROW_BYTES * EPD_HEIGHT) /* 80,640 per color plane */

/* Pin map (override with -D or in sdkconfig-driven headers if the board changes) */
#ifndef EPD_CS1_PIN
#define EPD_CS1_PIN      13
#define EPD_CS2_PIN      3
#define EPD_BUSY1_PIN    12
#define EPD_BUSY2_PIN    8
#define EPD_DC_PIN       10   /* Data = high, Command = low */
#define EPD_RST_PIN      11
#define EPD_SCL_PIN      21
#define EPD_SDA_PIN      47
#define EPD_PWR_EN_PIN   48
#endif

#ifndef EPD_SPI_HOST
#define EPD_SPI_HOST     SPI2_HOST
#endif
#ifndef EPD_SPI_HZ
#define EPD_SPI_HZ       (20 * 1000 * 1000)
#endif

/* 1 = a refresh is done only when BOTH BUSY lines are high (recommended).
 * 0 = old Python behavior (stop waiting as soon as EITHER line is high). */
#ifndef EPD_BUSY_REQUIRE_BOTH
#define EPD_BUSY_REQUIRE_BOTH 1
#endif

typedef enum {
    EPD_WHITE = 0,
    EPD_BLACK = 1,
    EPD_RED   = 2,
} epd_color_t;

typedef struct {
    /* The two panel part numbers use different bit orders. Python: display_pn == "84"
     * means NO reversal, anything else (default "B1") means reverse. */
    bool reverse_bits;
} epd_config_t;

#define EPD_CONFIG_DEFAULT() { .reverse_bits = true }

/* Init GPIO, SPI, framebuffers. Call once. */
esp_err_t epd_init(const epd_config_t *cfg);

/* Direct access to the framebuffers. Layout matches MicroPython MONO_HLSB:
 * row-major, 120 bytes per row, bit 7 = leftmost pixel, 1 = ink on that plane.
 * Black and red planes should not both be set for the same pixel. */
uint8_t *epd_black_fb(void);
uint8_t *epd_red_fb(void);

void epd_clear(void);                       /* all white */
void epd_fill(epd_color_t color);
void epd_draw_pixel(int x, int y, epd_color_t color);

/* Push the framebuffers to the panel. Blocking (~16-25 s). Thread-safe.
 * Returns ESP_ERR_TIMEOUT if a BUSY wait timed out. */
esp_err_t epd_update(void);

/* True if the last successful update contained no red pixels. */
bool epd_panel_is_monochrome(void);

/* Optional full CoG power-down (port of cog_poweroff). */
esp_err_t epd_poweroff(void);

#ifdef __cplusplus
}
#endif