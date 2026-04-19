/*
 * oled.c — SSD1306 128×64 driver for ESP-IDF / FreeRTOS
 *
 * Improvements over original:
 *  - Fixed font table: N–Z uppercase were off-by-one; many lowercase glyphs
 *    were wrong or zeroed (x, y, z completely missing).
 *  - Thread-safe: every public function is guarded by a FreeRTOS mutex.
 *  - oled_flush() is now public so callers control when the buffer is pushed.
 *  - New primitives: pixel, h/v-line, Bresenham line, rect, fill_rect,
 *    circle, fill_circle, bitmap blitting.
 *  - Text helpers: oled_text_width(), oled_display_text_centered(),
 *    oled_display_lines().
 *  - Display-control commands: contrast, invert, power on/off.
 *  - oled_display_text() now centres both lines in their vertical half
 *    instead of pinning them to pages 0 and 2.
 *  - Clip guard in oled_draw_char() corrected (was >= 124, now > 122).
 */

#include "oled.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "hw_config.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "OLED";

// ─────────────────────────────────────────────
//  Internal state
// ─────────────────────────────────────────────
static uint8_t           s_buf[OLED_WIDTH * OLED_PAGES];
static SemaphoreHandle_t s_mutex = NULL;

#define OLED_LOCK()   xSemaphoreTake(s_mutex, portMAX_DELAY)
#define OLED_UNLOCK() xSemaphoreGive(s_mutex)

// ─────────────────────────────────────────────
//  5×7 font  (ASCII 32–126, 5 bytes per glyph)
//
//  Original had the entire uppercase block from N onwards shifted by
//  one slot (each letter held the next letter's bitmap). Fixed here.
//  Several lowercase glyphs were also wrong or zeroed — all corrected.
// ─────────────────────────────────────────────
static const uint8_t s_font5x7[] = {
    0x00,0x00,0x00,0x00,0x00, //   (space)
    0x00,0x00,0x5F,0x00,0x00, // !
    0x00,0x07,0x00,0x07,0x00, // "
    0x14,0x7F,0x14,0x7F,0x14, // #
    0x24,0x2A,0x7F,0x2A,0x12, // $
    0x23,0x13,0x08,0x64,0x62, // %
    0x36,0x49,0x55,0x22,0x50, // &
    0x00,0x05,0x03,0x00,0x00, // '
    0x00,0x1C,0x22,0x41,0x00, // (
    0x00,0x41,0x22,0x1C,0x00, // )
    0x14,0x08,0x3E,0x08,0x14, // *
    0x08,0x08,0x3E,0x08,0x08, // +
    0x00,0x50,0x30,0x00,0x00, // ,
    0x08,0x08,0x08,0x08,0x08, // -
    0x00,0x60,0x60,0x00,0x00, // .
    0x20,0x10,0x08,0x04,0x02, // /
    0x3E,0x51,0x49,0x45,0x3E, // 0
    0x00,0x42,0x7F,0x40,0x00, // 1
    0x42,0x61,0x51,0x49,0x46, // 2
    0x21,0x41,0x45,0x4B,0x31, // 3
    0x18,0x14,0x12,0x7F,0x10, // 4
    0x27,0x45,0x45,0x45,0x39, // 5
    0x3C,0x4A,0x49,0x49,0x30, // 6
    0x01,0x71,0x09,0x05,0x03, // 7
    0x36,0x49,0x49,0x49,0x36, // 8
    0x06,0x49,0x49,0x29,0x1E, // 9
    0x00,0x36,0x36,0x00,0x00, // :
    0x00,0x56,0x36,0x00,0x00, // ;
    0x08,0x14,0x22,0x41,0x00, // <
    0x14,0x14,0x14,0x14,0x14, // =
    0x00,0x41,0x22,0x14,0x08, // >
    0x02,0x01,0x59,0x09,0x06, // ?
    0x3E,0x41,0x5D,0x59,0x4E, // @
    /* ── Uppercase ── */
    0x7C,0x12,0x11,0x12,0x7C, // A
    0x7F,0x49,0x49,0x49,0x36, // B
    0x3E,0x41,0x41,0x41,0x22, // C
    0x7F,0x41,0x41,0x22,0x1C, // D
    0x7F,0x49,0x49,0x49,0x41, // E
    0x7F,0x09,0x09,0x09,0x01, // F
    0x3E,0x41,0x49,0x49,0x7A, // G
    0x7F,0x08,0x08,0x08,0x7F, // H
    0x00,0x41,0x7F,0x41,0x00, // I
    0x20,0x40,0x41,0x3F,0x01, // J
    0x7F,0x08,0x14,0x22,0x41, // K
    0x7F,0x40,0x40,0x40,0x40, // L
    0x7F,0x02,0x0C,0x02,0x7F, // M
    /* FIX: was 0x3E,0x41,0x41,0x41,0x3E (that is O's glyph) */
    0x7F,0x04,0x08,0x10,0x7F, // N ← corrected
    0x3E,0x41,0x41,0x41,0x3E, // O ← corrected
    0x7F,0x09,0x09,0x09,0x06, // P ← corrected
    0x3E,0x41,0x51,0x21,0x5E, // Q ← corrected
    0x7F,0x09,0x19,0x29,0x46, // R ← corrected
    0x46,0x49,0x49,0x49,0x31, // S ← corrected
    0x01,0x01,0x7F,0x01,0x01, // T ← corrected
    0x3F,0x40,0x40,0x40,0x3F, // U ← corrected
    0x1F,0x20,0x40,0x20,0x1F, // V ← corrected
    0x7F,0x20,0x18,0x20,0x7F, // W ← corrected
    0x63,0x14,0x08,0x14,0x63, // X ← corrected
    0x03,0x04,0x78,0x04,0x03, // Y ← corrected
    0x61,0x51,0x49,0x45,0x43, // Z ← corrected
    /* ── Punctuation between Z and a ── */
    0x00,0x7F,0x41,0x41,0x00, // [
    0x02,0x04,0x08,0x10,0x20, // backslash
    0x00,0x41,0x41,0x7F,0x00, // ]
    0x04,0x02,0x01,0x02,0x04, // ^
    0x40,0x40,0x40,0x40,0x40, // _
    0x00,0x01,0x02,0x04,0x00, // `
    /* ── Lowercase (all corrected/filled) ── */
    0x20,0x54,0x54,0x54,0x78, // a
    0x7F,0x48,0x44,0x44,0x38, // b
    0x38,0x44,0x44,0x44,0x20, // c
    0x38,0x44,0x44,0x48,0x7F, // d
    0x38,0x54,0x54,0x54,0x18, // e
    0x08,0x7E,0x09,0x01,0x02, // f
    0x0C,0x52,0x52,0x52,0x3E, // g
    0x7F,0x08,0x04,0x04,0x78, // h
    0x00,0x44,0x7D,0x40,0x00, // i
    0x20,0x40,0x44,0x3D,0x00, // j
    0x7F,0x10,0x28,0x44,0x00, // k
    0x00,0x41,0x7F,0x40,0x00, // l  ← corrected (was m's glyph)
    0x7C,0x04,0x18,0x04,0x78, // m  ← corrected
    0x7C,0x08,0x04,0x04,0x78, // n  ← corrected
    0x38,0x44,0x44,0x44,0x38, // o  ← corrected
    0x7C,0x14,0x14,0x14,0x08, // p  ← corrected
    0x08,0x14,0x14,0x18,0x7C, // q  ← corrected
    0x7C,0x08,0x04,0x04,0x00, // r  ← corrected
    0x48,0x54,0x54,0x54,0x20, // s  ← corrected
    0x04,0x3F,0x44,0x40,0x20, // t  ← corrected
    0x3C,0x40,0x40,0x20,0x7C, // u  ← corrected
    0x1C,0x20,0x40,0x20,0x1C, // v  ← corrected
    0x3C,0x40,0x30,0x40,0x3C, // w  ← corrected
    0x44,0x28,0x10,0x28,0x44, // x  ← was 0x00 (missing)
    0x0C,0x50,0x50,0x50,0x3C, // y  ← was 0x00 (missing)
    0x44,0x64,0x54,0x4C,0x44, // z  ← was 0x00 (missing)
    0x00,0x08,0x36,0x41,0x00, // {
    0x00,0x00,0x7F,0x00,0x00, // |
    0x00,0x41,0x36,0x08,0x00, // }
    0x02,0x01,0x02,0x04,0x02, // ~
};

// ─────────────────────────────────────────────
//  Low-level I2C helpers
// ─────────────────────────────────────────────
static esp_err_t i2c_write_cmd(uint8_t command)
{
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    if (!h) return ESP_ERR_NO_MEM;

    i2c_master_start(h);
    i2c_master_write_byte(h, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x00, true);   // Co=0, D/C#=0 → command
    i2c_master_write_byte(h, command, true);
    i2c_master_stop(h);

    esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, h, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(h);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "i2c_write_cmd(0x%02x) failed: 0x%x", command, ret);
    }
    return ret;
}

static esp_err_t i2c_write_data(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    i2c_cmd_handle_t h = i2c_cmd_link_create();
    if (!h) return ESP_ERR_NO_MEM;

    i2c_master_start(h);
    i2c_master_write_byte(h, (OLED_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x40, true);   // Co=0, D/C#=1 → data
    i2c_master_write(h, data, len, true);
    i2c_master_stop(h);

    esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, h, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(h);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "i2c_write_data(%u bytes) failed: 0x%x", (unsigned)len, ret);
    }
    return ret;
}

static void send_cmd_list(const uint8_t *cmds, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        esp_err_t ret = i2c_write_cmd(cmds[i]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "send_cmd_list[%u] = 0x%02x failed: 0x%x",
                     (unsigned)i, cmds[i], ret);
        }
    }
}

// ─────────────────────────────────────────────
//  Internal: push buffer to hardware
// ─────────────────────────────────────────────
static void flush_locked(void)
{
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        uint8_t header[3] = {
            (uint8_t)(0xB0 | page), // page address
            0x00,                   // lower column = 0
            0x10,                   // upper column = 0
        };
        send_cmd_list(header, sizeof(header));
        esp_err_t ret = i2c_write_data(&s_buf[page * OLED_WIDTH], OLED_WIDTH);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "flush page %d failed: 0x%x", page, ret);
        }
    }
}

// ─────────────────────────────────────────────
//  Internal: draw single character into buffer (no mutex — callers hold it)
// ─────────────────────────────────────────────
static void draw_char_locked(uint8_t x, uint8_t page, char c)
{
    if (x > OLED_WIDTH - 6 || page >= OLED_PAGES) return; // clip

    if (c < 32 || c > 126) c = '?';
    uint32_t idx = (uint32_t)(c - 32) * 5;
    uint32_t base = (uint32_t)page * OLED_WIDTH + x;

    for (uint8_t i = 0; i < 5; i++) {
        s_buf[base + i] = s_font5x7[idx + i];
    }
    s_buf[base + 5] = 0x00; // 1-pixel inter-character gap
}

// ─────────────────────────────────────────────
//  oled_init
// ─────────────────────────────────────────────
void oled_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = OLED_I2C_SDA_GPIO,
        .scl_io_num       = OLED_I2C_SCL_GPIO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(OLED_I2C_PORT, &cfg));
    ESP_ERROR_CHECK(i2c_driver_install(OLED_I2C_PORT, cfg.mode, 0, 0, 0));

    static const uint8_t init_cmds[] = {
        0xAE,       // display off
        0x20, 0x00, // horizontal addressing mode
        0xB0,       // page start = 0
        0xC8,       // COM scan direction (flip vertical)
        0x00,       // lower column = 0
        0x10,       // upper column = 0
        0x40,       // start line = 0
        0x81, 0x7F, // contrast = 127
        0xA1,       // segment remap
        0xA6,       // normal (non-inverted)
        0xA8, 0x3F, // multiplex ratio = 64
        0xA4,       // display follows RAM
        0xD3, 0x00, // display offset = 0
        0xD5, 0x80, // clock divide ratio / oscillator freq
        0xD9, 0x22, // pre-charge period
        0xDA, 0x12, // COM pins hardware config
        0xDB, 0x20, // VCOMH deselect level
        0x8D, 0x14, // enable charge pump
        0xAF,       // display on
    };
    send_cmd_list(init_cmds, sizeof(init_cmds));
    vTaskDelay(pdMS_TO_TICKS(100));

    memset(s_buf, 0, sizeof(s_buf));
    flush_locked();

    ESP_LOGI(TAG, "OLED initialised (%dx%d)", OLED_WIDTH, OLED_HEIGHT);
}

// ─────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────
void oled_clear(void)
{
    OLED_LOCK();
    memset(s_buf, 0, sizeof(s_buf));
    OLED_UNLOCK();
}

void oled_flush(void)
{
    OLED_LOCK();
    flush_locked();
    OLED_UNLOCK();
}

void oled_clear_and_flush(void)
{
    OLED_LOCK();
    memset(s_buf, 0, sizeof(s_buf));
    flush_locked();
    OLED_UNLOCK();
}

// ─────────────────────────────────────────────
//  Display control
// ─────────────────────────────────────────────
void oled_set_contrast(uint8_t contrast)
{
    OLED_LOCK();
    i2c_write_cmd(0x81);
    i2c_write_cmd(contrast);
    OLED_UNLOCK();
}

void oled_set_invert(bool invert)
{
    OLED_LOCK();
    i2c_write_cmd(invert ? 0xA7 : 0xA6);
    OLED_UNLOCK();
}

void oled_set_power(bool on)
{
    OLED_LOCK();
    i2c_write_cmd(on ? 0xAF : 0xAE);
    OLED_UNLOCK();
}

// ─────────────────────────────────────────────
//  Text
// ─────────────────────────────────────────────
void oled_draw_text_at(uint8_t x, uint8_t page, const char *text)
{
    if (!text) return;
    OLED_LOCK();
    while (*text && x <= OLED_WIDTH - 6) {
        draw_char_locked(x, page, *text++);
        x += 6;
    }
    OLED_UNLOCK();
}

uint8_t oled_text_width(const char *text)
{
    if (!text || *text == '\0') return 0;
    size_t len = strlen(text);
    uint16_t w = (uint16_t)len * 6;
    return (uint8_t)(w > 255 ? 255 : w);
}

/*  oled_display_text:
 *  Clears, draws line1 centred in the top half (pages 1–3),
 *  draws line2 centred in the bottom half (pages 4–6), then flushes.
 *
 *  Previous version hardcoded pages 0 and 2 with no horizontal centring.
 */
void oled_display_text(const char *line1, const char *line2)
{
    if (!line1 || !line2) {
        ESP_LOGW(TAG, "oled_display_text: NULL pointer");
        return;
    }

    OLED_LOCK();
    memset(s_buf, 0, sizeof(s_buf));

    uint8_t w1 = (uint8_t)(strlen(line1) * 6);
    uint8_t x1 = (w1 < OLED_WIDTH) ? (OLED_WIDTH - w1) / 2 : 0;

    uint8_t w2 = (uint8_t)(strlen(line2) * 6);
    uint8_t x2 = (w2 < OLED_WIDTH) ? (OLED_WIDTH - w2) / 2 : 0;

    // Draw without re-acquiring the mutex (we already hold it)
    const char *p = line1;
    uint8_t cx = x1;
    while (*p && cx <= OLED_WIDTH - 6) { draw_char_locked(cx, 1, *p++); cx += 6; }

    p  = line2;
    cx = x2;
    while (*p && cx <= OLED_WIDTH - 6) { draw_char_locked(cx, 5, *p++); cx += 6; }

    flush_locked();
    OLED_UNLOCK();
}

void oled_display_text_centered(const char *line1, const char *line2)
{
    // Identical logic to oled_display_text — centring is already built in.
    oled_display_text(line1, line2);
}

void oled_display_lines(const char * const *lines, uint8_t count)
{
    if (!lines || count == 0) return;
    if (count > OLED_PAGES) count = OLED_PAGES;

    OLED_LOCK();
    memset(s_buf, 0, sizeof(s_buf));
    for (uint8_t i = 0; i < count; i++) {
        if (!lines[i]) continue;
        const char *p = lines[i];
        uint8_t x = 0;
        while (*p && x <= OLED_WIDTH - 6) {
            draw_char_locked(x, i, *p++);
            x += 6;
        }
    }
    flush_locked();
    OLED_UNLOCK();
}

// ─────────────────────────────────────────────
//  Drawing primitives
// ─────────────────────────────────────────────

/*  All primitives write to the buffer only.
 *  Call oled_flush() to make them visible.
 */

void oled_draw_pixel(uint8_t x, uint8_t y, bool set)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    OLED_LOCK();
    uint32_t byte_idx = (uint32_t)(y / 8) * OLED_WIDTH + x;
    uint8_t  bit_mask = (uint8_t)(1 << (y & 7));
    if (set) {
        s_buf[byte_idx] |= bit_mask;
    } else {
        s_buf[byte_idx] &= (uint8_t)~bit_mask;
    }
    OLED_UNLOCK();
}

void oled_draw_hline(uint8_t x, uint8_t y, uint8_t w)
{
    if (y >= OLED_HEIGHT || x >= OLED_WIDTH) return;
    if ((uint16_t)x + w > OLED_WIDTH) w = (uint8_t)(OLED_WIDTH - x);

    OLED_LOCK();
    uint8_t page = y / 8;
    uint8_t bit  = (uint8_t)(1 << (y & 7));
    for (uint8_t i = 0; i < w; i++) {
        s_buf[(uint32_t)page * OLED_WIDTH + x + i] |= bit;
    }
    OLED_UNLOCK();
}

void oled_draw_vline(uint8_t x, uint8_t y, uint8_t h)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    OLED_LOCK();
    for (uint8_t row = y; row < y + h && row < OLED_HEIGHT; row++) {
        uint32_t idx = (uint32_t)(row / 8) * OLED_WIDTH + x;
        s_buf[idx] |= (uint8_t)(1 << (row & 7));
    }
    OLED_UNLOCK();
}

void oled_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    /* Bresenham — avoids floating point */
    int dx  =  abs((int)x1 - x0);
    int dy  = -abs((int)y1 - y0);
    int sx  = (x0 < x1) ? 1 : -1;
    int sy  = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    OLED_LOCK();
    while (1) {
        if (x0 < OLED_WIDTH && y0 < OLED_HEIGHT) {
            s_buf[(uint32_t)(y0 / 8) * OLED_WIDTH + x0] |= (uint8_t)(1 << (y0 & 7));
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 = (uint8_t)(x0 + sx); }
        if (e2 <= dx) { err += dx; y0 = (uint8_t)(y0 + sy); }
    }
    OLED_UNLOCK();
}

void oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    oled_draw_hline(x,         y,         w);
    oled_draw_hline(x,         y + h - 1, w);
    oled_draw_vline(x,         y,         h);
    oled_draw_vline(x + w - 1, y,         h);
}

void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    if ((uint16_t)x + w > OLED_WIDTH)  w = (uint8_t)(OLED_WIDTH  - x);
    if ((uint16_t)y + h > OLED_HEIGHT) h = (uint8_t)(OLED_HEIGHT - y);

    OLED_LOCK();
    for (uint8_t row = y; row < y + h; row++) {
        uint32_t idx = (uint32_t)(row / 8) * OLED_WIDTH + x;
        uint8_t  bit = (uint8_t)(1 << (row & 7));
        for (uint8_t col = 0; col < w; col++) {
            s_buf[idx + col] |= bit;
        }
    }
    OLED_UNLOCK();
}

/* Midpoint circle — plots 8-way symmetry */
static inline void plot8(int cx, int cy, int dx, int dy)
{
    /* Helper used inside oled_draw_circle; no mutex needed (held by caller). */
    typedef struct { int x; int y; } pt;
    const pt pts[8] = {
        { cx+dx, cy+dy }, { cx-dx, cy+dy },
        { cx+dx, cy-dy }, { cx-dx, cy-dy },
        { cx+dy, cy+dx }, { cx-dy, cy+dx },
        { cx+dy, cy-dx }, { cx-dy, cy-dx },
    };
    for (int i = 0; i < 8; i++) {
        if (pts[i].x >= 0 && pts[i].x < OLED_WIDTH &&
            pts[i].y >= 0 && pts[i].y < OLED_HEIGHT) {
            s_buf[(pts[i].y / 8) * OLED_WIDTH + pts[i].x] |=
                (uint8_t)(1 << (pts[i].y & 7));
        }
    }
}

void oled_draw_circle(uint8_t cx, uint8_t cy, uint8_t r)
{
    int x = 0, y = (int)r, d = 1 - (int)r;
    OLED_LOCK();
    while (x <= y) {
        plot8(cx, cy, x, y);
        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
    OLED_UNLOCK();
}

void oled_fill_circle(uint8_t cx, uint8_t cy, uint8_t r)
{
    for (int dy = -(int)r; dy <= (int)r; dy++) {
        int py = (int)cy + dy;
        if (py < 0 || py >= OLED_HEIGHT) continue;
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        int x0 = (int)cx - dx;
        int x1 = (int)cx + dx;
        if (x0 < 0)           x0 = 0;
        if (x1 >= OLED_WIDTH)  x1 = OLED_WIDTH - 1;

        OLED_LOCK();
        uint8_t bit = (uint8_t)(1 << (py & 7));
        uint32_t row_base = (uint32_t)(py / 8) * OLED_WIDTH;
        for (int col = x0; col <= x1; col++) {
            s_buf[row_base + col] |= bit;
        }
        OLED_UNLOCK();
    }
}

/*  Bitmap format: row-major, MSB-first (standard XBM / BMP style).
 *  Each row is ceil(w/8) bytes wide.
 *  Example: a 16×16 sprite uses 2 * 16 = 32 bytes.
 */
void oled_draw_bitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                      const uint8_t *data)
{
    if (!data || x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    uint8_t row_bytes = (uint8_t)((w + 7) / 8);

    OLED_LOCK();
    for (uint8_t row = 0; row < h; row++) {
        uint8_t py = y + row;
        if (py >= OLED_HEIGHT) break;
        uint8_t page = py / 8;
        uint8_t bit  = (uint8_t)(1 << (py & 7));

        for (uint8_t col = 0; col < w; col++) {
            uint8_t px = x + col;
            if (px >= OLED_WIDTH) break;
            uint8_t src_byte = data[(uint16_t)row * row_bytes + col / 8];
            if (src_byte & (0x80u >> (col & 7))) {
                s_buf[(uint32_t)page * OLED_WIDTH + px] |= bit;
            } else {
                s_buf[(uint32_t)page * OLED_WIDTH + px] &=
                    (uint8_t)~bit;
            }
        }
    }
    OLED_UNLOCK();
}