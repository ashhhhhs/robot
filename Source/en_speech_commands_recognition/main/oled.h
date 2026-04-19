#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ─────────────────────────────────────────────
//  Display geometry
// ─────────────────────────────────────────────
#define OLED_WIDTH   128
#define OLED_HEIGHT   64
#define OLED_PAGES    (OLED_HEIGHT / 8)   // 8

// ─────────────────────────────────────────────
//  Lifecycle
// ─────────────────────────────────────────────

/** Initialise I2C bus and SSD1306 hardware. Must be called once before anything else. */
void oled_init(void);

/** Zero the frame buffer (does NOT push to display — call oled_flush()). */
void oled_clear(void);

/** Push the entire frame buffer to the display over I2C. */
void oled_flush(void);

/** Clear the buffer and immediately flush (convenience wrapper). */
void oled_clear_and_flush(void);

// ─────────────────────────────────────────────
//  Display control
// ─────────────────────────────────────────────

/** Set display contrast (0–255, default ~127). */
void oled_set_contrast(uint8_t contrast);

/** Invert all pixels on the display (hardware command, no buffer change). */
void oled_set_invert(bool invert);

/** Turn the display panel on or off (does not affect the frame buffer). */
void oled_set_power(bool on);

// ─────────────────────────────────────────────
//  Text
// ─────────────────────────────────────────────

/**
 * Draw a null-terminated string into the buffer at pixel-column x, page row page.
 * Characters outside the display boundary are clipped.
 * Call oled_flush() afterwards to make it visible.
 */
void oled_draw_text_at(uint8_t x, uint8_t page, const char *text);

/**
 * Return the pixel width a string would occupy (does not draw anything).
 * Useful for computing centred positions: x = (OLED_WIDTH - oled_text_width(s)) / 2
 */
uint8_t oled_text_width(const char *text);

/**
 * Classic two-line helper: clears, draws line1 on the upper half and
 * line2 on the lower half, centred horizontally, then flushes.
 */
void oled_display_text(const char *line1, const char *line2);

/**
 * Same as oled_display_text() but centres each line horizontally.
 * Both strings are drawn in the vertical centre of their respective halves.
 */
void oled_display_text_centered(const char *line1, const char *line2);

/**
 * Draw up to `count` strings (max 8, one per page row), clear first, then flush.
 * Useful for status screens or debug dumps.
 */
void oled_display_lines(const char * const *lines, uint8_t count);

// ─────────────────────────────────────────────
//  Pixel & primitives  (buffer only — call oled_flush() to show)
// ─────────────────────────────────────────────

/** Set or clear a single pixel at (x, y). y=0 is the top of the display. */
void oled_draw_pixel(uint8_t x, uint8_t y, bool set);

/** Draw a horizontal line from (x, y) with length w. */
void oled_draw_hline(uint8_t x, uint8_t y, uint8_t w);

/** Draw a vertical line from (x, y) with height h. */
void oled_draw_vline(uint8_t x, uint8_t y, uint8_t h);

/** Draw a Bresenham line between two arbitrary points. */
void oled_draw_line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

/** Draw an unfilled rectangle. */
void oled_draw_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/** Draw a filled rectangle. */
void oled_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/** Draw an unfilled circle using the midpoint algorithm. */
void oled_draw_circle(uint8_t cx, uint8_t cy, uint8_t r);

/** Draw a filled circle. */
void oled_fill_circle(uint8_t cx, uint8_t cy, uint8_t r);

/**
 * Draw a 1-bit-per-pixel bitmap into the buffer.
 * `data` must be a row-major byte array, MSB-first, of size ceil(w/8)*h bytes.
 * Useful for pre-rendered dog-face sprites.
 */
void oled_draw_bitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                      const uint8_t *data);