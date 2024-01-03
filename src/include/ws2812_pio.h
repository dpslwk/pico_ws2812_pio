
#ifndef _PICO_WS2812_PIO_H
#define _PICO_WS2812_PIO_H

#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/async_context_freertos.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/sem.h"
#include "pico/time.h"

#ifdef __cplusplus
extern "C" {
#endif

#define WS2812_MAX_LANES            (8)
#define WS2812_RESET_TIME_US        (250)  /* RES time, specification says it needs at least 50 us. Need to pause bit stream for this time at the end to latch the values into the LED */

#define WS2812_SPLIT_RGB(u32)       (((u32) >> 16) & 0xff), (((u32) >> 8) & 0xff), ((u32) & 0xff)
#define WS2812_COMBINE_RGB(r,g,b)   (((uint32_t) (r) << 16) | ((uint32_t) (g) << 8) | (uint32_t) (b))

typedef struct {
    PIO pio;
    int pio_sm;
    int base_pin;
    uint8_t lanes;
    uint16_t pixels;

    unsigned int pio_offset;
    int dma;
    semaphore_t reset_delay_complete_sem;  /* semaphore used to make a delay at the end of the transfer. Posted when it is safe to output a new set of values */
    alarm_id_t reset_delay_alarm_id;            /*  alarm id handle for handling delay */

    size_t buffer_length;
    uint32_t *draw_buffer;
    uint32_t *transmit_buffer;
    uint32_t *buffer;
} ws2812_pio_t;

extern ws2812_pio_t ws2812_state;

/**
 * Initialise the WS2812 driver.
 *
 * @param ws2812_pio_t *self    the driver state object. This should always be \c &ws2812_state
 * @param PIO pio               PIO to use
 * @param int sm                sm to use
 * @param int base_pin          GPIO pin of the first lane
 * @param uint16_t pixels       Number of pixels per lane
 * @param uint8_t lanes         Number of lanes/strips max 8 (WS2812_MAX_LANES)
 *
 * @return bool
 */
bool ws2812_pio_init(
    ws2812_pio_t *self,
    PIO pio,
    int sm,
    int base_pin,
    uint16_t pixels,
    uint8_t lanes
);

/**
 * De-initialise the given WS2812.
 *
 * @param ws2812_pio_t *self    WS2812 instance itself
 */
bool ws2812_pio_deinit(ws2812_pio_t *self);

bool ws2812_dma_transfer_is_ongoing(ws2812_pio_t *self);
void ws2812_wait_for_buffer_ready(ws2812_pio_t *self); // TODO:
bool ws2812_transfer(ws2812_pio_t *self);
bool ws2812_show(ws2812_pio_t *self);
bool ws2812_show(ws2812_pio_t *self, bool trasnfer);

/**
 * Use DMA to copy a buffer.
 *
 * @param self      WS2812 instance itself
 * @param dst       destination buffer
 * @param src       source buffer
 * @param length    length of the buffer in uint32_t
 */
void _ws2812_dma_memcpy_buffer(ws2812_pio_t *self, uint32_t *dst, uint32_t *src, uint32_t length);

bool ws2812_clear_pixel(ws2812_pio_t *self, uint8_t lane, uint16_t pixel);
bool ws2812_clear_all_pixel(ws2812_pio_t *self);
bool ws2812_set_all_pixel_color(ws2812_pio_t *self, uint32_t color);
bool ws2812_set_pixel_color(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint32_t color);
bool ws2812_get_pixel_color(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint32_t *color);

bool ws2812_set_pixel_rgb(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue);
bool ws2812_get_pixel_rgb(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t *redP, uint8_t *greenP, uint8_t *blueP);

#ifdef __cplusplus
}
#endif

#endif
