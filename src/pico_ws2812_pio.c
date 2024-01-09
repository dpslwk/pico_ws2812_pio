
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifdef LIB_FREERTOS_KERNEL
#include "FreeRTOS.h"
#include "task.h"
#define WS2821_TRANSFER_TASK_STACK_SIZE (512)
#define WS2812_CALLOC(xNum, xSize) pvPortCalloc(xNum, xSize)
#else
#define WS2812_CALLOC(num, size) calloc(num, size)
#endif

#include "pico/stdlib.h"
// #include "pico/async_context_freertos.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "pico/sem.h"
#include "pico/time.h"

#include "ws2812.pio.h"

#include "ws2812_pio.h"

ws2812_pio_t ws2812_state;

static size_t _ws2812_buffer_per_pixel[WS2812_MAX_LANES + 1] = {
    1, 1,       // 0>1 lanes
    3, 3, 3,    // 2>4 lanes
    6, 6, 6, 6, // 5>8 lanes
};


void hexdump(const void *mem, size_t len, uint8_t cols) {
    // print_lock.acquire();
    const uint8_t *src = (const uint8_t*) mem;
    printf("\n[HEXDUMP] Address: 0x%08X len: 0x%zX (%zu)", (ptrdiff_t)src, len, len);
    for (uint32_t i = 0; i < len; ++i) {
        if (i % cols == 0) {
            printf("\n[0x%08X] 0x%08lX: ", (ptrdiff_t)src, i);
        }
        printf("%02X ", *src);
        src++;
    }
    printf("\n");
    // print_lock.release();
}

static int64_t _ws2812_reset_delay_complete(alarm_id_t id, void *user_data) {
    ws2812_pio_t *self = user_data;
    self->reset_delay_alarm_id = 0; /* reset alarm id */
    sem_release(&self->reset_delay_complete_sem); /* release semaphore */
    return 0; /* no repeat */
}

void _ws2812_dma_complete_handler(void) {
    if (dma_channel_get_irq0_status(ws2812_state.dma)) {
        dma_channel_acknowledge_irq0(ws2812_state.dma);

        if (ws2812_state.reset_delay_alarm_id != 0) { /* safety check: is there somehow an alarm already running? */
            cancel_alarm(ws2812_state.reset_delay_alarm_id); /* cancel it */
        }

        /* setup alarm to wait for the required latch-in time for the LES at the end of the transfer */
        ws2812_state.reset_delay_alarm_id = add_alarm_in_us(WS2812_RESET_TIME_US, _ws2812_reset_delay_complete, (void *) &ws2812_state, true);
    }
}

bool _ws2812_dma_init(ws2812_pio_t *self) {
    self->dma = dma_claim_unused_channel(true);

    dma_channel_config channel_config = dma_channel_get_default_config(self->dma); /* get default configuration */
    channel_config_set_dreq(&channel_config, pio_get_dreq(self->pio, self->pio_sm, true)); /* configure data request. true: sending data to the PIO state machine */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32); /* data transfer size is 32 bits */
    channel_config_set_read_increment(&channel_config, true); /* each read of the data will increase the read pointer */

    dma_channel_configure(self->dma,
        &channel_config,
        &self->pio->txf[self->pio_sm], /* write address: write to PIO FIFO */
        NULL, /* don't provide a read address yet */
        self->buffer_length, /* number of transfers */
        false); /* don't start yet */

    irq_add_shared_handler(DMA_IRQ_0, &_ws2812_dma_complete_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY); /* after DMA all data, raise an interrupt */
    dma_channel_set_irq0_enabled(self->dma, true); /* map DMA channel to interrupt */
    irq_set_enabled(DMA_IRQ_0, true); /* enable interrupt */

    return true;
}

bool ws2812_dma_transfer_is_ongoing(ws2812_pio_t *self) {
    return ! sem_acquire_timeout_ms(&self->reset_delay_complete_sem, 0); /* returns false for timeout (lock still hold by DMA) */
}

void ws2812_wait_for_buffer_ready(ws2812_pio_t *self) {
  /* in case of DMA transfer still going on, need to wait, otherwise we overwrite the data buffer. Another approach would be swapping buffers */
 // while(WS2812_DMATransferIsOngoing()) {
 //   vTaskDelay(pdMS_TO_TICKS(1));
 // }
}

void _ws2812_dma_memcpy_buffer(ws2812_pio_t *self, uint32_t *dst, uint32_t *src, uint32_t length) {
    int chan = dma_claim_unused_channel(false);
    if (chan == false) {
        memcpy(dst, src, length * sizeof(uint32_t));

        return;
    }

    dma_channel_config c = dma_channel_get_default_config(chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);

    dma_channel_configure(
        chan,       // Channel to be configured
        &c,         // The configuration we just created
        dst,        // The initial write address
        src,        // The initial read address
        length,     // Number of transfers;
        false       // Don't start immediately.
    );

    sem_acquire_blocking(&self->reset_delay_complete_sem); /* get semaphore */
    dma_channel_start(chan);
    dma_channel_wait_for_finish_blocking(chan);
    sem_release(&self->reset_delay_complete_sem); /* release semaphore */

    dma_channel_unclaim(chan);
}

bool ws2812_transfer(ws2812_pio_t *self) {
    sem_acquire_blocking(&self->reset_delay_complete_sem); /* get semaphore */
    dma_channel_set_read_addr(self->dma, (void *) self->transmit_buffer, true); /* trigger DMA transfer */

    return true;
}

bool ws2812_show(ws2812_pio_t *self) {
    _ws2812_dma_memcpy_buffer(self, self->transmit_buffer, self->draw_buffer, self->buffer_length);

    return true;
}

bool ws2812_pio_init(
    ws2812_pio_t *self,
    PIO pio,
    int sm,
    int base_pin,
    uint16_t pixels,
    uint8_t lanes
) {
    if (lanes > WS2812_MAX_LANES) {
        panic("WS2812: To many lanes requested");

        return false;
    }

    self->pio = pio;
    self->pio_sm = sm;
    self->base_pin = base_pin;
    self->lanes = lanes;
    self->pixels = pixels;

    self->buffer_length = (size_t) _ws2812_buffer_per_pixel[self->lanes] * self->pixels;
    self->draw_buffer = WS2812_CALLOC(
        self->buffer_length,
        sizeof(uint32_t)
    );

    if (self->draw_buffer == NULL) {
        panic("WS2812: Failed to calloc draw_buffer");

        return false;
    }

    self->transmit_buffer = WS2812_CALLOC(
        self->buffer_length,
        sizeof(uint32_t)
    );

    if (self->transmit_buffer == NULL) {
        panic("WS2812: Failed to calloc transmit_buffer");

        return false;
    }

    if (self->lanes > 4) {
        printf("WS2812: Using 8 lane pio\n");
        self->pio_offset = pio_add_program(self->pio, &ws2812_parallel_8_program);
        ws2812_parallel_8_program_init(self->pio,  self->pio_sm , self->pio_offset, self->base_pin, self->lanes);
    } else if (self->lanes > 1) {
        printf("WS2812: Using 4 lane pio\n");
        self->pio_offset = pio_add_program(self->pio, &ws2812_parallel_4_program);
        ws2812_parallel_4_program_init(self->pio,  self->pio_sm , self->pio_offset, self->base_pin, self->lanes);
    } else {
        printf("WS2812: Using 1 lane pio\n");
        self->pio_offset = pio_add_program(self->pio, &ws2812_program);
        ws2812_program_init(self->pio,  self->pio_sm , self->pio_offset, self->base_pin);
    }

    sem_init(&self->reset_delay_complete_sem, 1, 1); /* semaphore initially posted so we don't block first time */

    if (! _ws2812_dma_init(self)) {
        printf("WS2812: dma init failed\n");

        return false;
    }

    return true;
}

bool ws2812_pio_deinit(ws2812_pio_t *self) {

    return true;
}

bool ws2812_clear_pixel(ws2812_pio_t *self, uint8_t lane, uint16_t pixel) {
    return ws2812_set_pixel_rgb(self, lane, pixel, 0, 0, 0);
}

bool ws2812_clear_all_pixel(ws2812_pio_t *self) {
    memset(self->draw_buffer, 0, self->buffer_length * sizeof(uint32_t));

    return true;
}

bool ws2812_clear_all_pixel_in_lane(ws2812_pio_t *self, uint8_t lane) {
    uint16_t pixel;
    bool res;

    for (pixel = 0; pixel < self->pixels; ++pixel) {
        res = ws2812_clear_pixel(self, lane, pixel);
        if (res != true) {
            return res;
        }
    }
}

bool ws2812_set_all_pixel_color(ws2812_pio_t *self, uint32_t color) {
    uint8_t lane;
    uint16_t pixel;
    bool res;

    for (pixel = 0; pixel < self->pixels; ++pixel) {
        for (lane = 0; lane < self->lanes; ++lane) {
            res = ws2812_set_pixel_color(self, lane, pixel, color);
            if (res != true) {
                return res;
            }
        }
    }

    return true;
}

bool ws2812_set_pixel_color(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint32_t color) {
    return ws2812_set_pixel_rgb(self, lane, pixel, WS2812_SPLIT_RGB(color));
}

bool ws2812_get_pixel_color(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint32_t *color) {
    uint8_t res;
    uint8_t r;
    uint8_t g;
    uint8_t b;

    res = ws2812_get_pixel_rgb(self, lane, pixel, &r, &g, &b);
    *color = WS2812_COMBINE_RGB(r, g, b);

    return res;
}

bool ws2812_set_pixel_rgb(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue) {
    if (lane >= self->lanes || pixel >= self->pixels) {
        printf("WS2812: set_pixel_rgb: Out of range\n");

        return false; /* error, out of range */
    }
    int idx;
    int i;
    uint8_t *p;

    idx = pixel * _ws2812_buffer_per_pixel[self->lanes];
    p = (uint8_t*) &self->draw_buffer[idx];

    if (self->lanes > 4) {
        /*
         * 8 lanes GRB
         *                        l7l6l5l4 l3l2l1l0
         * buffer[0]: 4x 8bit:    g7g7g7g7 g7g7g7g7 | g6g6g6g6 g6g6g6g6 | g5g5g5g5 g5g5g5g5 | g4g4g4g4 g4g4g4g4
         * buffer[1]: 4x 8bit:    g3g3g3g3 g3g3g3g3 | g2g2g2g2 g2g2g2g2 | g1g1g1g1 g1g1g1g1 | g0g0g0g0 g0g0g0g0
         * buffer[2]: 4x 8bit:    r7r7r7r7 r7r7r7r7 | r6r6r6r6 r6r6r6r6 | r6r6r6r6 r6r6r6r6 | r6r6r6r6 r6r6r6r6
         * buffer[3]: 4x 8bit:    r3r3r3r3 r3r3r3r3 | r2r2r2r2 r2r2r2r2 | r1r1r1r1 r1r1r1r1 | r0r0r0r0 r0r0r0r0
         * buffer[4]: 4x 8bit:    b7b7b7b7 b7b7b7b7 | b6b6b6b6 b6b6b6b6 | b6b6b6b6 b6b6b6b6 | b6b6b6b6 b6b6b6b6
         * buffer[5]: 4x 8bit:    b3b3b3b3 b3b3b3b3 | b2b2b2b2 b2b2b2b2 | b1b1b1b1 b1b1b1b1 | b0b0b0b0 b0b0b0b0
         */
        for (i = 0; i < 8; ++i) {
            if (green & 0x80) { // pick top bit
                *p |= (1 << lane);
            } else {
                *p &= ~(1 << lane);
            }
            green <<= 1;

            if (red & 0x80) {
                *(p + 8) |= (1 << lane);
            } else {
                *(p + 8) &= ~(1 << lane);
            }
            red <<= 1;

            if (blue & 0x80) {
                *(p + 16) |= (1 << lane);
            } else {
                *(p + 16) &= ~(1 << lane);
            }
            blue <<= 1;

            p++;
        }
    } else if (self->lanes > 1) {
        /*
         * 4 lanes GRB
         *                        l3l2l1l0 l3l2l1l0
         * buffer[0]: 4x 8bit:    g7g7g7g7 g6g6g6g6 | g5g5g5g5 g4g4g4g4 | g3g3g3g3 g2g2g2g2 | g1g1g1g1 g0g0g0g0
         * buffer[1]: 4x 8bit:    r7r7r7r7 r6r6r6r6 | r5r5r5r5 r4r4r4r4 | r3r3r3r3 r2r2r2r2 | r1r1r1r1 r0r0r0r0
         * buffer[2]: 4x 8bit:    b7b7b7b7 b6b6b6b6 | b5b5b5b5 b4b4b4b4 | b3b3b3b3 b2b2b2b2 | b1b1b1b1 b0b0b0b0
         */
        bool even_bit;
        for (i = 0; i < 8; ++i) {
            even_bit = (i % 2) == 1;
            if (green & 0x80) {
                *p |= (1 << (lane + (even_bit ? 0: 4)));
            } else {
                *p &= ~(1 << (lane + (even_bit ? 0: 4)));
            }
            green <<= 1;

            if (red & 0x80) {
                *(p + 4) |= (1 << (lane + (even_bit ? 0: 4)));
            } else {
                *(p + 4) &= ~(1 << (lane + (even_bit ? 0: 4)));
            }
            red <<= 1;

            if (blue & 0x80) {
                *(p + 8) |= (1 << (lane + (even_bit ? 0: 4)));
            } else {
                *(p + 8) &= ~(1 << (lane + (even_bit ? 0: 4)));
            }
            blue <<= 1;

            if (even_bit) {
                p++;
            }
        }
    } else {
        /*
         * 1 lanes GRB
         * buffer[0]: 4x 8bit:    g7g6g5g4 g3g2g1g0 | r7r6r5r4 r3r2r1r0 | b7b6b5b4 b3b2b1b0 | 0000 0000
         */
        self->draw_buffer[idx] = ((uint32_t) (green) << 24) | ((uint32_t) (red) << 16) | ((uint32_t) (blue) << 8);
    }

    return true;
}

bool ws2812_get_pixel_rgb(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t *redP, uint8_t *greenP, uint8_t *blueP) {
    if (lane >= self->lanes || pixel >= self->pixels) {
        return false; /* error, out of range */
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    int idx;
    int i;
    uint8_t *p;

    idx = pixel * _ws2812_buffer_per_pixel[self->lanes];
    p = (uint8_t*) &self->draw_buffer[idx];

    if (self->lanes > 4) {
        /*
         * 8 lanes GRB
         *                        l7l6l5l4 l3l2l1l0
         * buffer[0]: 4x 8bit:    g7g7g7g7 g7g7g7g7 | g6g6g6g6 g6g6g6g6 | g5g5g5g5 g5g5g5g5 | g4g4g4g4 g4g4g4g4
         * buffer[1]: 4x 8bit:    g3g3g3g3 g3g3g3g3 | g2g2g2g2 g2g2g2g2 | g1g1g1g1 g1g1g1g1 | g0g0g0g0 g0g0g0g0
         * buffer[2]: 4x 8bit:    r7r7r7r7 r7r7r7r7 | r6r6r6r6 r6r6r6r6 | r6r6r6r6 r6r6r6r6 | r6r6r6r6 r6r6r6r6
         * buffer[3]: 4x 8bit:    r3r3r3r3 r3r3r3r3 | r2r2r2r2 r2r2r2r2 | r1r1r1r1 r1r1r1r1 | r0r0r0r0 r0r0r0r0
         * buffer[4]: 4x 8bit:    b7b7b7b7 b7b7b7b7 | b6b6b6b6 b6b6b6b6 | b6b6b6b6 b6b6b6b6 | b6b6b6b6 b6b6b6b6
         * buffer[5]: 4x 8bit:    b3b3b3b3 b3b3b3b3 | b2b2b2b2 b2b2b2b2 | b1b1b1b1 b1b1b1b1 | b0b0b0b0 b0b0b0b0
         */
        for (i = 0; i < 8; ++i) {
            green <<= 1;
            if ((*p) & (1 << lane)) {
                green |= 1;
            }

            red <<= 1;
            if ((*(p + 8)) & (1 << lane)) {
                red |= 1;
            }

            blue <<= 1;
            if ((*(p + 16)) & (1 << lane)) {
                blue |= 1;
            }

            p++;
        }
    } else if (self->lanes > 1) {
        /*
         * 4 lanes GRB
         *                        l3l2l1l0 l3l2l1l0
         * buffer[0]: 4x 8bit:    g7g7g7g7 g6g6g6g6 | g5g5g5g5 g4g4g4g4 | g3g3g3g3 g2g2g2g2 | g1g1g1g1 g0g0g0g0
         * buffer[1]: 4x 8bit:    r7r7r7r7 r6r6r6r6 | r5r5r5r5 r4r4r4r4 | r3r3r3r3 r2r2r2r2 | r1r1r1r1 r0r0r0r0
         * buffer[2]: 4x 8bit:    b7b7b7b7 b6b6b6b6 | b5b5b5b5 b4b4b4b4 | b3b3b3b3 b2b2b2b2 | b1b1b1b1 b0b0b0b0
         */
        bool even_bit;
        for (i = 0; i < 8; ++i) {
            even_bit = (i % 2) == 1;

            green <<= 1;
            if ((*p) & (1 << (lane + (even_bit ? 0: 4)))) {
                green |= 1;
            }

            red <<= 1;
            if ((*(p + 4)) & (1 << (lane + (even_bit ? 0: 4)))) {
                red |= 1;
            }

            blue <<= 1;
            if ((*(p + 8)) & (1 << (lane + (even_bit ? 0: 4)))) {
                blue |= 1;
            }

            if (even_bit) {
                p++;
            }
        }
    } else {
        /*
         * 1 lanes GRB
         * buffer[0]: 4x 8bit:    g7g6g5g4 g3g2g1g0 | r7r6r5r4 r3r2r1r0 | b7b6b5b4 b3b2b1b0 | 0000 0000
         */
        blue = (self->draw_buffer[idx] >> 8) & 0xff;
        red = (self->draw_buffer[idx] >> 16) & 0xff;
        green = (self->draw_buffer[idx] >> 24) & 0xff;
    }

    *redP = red;
    *greenP = green;
    *blueP = blue;

    return true;
}

uint32_t ws2812_brightness_percent_color(uint32_t color, uint8_t percent) {
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    red = (color >> 16) & 0xff;
    green = (color >> 8) & 0xff;
    blue = color & 0xff;

    red = ((uint32_t) red * percent) / 100;
    green = ((uint32_t) green * percent) / 100;
    blue = ((uint32_t) blue * percent) / 100;

    color = (red << 16) | (green << 8) | blue;

    return color;
}

uint32_t ws2812_brightness_factor_color(uint32_t color, uint8_t factor) {
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    red = (color >> 16) & 0xff;
    green = (color >> 8) & 0xff;
    blue = color & 0xff;

    red = ((uint32_t) red * factor) / 255;
    green = ((uint32_t) green * factor) / 255;
    blue = ((uint32_t) blue * factor) / 255;

    color = (red << 16) | (green << 8) | blue;

    return color;
}

bool ws2812_dimm_pixel_by_percent(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t percent) {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint32_t dRed;
    uint32_t dGreen;
    uint32_t dBlue;
    uint8_t res;

    res = ws2812_get_pixel_rgb(self, lane, pixel, &red, &green, &blue);
    if (res != true) {
        return res;
    }

    dRed = ((uint32_t) red * (100 - percent)) / 100;
    dGreen = ((uint32_t) green * (100 - percent)) / 100;
    dBlue = ((uint32_t) blue * (100 - percent)) / 100;

    return ws2812_set_pixel_rgb(self, lane, pixel, (uint8_t) dRed, (uint8_t) dGreen, (uint8_t) dBlue);
}

bool ws2812_dimm_all_pixels_in_lane_by_percent(ws2812_pio_t *self, uint8_t lane, uint8_t percent) {
    uint16_t pixel;
    bool res;

    for (pixel = 0; pixel < self->pixels; ++pixel) {
        res = ws2812_dimm_pixel_by_percent(self, lane, pixel, percent);
        if (res != true) {
            return res;
        }
    }

    return res;
}


bool ws2812_dimm_pixel_by_factor(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint8_t factor) {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint32_t dRed;
    uint32_t dGreen;
    uint32_t dBlue;
    uint8_t res;

    res = ws2812_get_pixel_rgb(self, lane, pixel, &red, &green, &blue);
    if (res != true) {
        return res;
    }

    dRed = ((uint32_t) red * (255 - factor)) / 255;
    dGreen = ((uint32_t) green * (255 - factor)) / 255;
    dBlue = ((uint32_t) blue * (255 - factor)) / 255;

    return ws2812_set_pixel_rgb(self, lane, pixel, (uint8_t) dRed, (uint8_t) dGreen, (uint8_t) dBlue);
}

bool ws2812_dimm_all_pixels_in_lane_by_factor(ws2812_pio_t *self, uint8_t lane, uint8_t factor) {
    uint16_t pixel;
    bool res;

    for (pixel = 0; pixel < self->pixels; ++pixel) {
        res = ws2812_dimm_pixel_by_factor(self, lane, pixel, factor);
        if (res != true) {
            return res;
        }
    }

    return res;
}


#ifdef LIB_FREERTOS_KERNEL

static void _ws2812_transfer_task(void *params) {
    ws2812_pio_t *self = params;
    TickType_t xLastWakeTime;
    TickType_t xLastWatchdog;
    const TickType_t xFrequency = pdMS_TO_TICKS(self->refresh_rate);

    xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        if (xLastWakeTime - xLastWatchdog > pdMS_TO_TICKS(1000)) {
            xLastWatchdog = xLastWakeTime;
        }
        ws2812_transfer(self);
    }
}

bool ws2812_pio_init_freertos(
    ws2812_pio_t *self,
    PIO pio,
    int sm,
    int base_pin,
    uint16_t pixels,
    uint8_t lanes,
    uint16_t refresh_rate,
    uint32_t task_priority
) {
    bool ret;
    ret = ws2812_pio_init(self, pio, sm, base_pin, pixels, lanes);
    if (ret != true) {
        return false;
    }

    self->refresh_rate = refresh_rate;

    xTaskCreate(_ws2812_transfer_task, "WS2812TransferTask", WS2821_TRANSFER_TASK_STACK_SIZE, self, task_priority, &self->_trasnfer_task_handle);
}

#endif // LIB_FREERTOS_KERNEL
