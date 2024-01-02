
#ifndef _PICO_WS2812_PIO_H
#define _PICO_WS2812_PIO_H

#include <stdint.h>

#include "pico/async_context_freertos.h"
#include "pico/util/queue.h"
#include "hardware/pio.h"
#include "hardware/irq.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_WS2812_LANES
#define CONFIG_WS2812_LANES 4
#endif
#ifndef CONFIG_WS2812_PIXELS_PER_LANES
#define CONFIG_WS2812_PIXELS_PER_LANES 128
#endif



#ifdef __cplusplus
}
#endif

#endif
