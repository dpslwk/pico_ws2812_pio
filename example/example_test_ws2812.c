#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ws2812_pio.h"

// Standard Task priority
#define MAIN_TASK_STACK_SIZE (1024 * 2)
static const char *MAIN_TASK_NAME = "MainThread";
#define MAIN_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )

ws2812_pio_t ws2812_pio;

void main_task(void *params)
{
    printf("Main task: start\n");

    gpio_init(10);
    gpio_set_dir(10, GPIO_OUT);
    bool ret;
    ret = ws2812_pio_init(
        &ws2812_state,
        pio0,
        pio_claim_unused_sm(pio0, true),
        11, // GPIO pin
        25, // number of pixels
        1   // number of lanes
    );
    if (! ret) {
        printf("ws2812 init failed\n");
    }

    ret = ws2812_set_all_pixel_color(&ws2812_state, 0x00001100);
    if (! ret) {
        printf("failed to set all colours\n");
    }

    // ws2812_set_pixel_color(ws2812_pio_t *self, uint8_t lane, uint16_t pixel, uint32_t color)
    ws2812_set_pixel_color(&ws2812_state, 0, 3, 0x00110000);
    if (! ret) {
        printf("failed to set 3\n");
    }
    ws2812_set_pixel_color(&ws2812_state, 0, 8, 0x00000011);
    if (! ret) {
        printf("failed to set 8\n");
    }

    ws2812_show(&ws2812_state);
    printf("Main task: Entering loop\n");
    while (true) {
        gpio_put(10, ! gpio_get(10));
        ws2812_transfer(&ws2812_state);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vLaunch()
{
    TaskHandle_t task;

    xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &task);

    // more tasks, they can set vTaskCoreAffinitySet if needed

    // Start the tasks and timer running.
    vTaskStartScheduler();
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("GO\n");


    // gpio_init(10);
    // gpio_set_dir(10, GPIO_OUT);
    // bool ret;
    // ret = ws2812_pio_init(
    //     &ws2812_state,
    //     pio0,
    //     pio_claim_unused_sm(pio0, true),
    //     11, // GPIO pin
    //     25, // number of pixels
    //     1   // number of lanes
    // );
    // if (! ret) {
    //     printf("ws2812 init failed\n");
    // }

    // ret = ws2812_set_all_pixel_color(&ws2812_state, 0x00110000);
    // if (! ret) {
    //     printf("failed to set all colours\n");
    // }

    // ws2812_transfer(&ws2812_state);

    // printf("Entering loop\n");
    // while (true) {
    //     gpio_put(10, ! gpio_get(10));
    //     ws2812_transfer(&ws2812_state);
    //     // vTaskDelay(pdMS_TO_TICKS(1000));
    //     sleep_ms(1000);
    // }


    vLaunch();

    return 0;
}
