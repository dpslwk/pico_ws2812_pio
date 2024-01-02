#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ws2812_pio.h"

// Standard Task priority
#define MAIN_TASK_STACK_SIZE (1024 * 2)
static const char *MAIN_TASK_NAME = "MainThread";
#define MAIN_TASK_PRIORITY ( tskIDLE_PRIORITY + 1UL )


void main_task(void *params)
{
    printf("Main task: start\n");

    printf("Main task: Entering loop\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    vLaunch();

    return 0;
}
