#include "app_freertos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "health.h"
#include <string.h>

void MX_FREERTOS_Init(void)
{
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    char msg[48] = "stack overflow: ";
    if (pcTaskName) {
        strncat(msg, pcTaskName, sizeof(msg) - strlen(msg) - 1);
    }
    health_reset(msg);
}

void vApplicationMallocFailedHook(void)
{
    health_reset("FreeRTOS pvPortMalloc failed (OOM)");
}
