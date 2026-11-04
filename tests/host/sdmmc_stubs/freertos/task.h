#pragma once

#include "../../freertos/FreeRTOS.h"

typedef void *TaskHandle_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void (*TaskFunction_t)(void *);

#define pdPASS 1
#define tskIDLE_PRIORITY 0
#define tskNO_AFFINITY (-1)

TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
