#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SOLAR_OS_TASK_STOP_WAIT_MS 2000U

typedef enum {
    SOLAR_OS_TASK_ROLE_SYSTEM = 0,
    SOLAR_OS_TASK_ROLE_FOREGROUND,
    SOLAR_OS_TASK_ROLE_BACKGROUND,
} solar_os_task_role_t;

BaseType_t solar_os_task_create_pinned_internal(TaskFunction_t task,
                                                const char *name,
                                                uint32_t stack_depth,
                                                void *parameters,
                                                UBaseType_t priority,
                                                TaskHandle_t *handle,
                                                BaseType_t core_id,
                                                solar_os_task_role_t role);
void solar_os_task_delete_internal(TaskHandle_t task);
bool solar_os_task_wait_done(TaskHandle_t task, volatile bool *task_done, uint32_t timeout_ms);
