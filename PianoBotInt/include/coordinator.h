#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void coordinator_init(TaskHandle_t fingerTask, TaskHandle_t stepperTask);
