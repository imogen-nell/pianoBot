#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// main controller task handle
extern TaskHandle_t coordinatorTaskHandle;
extern TaskHandle_t vcTaskHandle;
extern TaskHandle_t t_controllerTaskHandle;

extern int * key_positions;
extern int* notes; //pointer to integer
