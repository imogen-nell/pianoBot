#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// main controller task handle
extern TaskHandle_t coordinatorTaskHandle;

// voice coil and translation (stepper) task handles
extern TaskHandle_t vcTaskHandle;
extern TaskHandle_t t_controllerTaskHandle;

extern int * next_key_ptr;
extern int* next_note_ptr; //pointer to integer
