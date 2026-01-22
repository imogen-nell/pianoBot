#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Declare the mutex for all controllers/tasks
extern SemaphoreHandle_t ctrl_Mutex;

extern int * key_positions;
extern int* notes; //pointer to integer
