#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Coordinator {
public:
    Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask);
    TaskHandle_t getTaskHandle();

private:
    TaskHandle_t coordinatorTaskHandle;
    TaskHandle_t fingerTaskHandle;
    TaskHandle_t stepperTaskHandle;

    static void coordinatorTaskEntry(void* pvParameters);
    void coordinatorTask();
};