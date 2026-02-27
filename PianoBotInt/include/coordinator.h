#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Coordinator {
public:
    Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask, int flag);
    TaskHandle_t getTaskHandle() const; //const: calling fcn doesnt modisy this object


private:
    TaskHandle_t coordinatorTaskHandle;
    TaskHandle_t fingerTaskHandle;
    TaskHandle_t stepperTaskHandle;
    int flag; //for data logging, indicates which finger is playing

    static void coordinatorTaskEntry(void* pvParameters);
    void coordinatorTask();
};