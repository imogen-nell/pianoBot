#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

class Coordinator {
public:
    Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask, int coreID, EventGroupHandle_t playSyncGroup, EventBits_t mySyncBit, EventBits_t allFingersMask);
    TaskHandle_t getTaskHandle() const; //const: calling fcn doesnt modisy this object

private:
    TaskHandle_t coordinatorTaskHandle ;
    TaskHandle_t fingerTaskHandle;
    TaskHandle_t stepperTaskHandle;
    EventGroupHandle_t syncStartEventGroup;
    EventGroupHandle_t playSyncGroup; // Shared between all coordinator instances
    EventBits_t mySyncBit;            // Unique to this instance (e.g., 1 << 0)
    EventBits_t allFingersMask;
    int coreID; //for data logging, indicates which finger is playing

    static void coordinatorTaskEntry(void* pvParameters);
    void coordinatorTask();
};