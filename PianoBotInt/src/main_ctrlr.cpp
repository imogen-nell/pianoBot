#include "coordinator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HardwareSerial.h>

Coordinator::Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask)
    : fingerTaskHandle(fingerTask), stepperTaskHandle(stepperTask)
{

    xTaskCreatePinnedToCore(
        &Coordinator::coordinatorTaskEntry,// expectrs taskFunction_t(void* params), therefore need wrapper (below)
        "coordinatorTask",
        4096,
        this,
        2,
        &coordinatorTaskHandle,
        0
    );
}

TaskHandle_t Coordinator::getTaskHandle() const {
    return coordinatorTaskHandle;
}


//entry wrapper
//static wrapper with correct RTOS signature void(*)(void*)
void Coordinator::coordinatorTaskEntry(void* pvParameters) {
    //convert void* back to ptr to coordinator object for fcn/var access
    static_cast<Coordinator*>(pvParameters)->coordinatorTask();
}
//actual signatue void Coordinator::coordinatorTask()


void Coordinator::coordinatorTask() {
    while(1) {
        // tell stepper to move
        xTaskNotifyGive(stepperTaskHandle);
        // wait until moved
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // tell finger to play note
        // xTaskNotifyGive(fingerTaskHandle);
        // // wait until finger up
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

