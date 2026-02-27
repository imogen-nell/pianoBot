#include "coordinator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HardwareSerial.h>

Coordinator::Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask, int flag)
    : fingerTaskHandle(fingerTask), stepperTaskHandle(stepperTask), flag(flag)
{

    xTaskCreatePinnedToCore(
        &Coordinator::coordinatorTaskEntry,// expectrs taskFunction_t(void* params), therefore need wrapper (below)
        "coordinatorTask",
        4096,
        this,
        2, // lower priority than worker tasks to ensure it yeilds to them when waiting for notifications, but high enough to run as soon as workers are done
        &this->coordinatorTaskHandle,
        flag
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
        xTaskNotifyGive(fingerTaskHandle);
        // wait until finger up
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Serial.printf("%d", flag); //for data logger, indicates which finger is playing
        //yeild cpu
        // taskYIELD(); //only yeilds to tasks of equal or higher proprity (the other fingers), may be bttter to use vTaskDelay to allow lower priority tasks (like data logger) to run and print between notes, also prevents starvation of other tasks
        vTaskDelay(pdMS_TO_TICKS(10)); //small delay to prevent starvation of other tasks, also gives time for serial print to go through before next note (for data logger)
        if(flag == 1){
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

