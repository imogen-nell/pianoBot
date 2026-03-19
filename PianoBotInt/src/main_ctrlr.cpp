#include "coordinator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HardwareSerial.h>
#include "freertos/event_groups.h"


Coordinator::Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask, int coreID, EventGroupHandle_t playSyncGroup, EventBits_t mySyncBit, EventBits_t allFingersMask)
    : fingerTaskHandle(fingerTask), stepperTaskHandle(stepperTask), playSyncGroup(playSyncGroup), mySyncBit(mySyncBit), allFingersMask(allFingersMask)
{

    xTaskCreatePinnedToCore(
        &Coordinator::coordinatorTaskEntry,// expectrs taskFunction_t(void* params), therefore need wrapper (below)
        "coordinatorTask",
        4096,
        this,
        2, // lower priority than worker tasks to ensure it yeilds to them when waiting for notifications, but high enough to run as soon as workers are done
        &this->coordinatorTaskHandle,
        coreID
    );
}

TaskHandle_t Coordinator::getTaskHandle() const {
    return coordinatorTaskHandle;
}


//entry wrapper
//static wrapper with correct RTOS signature void(*)(void*)
void Coordinator::coordinatorTaskEntry( void* pvParameters) {
    //convert void* back to ptr to coordinator object for fcn/var access
    static_cast<Coordinator*>(pvParameters)->coordinatorTask();
}



void Coordinator::coordinatorTask() {

    if (playSyncGroup != NULL) {
        xEventGroupSync(
            playSyncGroup,
            mySyncBit,        // Use the instance bit passed in constructor
            allFingersMask,   // Use the mask passed in constructor
            portMAX_DELAY
        );
    }

    while(1) {
        // tell stepper to move
        xTaskNotifyGive(stepperTaskHandle);
        // wait until moved
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (playSyncGroup != NULL) {
            xEventGroupSync(
                playSyncGroup, 
                mySyncBit,       // "I am ready"
                allFingersMask,  // "Wait for everyone else"
                portMAX_DELAY
            );
        }
        // tell finger to play note
        xTaskNotifyGive(fingerTaskHandle);
        // wait until finger up
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //yeild cpu
        vTaskDelay(pdMS_TO_TICKS(1)); //small delay to prevent starvation of other tasks, also gives time for serial print to go through before next note (for data logger)
    }
}

