#include "coordinator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <HardwareSerial.h>
#include "freertos/event_groups.h"

Coordinator::Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask, int flag, EventGroupHandle_t eventGroup)
    : fingerTaskHandle(fingerTask), stepperTaskHandle(stepperTask), flag(flag), syncStartEventGroup(eventGroup)
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
void Coordinator::coordinatorTaskEntry( void* pvParameters) {
    //convert void* back to ptr to coordinator object for fcn/var access
    static_cast<Coordinator*>(pvParameters)->coordinatorTask();
}
//actual signatue void Coordinator::coordinatorTask()


void Coordinator::coordinatorTask() {
    //determine bits 
    const EventBits_t finger1Bit = (1<<0);
    const EventBits_t finger2Bit = (1<<1);
    const EventBits_t allReadyBits = finger1Bit | finger2Bit;
    EventBits_t thisBit = (flag == 0) ? finger1Bit : finger2Bit;

    //BARRIER
    //wait until both fingers are ready (have hit their barrier/ moved to first position and are ready to play)
    if (syncStartEventGroup != NULL) {
        xEventGroupSync(
            syncStartEventGroup,
            thisBit,           // Set my bit
            allReadyBits,    // Wait for both
            portMAX_DELAY    // Wait forever until the other motor catches up
        );
    }
    while(1) {
        // tell stepper to move
        xTaskNotifyGive(stepperTaskHandle);
        // wait until moved
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // tell finger to play note
        xTaskNotifyGive(fingerTaskHandle);
        // wait until finger up
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Serial.printf("%d", flag); //for data logger, indicates which finger is playing
        //yeild cpu
        // taskYIELD(); //only yeilds to tasks of equal or higher proprity (the other fingers), may be bttter to use vTaskDelay to allow lower priority tasks (like data logger) to run and print between notes, also prevents starvation of other tasks
        vTaskDelay(pdMS_TO_TICKS(1)); //small delay to prevent starvation of other tasks, also gives time for serial print to go through before next note (for data logger)
    }
}

