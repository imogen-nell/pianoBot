#include "vc_controller.h"
// #include "position_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "global.h"


class Coordinator{
public: 
    Coordinator(TaskHandle_t fingerTask, TaskHandle_t stepperTask)
        : fingerTaskHandle(fingerTask), stepperTaskHandle(stepperTask)
        {
    // fingerTaskHandle = fingerTask;
    // stepperTaskHandle = stepperTask;
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
    
    TaskHandle_t getTaskHandle() const {
        return coordinatorTaskHandle;
    }


private: 

    //this: hidden, implicit pointer automatically passed 
    //      to every non-static member function of a class

    //task config 
    TaskHandle_t coordinatorTaskHandle ;
    TaskHandle_t fingerTaskHandle ; //no static: lifetime and visibility automatically tied to the instance
    TaskHandle_t stepperTaskHandle ; 

    //entry wrapper
    //static fcn with correct RTOS signature void(*)(void*)
    static void coordinatorTaskEntry(void* pvParameters) {
        //convert void* back to ptr to coordinator object for fcn/var access
        static_cast<Coordinator*>(pvParameters)->coordinatorTask();
    }
    //actual signatue void Coordinator::coordinatorTask()
    void coordinatorTask() {
        while(1) {
            // tell stepper to move
            xTaskNotifyGive(stepperTaskHandle);
            
            // wait until moved
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // tell finger to play note
            xTaskNotifyGive(fingerTaskHandle);

            // wait until finger up
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }

};
