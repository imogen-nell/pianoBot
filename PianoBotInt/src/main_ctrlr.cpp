#include "vc_controller.h"
#include "position_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "global.h"

//task config 
TaskHandle_t coordinatorTaskHandle = nullptr;
static TaskHandle_t fingerTaskHandle = nullptr;
static TaskHandle_t stepperTaskHandle = nullptr;

static void coordinatorTask(void * pvParameters){
    while(1) {
        // 1. tell finger to play note
        xTaskNotifyGive(fingerTaskHandle);

        // wait until finger fully up
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. tell stepper to move
        xTaskNotifyGive(stepperTaskHandle);

        // wait until stepper done moving
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}


void coordinator_init(TaskHandle_t fingerTask, TaskHandle_t stepperTask){
    fingerTaskHandle = fingerTask;
    stepperTaskHandle = stepperTask;
    xTaskCreatePinnedToCore(
        coordinatorTask,
        "coordinatorTask",
        4096,
        NULL,
        2,
        &coordinatorTaskHandle,
        0
    );
}
