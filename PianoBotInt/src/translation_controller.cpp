#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "t_controller.h"

//task config
static TaskHandle_t t_controllerTaskHandle = nullptr;

//esp32 pins
static const int STEP_PIN = 25;
static const int DIR_PIN  = 26;
const int HOME_SWITCH_PIN = 27; 


//global
static bool dirr = true;
static float currentPosition = 0.0f;

//home to far left end 
static void home_stepper(){
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN); // safe pull-down

    digitalWrite(DIR_PIN, true); //set direction to left
    vTaskDelay(pdMS_TO_TICKS(20)); 

    bool homed = false; // false = left 
    //move left until button 
    while (!homed) {
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(1000)); 
        if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
            delay(10); // small delay to filter bounce
            if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
                homed = true;
                Serial.println("--------------homing complete--------------"); // <-- print when homed


            }
        }
    }

}

//stepper task 
static void t_controllerTask(void* params){
    digitalWrite(DIR_PIN, false); //false = right

    for(int i =0; i<2500; i++){
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(700));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(700));    
    }

    while(1){
        vTaskDelay(pdMS_TO_TICKS(250)); 
    }

    while(1){
        digitalWrite(DIR_PIN, false); //false = right
        // digitalWrite(DIR_PIN, !dirr);
        vTaskDelay(pdMS_TO_TICKS(250)); 

        for(int i =0; i<200; i++){
            digitalWrite(STEP_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(700)); 
            digitalWrite(STEP_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(700));    
        }

        dirr = true; // reset direction
        
        vTaskDelay(pdMS_TO_TICKS(250)); // yield CPU properly
    }
}

//init stepper motor controller
void init_t_ctrl(){
    //initialize pins for stepper motor
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    //homing button
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN);
    Serial.println("------------------homing------------------"); // <-- print when homed

    home_stepper();



    //create controller task
    xTaskCreatePinnedToCore(
        t_controllerTask,          /* Task function. */
        "Controller Task",       /* name of task. */
        4096,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        2,                       /* priority of the task */
        &t_controllerTaskHandle,   /* Task handle to keep track of created task */
        0);      //CORE 0

}

