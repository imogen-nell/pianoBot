#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "t_controller.h"
#include "stepper_tests.h"
//task config
static TaskHandle_t t_controllerTaskHandle = nullptr;
static TaskHandle_t t_limitCheckTaskHandle = nullptr;

//esp32 pins
static const int STEP_PIN = 25;
static const int DIR_PIN  = 26;
const int HOME_SWITCH_PIN = 27; 
//const int RIGHT_LIMIT_PIN = 14; //not used yet

//global
static bool dirr = true;
static int currentKey = 0;
static bool hit_limit = false;
static bool running = false;

//home to far left end 
static void home_stepper(){
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN); // safe pull-down

    digitalWrite(DIR_PIN, true); //set direction to left
    vTaskDelay(pdMS_TO_TICKS(20)); 

    bool homed = false; // false = left 
    //move left until button 
    while (!homed) {
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(40));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(40)); 
        if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
            delay(5); // small delay to filter bounce
            if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
                homed = true;
                running =true;
                currentKey = 0; //reset position
            }
        }
    }
}

static void move_right(int steps){
    digitalWrite(DIR_PIN, false); //false = right
    for(int i =0; i<steps; i++){
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));    
    }
}

// static void move_left(int steps){
//     digitalWrite(DIR_PIN, true); //true = left
//     for(int i =0; i<steps; i++){
//         digitalWrite(STEP_PIN, HIGH);
//         vTaskDelay(pdMS_TO_TICKS(700));    
//         digitalWrite(STEP_PIN, LOW);
//         vTaskDelay(pdMS_TO_TICKS(700));    
//     }
// }

// static void stop_stepper(){
//     //TODO implement stop function
//     //send 0 PWM to stepper driver
//     running = false;
//     digitalWrite(STEP_PIN, LOW);
//     vTaskDelay(pdMS_TO_TICKS(2500)); 
// } 

//limit checking - stay in bounds 
// static void t_limitCheckTask(void* params){
//     //check left limit
//     if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
//         delay(10); // small delay to filter bounce
//         if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
//             Serial.println("--------------hit limit--------------"); // <-- print when homed
//             stop_stepper();
//             move_right(50);
//             // home_stepper();
//         }
//     }
//     vTaskDelay(pdMS_TO_TICKS(20)); //yield CPU properly
    
// }

//stepper task 
static void t_controllerTask(void* params){
    Serial.println("------------------running ------------------");
    bool dirr = true;
    while(1){   
        digitalWrite(DIR_PIN, dirr); //false = right

        for(int i =0; i<250; i++){
            digitalWrite(STEP_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(2));    
            digitalWrite(STEP_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(2));    
        }
        dirr = !dirr; //reverse direction
        vTaskDelay(pdMS_TO_TICKS(70));
    };
}

//init stepper motor controller
void init_t_ctrl(){
    //initialize pins for stepper motor
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    //homing button
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN);

    // //begin homing
    // Serial.println("------------------homing------------------"); // <-- print when homed
    // home_stepper();
    // Serial.println("------------------homing done ------------------");
    // move_right(20);
    //begin background limit checking task
    // xTaskCreatePinnedToCore(
    //     t_limitCheckTask,          /* Task function. */
    //     "Limit Check Task",       /* name of task. */
    //     2048,                    /* Stack size of task */
    //     NULL,                    /* parameter of the task */
    //     3,                       /* priority of the task */
    //     &t_limitCheckTaskHandle,   /* Task handle to keep track of created task */
    //     1);      //CORE 1



    // // create controller task
    // xTaskCreatePinnedToCore(
    //     t_controllerTask,          /* Task function. */
    //     "Controller Task",       /* name of task. */
    //     4096,                    /* Stack size of task */
    //     NULL,                    /* parameter of the task */
    //     2,                       /* priority of the task */
    //     &t_controllerTaskHandle,   /* Task handle to keep track of created task */
    //     0);      //CORE 0


    //run test
    back_and_forth_test(DIR_PIN, STEP_PIN);

}

