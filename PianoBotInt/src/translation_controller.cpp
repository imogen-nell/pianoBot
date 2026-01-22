#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "t_controller.h"
#include "stepper_tests.h"
#include "global.h"

//task config
static TaskHandle_t t_controllerTaskHandle = nullptr;
static TaskHandle_t t_limitCheckTaskHandle = nullptr;

//esp32 pins
static const int STEP_PIN = 25;
static const int DIR_PIN  = 26;
const int HOME_SWITCH_PIN = 27; 

//const int RIGHT_LIMIT_PIN = 14; //not used yet
#define MAX_KEYS 8

//global
static bool dirr = true;
static int currentKey = 0;
static bool hit_limit = false;




enum direction {RIGHT,LEFT};

//home to far left end 
static void home_stepper(){
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN); // safe pull-down

    digitalWrite(DIR_PIN, direction::LEFT); 
    vTaskDelay(pdMS_TO_TICKS(20)); 

    bool homed = false; // false = left 
    //move left until button 
    while (!homed) {
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(4));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(4)); 
        if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
            delay(5); // small delay to filter bounce
            if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
                homed = true;
                currentKey = 0; //reset position
            }
        }
    }
}

static void move_steps(int steps, direction dirr){
    digitalWrite(DIR_PIN, dirr); //false = right
    for(int i =0; i<steps; i++){
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(4));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(4));    
    }
}

// static void stop_stepper(){
//     //TODO implement stop function
//     //send 0 PWM to stepper driver
//     digitalWrite(STEP_PIN, LOW);
//     vTaskDelay(pdMS_TO_TICKS(2500)); 
// } 

// // limit checking - stay in bounds 
// static void t_limitCheckTask(void* params){
//     //check left limit
//     if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
//         delay(10); // small delay to filter bounce
//         if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
//             Serial.println("--------------hit limit--------------"); // <-- print when homed
//             stop_stepper();
//             move_steps(50, direction::RIGHT); //move off button
            
//             // re initialise current key 
//             currentKey = 0;
//         }
//     }
//     vTaskDelay(pdMS_TO_TICKS(10)); //yield CPU properly
    
// }

// //stepper task to move to target positions
static void t_controllerTask(void* params){
    //get next key (position) if next key = none
    while(1){
        Serial.println("-------------- WAITING FOR MUTEX ---------------");
        if (xSemaphoreTake(ctrl_Mutex, portMAX_DELAY)) { //maxdeley is wait time fo rmutex 
            //update global state variables


            // load next key 
            Serial.println("-------------- MOVING STEPS ---------------");
            move_steps(20, direction::RIGHT);
            currentKey++;
            Serial.print("Current Key Position: ");
            Serial.println(currentKey);
            

            

            
            //release mutex
            xSemaphoreGive(ctrl_Mutex);

        }
        
        if(currentKey > MAX_KEYS){
                //home again 
                home_stepper();
            }

        // wait for key to play
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    
}

//init stepper motor controller
void init_t_ctrl(){
    //initialize pins for stepper motor
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(HOME_SWITCH_PIN, INPUT_PULLDOWN);
    //stepper should have the mutex to start 
    // xSemaphoreTake(motorMutex, portMAX_DELAY);

    //begin homing
    Serial.println("----------------- homing -----------------"); 
    home_stepper();
    Serial.println("-------------- homing done ---------------");


    // begin background limit checking task TODO: how to ovveride current task - should only hit button if stepper has mutex 
    // xTaskCreatePinnedToCore(
    //     t_limitCheckTask,          /* Task function. */
    //     "Limit Check Task",       /* name of task. */
    //     2048,                    /* Stack size of task */
    //     NULL,                    /* parameter of the task */
    //     3,                       /* priority of the task */
    //     &t_limitCheckTaskHandle,   /* Task handle to keep track of created task */
    //     1);      //CORE 1

    // create main controller task which takes care of moving to positions
    Serial.println("-------------- TASK START ---------------");
    xTaskCreatePinnedToCore(
        t_controllerTask,          /* Task function. */
        "Controller Task",       /* name of task. */
        4096,                    /* Stack size of task */
        NULL,                    /* parameter of the task */
        2,                       /* priority of the task */
        &t_controllerTaskHandle,   /* Task handle to keep track of created task */
        0);      //CORE 0


}

