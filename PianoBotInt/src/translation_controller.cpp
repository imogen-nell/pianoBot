#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "t_controller.h"
#include "stepper_tests.h"
#include "global.h"

//task config
static TaskHandle_t t_controllerTaskHandle = nullptr;
static TaskHandle_t t_limitCheckTaskHandle = nullptr;

//esp32 pins allocated to stepper
static const int STEP_PIN = 25;
static const int DIR_PIN  = 26;
const int HOME_SWITCH_PIN = 27; 

//number of keys reachable by finger
#define MAX_KEYS 13
#define STEPS_PER_KEY 54 

//current key position
static int current_key = 0;

//limit hit flag
static bool hit_limit = false;

//pwm direction values
enum direction {RIGHT,LEFT};

//home to leftmost key 
static void home_stepper(int step_time = 4){
    //home left
    digitalWrite(DIR_PIN, direction::LEFT); 
    vTaskDelay(pdMS_TO_TICKS(5)); 

    //move left until home switch is hit
    while (1) {
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(step_time));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(step_time)); 

        if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
            delay(2); // small delay to filter bounce
            if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
                current_key = 0; //reset position
                return;
            }
        }
    }
}

//moves stepper a number of steps in given direction and updates position
//arguments: steps - number of steps to move
//           dirr  - direction RIGHT/LEFT
//           step_time - time between step in ms (default 4ms)
static void move_steps(int steps, direction dirr, int step_time = 4){
    //ensure move is in bounds
    if(current_key + steps * ((dirr == direction::RIGHT) ? 1 : -1)/ STEPS_PER_KEY < 0 ||
       current_key + steps * ((dirr == direction::RIGHT) ? 1 : -1)/ STEPS_PER_KEY > MAX_KEYS){
        Serial.println("-------------- out of bounds move ---------------");
        return;
    }
    //set motor dirrection
    digitalWrite(DIR_PIN, dirr);
    //move steps
    for(int i =0; i<steps; i++){
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(step_time));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(step_time));    
    }
    //update current key position
    current_key += steps * ((dirr == direction::RIGHT) ? 1 : -1) / STEPS_PER_KEY;
}

//moves stepper a number of keys in given direction
//arguments: keys - number of keys to move
//           dirr  - direction RIGHT/LEFT
//           step_time - time between step in ms (default 4ms)
static void move_keys(int keys, direction dirr, int step_time = 4){
    //ensure move is in bounds
    if(current_key + keys * ((dirr == direction::RIGHT) ? 1 : -1) < 0 ||
       current_key + keys * ((dirr == direction::RIGHT) ? 1 : -1) > MAX_KEYS){
        Serial.println("-------------- out of bounds move ---------------");
        return;
    }
    //set motor dirrection
    digitalWrite(DIR_PIN, dirr);
    //move steps
    for(int i =0; i<keys * STEPS_PER_KEY ; i++){
        digitalWrite(STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(step_time));    
        digitalWrite(STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(step_time));    
    }
    //update current key position
    current_key += keys * ((dirr == direction::RIGHT) ? 1 : -1);
}

//stops stepper motor movementand waits 2s
static void stop_motor(){
    //send 0 PWM to stepper driver
    digitalWrite(STEP_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(2000)); 
} 

// limit checking - stay in bounds 
static void t_limitCheckTask(void* params){
    //check left limit
    if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
        delay(5); // small delay to filter bounce
        if (digitalRead(HOME_SWITCH_PIN) == HIGH) {
            Serial.println("--------------hit limit--------------");
            stop_motor();
            //update position to 0 for left limit hit 
            current_key = 0;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); 
}

//stepper task to move to target positions
static void t_controllerTask(void* params){
    //for testing
    direction dirr = direction::RIGHT;
    while(1){

        //get next key (position)

        if (xSemaphoreTake(ctrl_Mutex, portMAX_DELAY)) { //wait indefinetly until mutex is avilable
            Serial.println("-------------- MOVE ---------------");
            Serial.println("Key position: " + String(current_key));

            //move to next key
            int next_key = *key_positions;

            int key_diff = next_key - current_key;
            if(key_diff > 0){
                dirr = direction::RIGHT;
            } else {
                dirr = direction::LEFT;
            }
            move_keys(abs(key_diff), dirr);
            //load next key position
            key_positions++;
            Serial.println("At key position: " + String(current_key));

            //for testing///////////
            // move_keys(1, dirr);
            // dirr = (dirr == direction::RIGHT) ? direction::LEFT : direction::RIGHT;//toggle direction for next move
            // ////////////////////////
        
            //release mutex
            xSemaphoreGive(ctrl_Mutex);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay before next move
        }   
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
    //     2,                       /* priority of the task */
    //     &t_limitCheckTaskHandle,   /* Task handle to keep track of created task */
    //     0);      //CORE 1

    // create main controller task which takes care of moving to positions
    xTaskCreatePinnedToCore(
        t_controllerTask,         
        "Controller Task",   
        4096,                    /* Stack size of task */
        NULL,                   
        1,                       /* priority of the task */
        &t_controllerTaskHandle, 
        0);      //CORE 


}

