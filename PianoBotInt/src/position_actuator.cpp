#include <Arduino.h>
#include "position_actuator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

const int PWM_PIN = 5;
const int DIR_PIN = 2;
// static int channel = 0;       // LEDC channel (0–15)
static int resolution = 8;    // 8-bit PWM (0–255)
static int freq = 5000;       // 5 kHz, max 20kHz for motor driver

//control val
static volatile int ctrl_pwm = 0; //shared variable
 
//rtos task handle
static TaskHandle_t actuatorTaskHandle = nullptr;


//actuator control task
//sets pwm based on ctrl_pwm variable
void actuatorTask(void* params){ //FreeRTOS mus return void  & accept single arg
    while(1){
        int pwm = ctrl_pwm;
        bool dir = pwm >= 0;
        //clamp
        if (pwm > 255) {
            pwm = 255;
        }
        //for debugging
        // Serial.printf("PWM log,%lu,%d\n", millis(), pwm);
        digitalWrite(DIR_PIN, dir); // Set direction
        analogWrite(PWM_PIN, pwm); // Set PWM value
        vTaskDelay(2 / portTICK_PERIOD_MS); // run every 2 ms
    }
}

//implementation
void init_actuator(){
    //initialize pins
    pinMode(DIR_PIN, OUTPUT);
    // analogWriteFrequency(freq) ; //default 5khz

    //create FreeRTOS task
    xTaskCreatePinnedToCore(
        actuatorTask,
        "actuatorTask",
        4096, //stack size, 4kB
        NULL, //params
        1, //priority
        &actuatorTaskHandle, //task handle
        0 //core 0: 
    );
}

void set_pwm(int pwm_value){
    ctrl_pwm = pwm_value;
}

void stop_actuator(){
    // kill task
    if (actuatorTaskHandle != nullptr) {
        vTaskDelete(actuatorTaskHandle);
        actuatorTaskHandle = nullptr;
    }
}