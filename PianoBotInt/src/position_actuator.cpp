#include <Arduino.h>
#include "position_actuator.h"
#include "controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//esp32 pins
const int PWM_PIN = 5;
const int DIR_PIN = 2;

// static int resolution = 8;    // 8-bit PWM (0–255)
static int freq = 10000;       // 10 kHz, max 20kHz for motor driver

 
//rtos task handle
static TaskHandle_t actuatorTaskHandle = nullptr;

//send relevant pwm signal to voice coil
void actuatorTask(void* params){ //FreeRTOS mus return void  & accept single arg
    while(1){
        //ctrl_pwm shared variable, set by controller
        bool dir = ctrl_pwm >= 0;
        digitalWrite(DIR_PIN, dir); // Set direction
        analogWrite(PWM_PIN, ctrl_pwm); // Set PWM value

        vTaskDelay(2 / portTICK_PERIOD_MS); // run every 2 ms
    }
}

//implementation
void init_actuator(){
    //initialize pins to motor driver
    pinMode(DIR_PIN, OUTPUT);
    analogWriteFrequency(freq) ; //default 5khz

    //create FreeRTOS task
    xTaskCreatePinnedToCore(
        actuatorTask,
        "actuatorTask",
        4096, //stack size, 4kB (could be less??)
        NULL, //params
        1, //priority
        &actuatorTaskHandle, //task handle
        0 //core 0: 
    );
}

