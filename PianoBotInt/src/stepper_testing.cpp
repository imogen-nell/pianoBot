#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void back_and_forth_test(int DIR_PIN, int STEP_PIN){
    Serial.println("------------------back and for test ------------------");
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

void max_speed_test(int DIR_PIN, int STEP_PIN){
    Serial.println("-----Max speed testing----");
    bool dirr = true; // moving left
    const int stepDelay = 300; //control speed in microseconds
    // 1/8 microstep
    int steps = 1600; //  1 rev
    
    unsigned long start_time = 0;
    unsigned long end_time = 0;

    digitalWrite(DIR_PIN, dirr);
    while(1){
        for(int i=0; i<steps; i++){
            if (i == 0){
                start_time = micros(); // record start time
                Serial.println("Start time: " + String(start_time/1e6));
            }
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(stepDelay);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(stepDelay);
            if (i == steps - 1){
                end_time = micros(); // record end time
                Serial.println("End time: " + String(end_time/1e6));
            }
        }
        float execute_time = (end_time - start_time) / 1e6; // sec
        int rpm = 1/execute_time * 60;
        Serial.println("Execution time: " + String(execute_time));
        Serial.println("RPM: " + String(rpm));
        delay(5000);
    }
}

void missed_step_test(int DIR_PIN, int STEP_PIN){
    // proceed for 1000 steps at various speeds
    Serial.println("-----Missed Step Testing ----");


}