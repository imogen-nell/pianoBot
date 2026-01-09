#include <Arduino.h>
#include "position_sensor.h"


//esp32 pins
const int PWM_PIN = 5;
const int DIR_PIN = 2;

// static int resolution = 8;    // 8-bit PWM (0–255)
static int freq = 5000;       //  max 20kHz for motor driver

void init_ol(){
    pinMode(DIR_PIN, OUTPUT);
    init_sensor();

}

void play_song_ol(int *note_array, int size){
    for (int i = 0; i < size; i++) {
        int ctrl_pwm = note_array[i];
        bool dir = ctrl_pwm <= 0;
        digitalWrite(DIR_PIN, dir); // Set direction
        analogWrite(PWM_PIN, abs(ctrl_pwm)); // Set PWM value
        Serial.printf("CTRL,%lu,%d\n", millis(), ctrl_pwm);
        vTaskDelay(5 / portTICK_PERIOD_MS); // 10ms delay
    }
}
