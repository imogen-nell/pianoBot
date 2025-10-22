//implementation for hall sensor
//data from hall to ADC input of esp32
#include <Arduino.h>
#include "position_sensor.h"
//calibration constants
static constexpr float ADC_MAX = 4095.0f;  // 12-bit ADC
static constexpr float VREF = 3.3f;  // i think this is 1.1?   
static const int HALL_PIN = 35;  //ADC1_7 GPIO35

static TaskHandle_t hallReadTaskHandle = nullptr;//rtos task handle


extern volatile float current_position = 0.0f; //shared with controller task


//read hall sensor and update current_position
void hallReadTask(void* parameter){
    //TODO: convert voltage to position (mm) 
    while(1) {
        //set current position
        int adc_value = analogRead(HALL_PIN);
        current_position = (float) (adc_value / ADC_MAX) * VREF; //TODO: to mm 

        //send to laptop for logging
        Serial.printf("Hall,%lu,%.4f\n", millis(), current_position);
        vTaskDelay(2 / portTICK_PERIOD_MS); 
    }
}

void init_sensor( ){
    analogReadResolution(12); // ESP32 default
    // no need to set pinMode for analog input on esp32

    xTaskCreatePinnedToCore(
        hallReadTask,
        "hallReadTask",
        4096, //stack size, 4kB
        NULL, //params
        2, //priority
        &hallReadTaskHandle, //task handle
        1 //2 cores 0,1 
    );

}
