//implementation for hall sensor
//data from hall to ADC input of esp32
#include <Arduino.h>
#include "position_sensor.h"
#include "vc_controller.h"

//calibration constants
static constexpr float ADC_MAX = 4095.0f;  // 12-bit ADC
static constexpr float VREF = 3.3f;  //  
static const int HALL_PIN = 32; // was 35 in old iteration  

//task handle
static TaskHandle_t hallReadTaskHandle = nullptr;


//reads hall sensor and update current_position
void hallReadTask(void* parameter){
    //TODO: convert voltage to position (mm) 
    while(1) {
        //set current position/voltage
        int adc_value = analogRead(HALL_PIN);
        current_position = (float) (adc_value / ADC_MAX) * VREF;

        //data logging
        // Serial.printf("Hall,%lu,%.4f\n", millis(), current_position);
        //loop period 1ms
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
}

void init_sensor( ){
    analogReadResolution(12); // ESP32 default
    // no need to set pinMode for analog input on esp32

    //begin vertical position read task
    xTaskCreatePinnedToCore(
        hallReadTask,
        "hallReadTask",
        4096, //stack size, 4kB
        NULL,
        2, // high priority
        &hallReadTaskHandle, //task handle
        1 //2 cores 0,1 
    );

}
