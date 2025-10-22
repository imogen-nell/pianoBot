//implementation for hall sensor
//data from hall to ADC input of esp32
#include <Arduino.h>
#include "position_sensor.h"
//calibration constants
static constexpr float ADC_MAX = 4095.0f;  // 12-bit ADC
static constexpr float VREF = 3.3f;    
static const int HALL_PIN = 35;  //ADC1_7 GPIO35

static TaskHandle_t hallReadTaskHandle = nullptr;
//do i need to initialise shared variable here?
extern volatile float current_volt = 0.0f; //shared with controller task

void hallReadTask(void* parameter){
    //convert to position (mm) later!!!
    while(1) {
        int adc_value = analogRead(HALL_PIN);
        ///set shared variable for controller
        current_volt = (float) (adc_value / ADC_MAX) * VREF;
        //send to laptop for logging
        Serial.printf("Hall,%lu,%.4f\n", millis(), current_volt);
        vTaskDelay(2 / portTICK_PERIOD_MS); 
    }
}

void init_sensor( ){
    // analogReference(3.3); //set ref to 3.3V
    // analogReadResolution(12); // ESP32 default
    // pinMode(HALL_PIN, INPUT);
    // delay(100)

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
// void calibrate_sensor(){
//     //TODO implement calibration routine
// }