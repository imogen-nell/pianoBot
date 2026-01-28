#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "t_controller.h"
#include "driver/rmt.h"


//esp32 pins allocated to stepper
// #define STEP_PIN GPIO_NUM_25
// #define DIR_PIN 26
// #define HOME_SWITCH_PIN 27
// #define RMT_CH RMT_CHANNEL_0

//number of keys reachable by finger
// #define MAX_KEYS 13
// #define STEPS_PER_KEY 54 * 8
// step freq reliable range: 1 kHz – 50 kHz
//current key position
// static int current_key = 0;

//limit hit flag
// static bool hit_limit = false;

// TaskHandle_t t_controllerTaskHandle = nullptr;



//init stepper motor controller
StepperController::StepperController(const StepperConfig& cfg, int* key_positions_start, int* key_positions_end)
    : config(cfg), next_key_ptr(key_positions_start), key_end(key_positions_end)
{    

    key_start = key_positions_start;
    //init pins 
    pinMode(config.DIR_PIN, OUTPUT);
    pinMode(config.HOME_SWITCH_PIN, INPUT_PULLDOWN);
    // pinMode(config.step_pin, OUTPUT);

    //begin homing
    Serial.println("----------------- homing -----------------"); 
    home();
    Serial.println("-------------- homing done ---------------");

    // RMT channel config
    //RMT -remote- module driver used to generate step pulses 
    //(waveform) with highly specific pulse duration 
    rmt_config_t rmt_cfg  = {}; //produced HIGH for X ticks and LOW for Y ticks

    //configure chanel in transmit mode 
    rmt_cfg .rmt_mode = RMT_MODE_TX;
    rmt_cfg .channel = config.RMT_CH;
    rmt_cfg .gpio_num = config.STEP_PIN;
    rmt_cfg .clk_div = 80;                 // 1 µs resolution
    rmt_cfg .mem_block_num = 1;
    rmt_cfg .tx_config.loop_en = false; 
    rmt_cfg .tx_config.carrier_en = false; //disable carrier signal
    rmt_cfg .tx_config.idle_output_en = true; //enable RMT output if idle
    rmt_cfg .tx_config.idle_level = RMT_IDLE_LEVEL_LOW; //signal level out output when idle

    rmt_config(&rmt_cfg );
    rmt_driver_install(rmt_cfg.channel, 0, 0); //RX buffer not used, default flags 

    //register RMT callback, called when transmittion ends (after moving stepper)
    rmt_register_tx_end_callback(rmt_tx_done_cb, (void*)this); //pass this for ISR

    // create main controller task which takes care of moving to desired key positions
    xTaskCreatePinnedToCore(
        taskEntry,         
        "StepperTask",   
        4096,                    /* Stack size of task */
        this,                   
        1,                       /* priority of the task */
        &taskHandle, 
        0);      //CORE 


}

// FreeRTOS entry point//same logic and main ctrlr
void StepperController::taskEntry(void* pvParameters) {
    static_cast<StepperController*>(pvParameters)->run();
}


//IRAM_ATTR : RMT TX done ISR
// callback occurs within the ISR, 
//the ISR is initiated by hardware when RMT transmission completes
void IRAM_ATTR StepperController::rmt_tx_done_cb(rmt_channel_t channel, void *arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //notify stepper task, unblock t_ctrlrtask
    StepperController* stepper = static_cast<StepperController*>(arg);
    xTaskNotifyFromISR(
        stepper->taskHandle, // task to notify
        0, // no value
        eNoAction,//no action on value
        &xHigherPriorityTaskWoken // higher priority task woken flag
    );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // yield if higher priority task was woken
}

//stepper task to move to target positions
void StepperController::run(){
    while(1){

        // Wait for coordinator command
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //for testing
        // Serial.printf("-------------- MOVING to Key: %d\n", *next_key_ptr);
        // Serial.println("Key position: " + String(current_key));


        int key_diff = *next_key_ptr - current_key;
        
        //move to next key 
        if(key_diff != 0){
            //move (keys, dirrection, step freq. Hz)
            move_keys(abs(key_diff), (key_diff > 0) ? direction::RIGHT : direction::LEFT, 10000);
            // Wait until RMT transmission finishes (from ISR)
            //ie block stepper task until RMT hardware signals that all step pulses complete
            // wait until RMT finished (callback will unblock)
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }   

        //look at next key position
        next_key_ptr++;
        
        if(next_key_ptr >= key_end){
            Serial.println("Resetting key positions");
            next_key_ptr = key_start; //reset key positions to start of array
        }

    }  
    
}

//moves stepper a number of keys in given direction
//arguments: keys - number of keys to move
//           dirr  - direction RIGHT/LEFT
//           step_time - time between step in ms (default 4ms)
void StepperController::move_keys(int keys, direction dirr, uint16_t hz , bool homing ){
    //ensure move is in bounds
    if (!homing){
        int target_key = current_key + keys * ((dirr == direction::RIGHT) ? 1 : -1);
        if(target_key < 0 || target_key > config.MAX_KEYS){
            Serial.println("-------------- out of bounds move ---------------");
            return;
        }
    }

    digitalWrite(config.DIR_PIN, dirr);

    uint32_t steps = keys * config.STEPS_PER_KEY;
    //create buffer for steps
    rmt_item32_t *step_buffer =(rmt_item32_t *) heap_caps_malloc( // returns void pointer, cast to rmt_item32_t*
        sizeof(rmt_item32_t) * steps, 
        MALLOC_CAP_DMA
    ); // heap caps guarantees memory is accessible by DMA

    if(!step_buffer) return; // allocation failed
    
    //fill items array with step pulses
    for(int i = 0; i < steps; i++){
        step_buffer[i] = stepPulseAtHz(hz); //25 kHz step pulse
    }

    // send step waveform from rmt_item array, NON BLOCKING
    //start RMT engine, DMA begin outputting step pulses, returns immediately
    //interrupt (callback within ISR) raised when RMT item complete
    rmt_write_items(config.RMT_CH, step_buffer, steps, false);

    //update current key position
    current_key += keys * ((dirr == direction::RIGHT) ? 1 : -1);
}



rmt_item32_t StepperController::stepPulseAtHz(uint16_t hz )
{
    uint32_t half_period_us = 1000000UL / hz /2;

    rmt_item32_t item;
    item.level0 = 1; item.duration0 = half_period_us;   //20 us HIGH (25 kHz)
    item.level1 = 0; item.duration1 = half_period_us;   // 20 us LOW (25 kHz)
    return item;
}

//home to leftmost key 
void StepperController::home(){
    pinMode(config.STEP_PIN, OUTPUT);
    digitalWrite(config.DIR_PIN, direction::LEFT);
    //move left until home switch is hit
    while (digitalRead(config.HOME_SWITCH_PIN) == LOW) {
        digitalWrite(config.STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));    
        digitalWrite(config.STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(1)); 
        //no debounce rn:
        // if (digitalRead(config.HOME_SWITCH_PIN) == HIGH) {
        //     delay(1); // small delay to filter bounce
        //     if (digitalRead(config.HOME_SWITCH_PIN) == HIGH) {
        //         current_key = 0; //reset position
        //         return;
        //     }
        // }
    }
    //update positoin
    current_key = 0;
}