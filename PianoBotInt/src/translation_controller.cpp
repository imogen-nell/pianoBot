#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "t_controller.h"
#include "driver/rmt.h"
StepperController* StepperController::instances[2] = {nullptr};
//init stepper motor controller
StepperController::StepperController(const StepperConfig& cfg, int* key_positions_start, int key_arr_len)
    : config(cfg), next_key_ptr(key_positions_start), key_start(key_positions_start),key_end(key_positions_start+key_arr_len)
{    
    pinMode(config.DIR_PIN, OUTPUT);
    pinMode(config.HOME_SWITCH_PIN, INPUT_PULLDOWN);
    // pinMode(config.step_pin, OUTPUT);

    //begin homing
    Serial.printf("----------------- homing %d -----------------", config.RMT_CH); 
    home();
    Serial.printf("-------------- homing done %d ---------------", config.RMT_CH);

    
    //allocate once
    step_buffer_capacity = config.MAX_KEYS * config.STEPS_PER_KEY;

    step_buffer = (rmt_item32_t*) heap_caps_malloc(
        step_buffer_capacity * sizeof(rmt_item32_t),
        MALLOC_CAP_DMA
    );

    assert(step_buffer);

    

    // RMT channel config
    instances[config.RMT_CH] = this; //Store this instance in the global lookup table
    //RMT -remote- module driver used to generate step pulses 
    //(waveform) with highly specific pulse duration 
    rmt_config_t rmt_cfg  = {}; //produced HIGH for X ticks and LOW for Y ticks

    //configure chanel in transmit mode 
    rmt_cfg .rmt_mode = RMT_MODE_TX;
    rmt_cfg .channel = config.RMT_CH;
    rmt_cfg .gpio_num = config.STEP_PIN;
    rmt_cfg .clk_div = 80;                 // 1 µs resolution (1-255), 1 Mhz, default 80 Mhz
    rmt_cfg .mem_block_num = 2;
    rmt_cfg .tx_config.loop_en = false; 
    rmt_cfg .tx_config.carrier_en = false; //disable carrier signal
    rmt_cfg .tx_config.idle_output_en = true; //enable RMT output if idle
    rmt_cfg .tx_config.idle_level = RMT_IDLE_LEVEL_LOW; //signal level out output when idle

    rmt_config(&rmt_cfg );
    rmt_driver_install(rmt_cfg.channel, 0, 0); //RX buffer not used, default flags 
    
    // Register the callback ONCE (it's okay to call it multiple times, 
    // but they will all point to the same static dispatcher now)
    rmt_register_tx_end_callback(StepperController::global_rmt_tx_done_cb, nullptr);
    // create main controller task which takes care of moving to desired key positions
    xTaskCreatePinnedToCore(
        taskEntry,         
        "StepperTask",   
        4096,        //task Stack size 
        this,                   
        3,       //higher priority for workers 
        &taskHandle, 
        config.RMT_CH == 0 ? 0 : 1);      //CORE 
}

// FreeRTOS entry point//same logic and main ctrlr
void StepperController::taskEntry(void* pvParameters) {
    static_cast<StepperController*>(pvParameters)->run();
}


//IRAM_ATTR : RMT TX done ISR
// callback occurs within the ISR, 
//the ISR is initiated by hardware when RMT transmission completes
void IRAM_ATTR StepperController::global_rmt_tx_done_cb(rmt_channel_t channel, void *arg) {
    // Serial.printf("RMT TX done on channel %d, notifying stepper task at %p\n", channel, arg);
    
    // Look up the correct object using the channel number provided by hardware
    StepperController* stepper = instances[channel];

    if (stepper == nullptr || stepper->taskHandle == nullptr) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xTaskNotifyFromISR(
        stepper->taskHandle, // task to notify
        0, // no value
        eNoAction,//no action on value
        &xHigherPriorityTaskWoken // higher priority task woken flag
    );
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken); //yeild if flagged
}

//stepper task to move to target positions
void StepperController::run(){

    while(1){

        // Wait coordinator command (with safety timeout)
        //prevents inf blocking if coordinator fails
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0){
            continue;
        }

        //for testing
        // Serial.printf("-------------- MOVING to Key: %d\n", *next_key_ptr);
        // Serial.println("Key position: " + String(current_key));


        int key_diff = *next_key_ptr - current_key;
        
        //move to next key 
        if(key_diff != 0){
            //move (keys, dirrection, step freq. Hz)
            move_keys(abs(key_diff), (key_diff > 0) ? direction::RIGHT : direction::LEFT, 5000);
            // Wait until RMT transmission finishes (from ISR) - callback will unblock
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }   

        //look at next key position
        next_key_ptr++;
        
        if(next_key_ptr >= key_end){
            next_key_ptr = key_start; //reset key positions to start of array
        }
        xTaskNotifyGive(coordinatorTaskHandle);

    }  
    
}

//moves stepper a number of keys in given direction
//          then updates current_key position
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
    ets_delay_us(5);  // ESP32-safe microsecond delay

    uint32_t steps = keys * config.STEPS_PER_KEY;
    // Serial.println("-------------- moving ---------------");
    for(int i = 0; i < steps; i++){
        // step_buffer[i] = stepPulseAtHz(hz); //25 kHz step pulse
        step_buffer[i] = trapezoid(steps, i);
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
    item.level0 = 1; 
    item.duration0 = half_period_us;   //20 us HIGH 
    item.level1 = 0; 
    item.duration1 = half_period_us;   // 20 us LOW 
    return item;
}

rmt_item32_t StepperController::trapezoid(int steps, int stepCount){   
    const double spm = 1600.0/(2.0*PI*0.0175);
    const double acc = spm*5.0; // steps/sec^2
    const double vel = spm*0.35; // step/s
    const double acc_step = (vel*vel) / (2.0*acc);
    
    const double cmin = 1000000.0/vel;
    const double c0 = 0.676*1000000.0*sqrt(2.0/acc);
    static double cn;

    rmt_item32_t item;
    int n = stepCount+1;
    if(n == 1){cn = c0;}

    //acceleration
    if(n<=acc_step){
        cn = cn - (2.0*cn) / (4.0*n+1.0);
    }

    //constant velocity
    else if (n<=(steps - acc_step) && steps>(2*acc_step)){
        cn = cmin;
    }

    //deceleration
    else{
        int m = steps - n + 1;
        cn = cn + (2.0*cn) / (4.0*m+1.0);
    }

    if(cn < cmin) cn = cmin;
    uint32_t half_period_us = (uint32_t)(cn/2.0); // 50% duty cycle

    item.level0 = 1; 
    item.duration0 = half_period_us;    
    item.level1 = 0; 
    item.duration1 = half_period_us;   
            

    return item;
}

//home to leftmost key 
void StepperController::home(){
    pinMode(config.STEP_PIN, OUTPUT);
    digitalWrite(config.DIR_PIN, direction::LEFT);
    //move left until hm switch is hit
    while (digitalRead(config.HOME_SWITCH_PIN) == LOW) {
        digitalWrite(config.STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));    
        digitalWrite(config.STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }

    //update positoin
    current_key = 0;
    
    digitalWrite(config.DIR_PIN, direction::RIGHT);
    //move to first key manually
    Serial.printf("-------------- moving to start key %d ---------------", current_key);
    for(int i = 0; i < *next_key_ptr * config.STEPS_PER_KEY; i++){
        digitalWrite(config.STEP_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));    
        digitalWrite(config.STEP_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
    current_key = *next_key_ptr;
}