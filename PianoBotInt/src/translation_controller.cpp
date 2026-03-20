#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "t_controller.h"
#include "driver/rmt.h"
#include "keys.h"

StepperController* StepperController::instances[2] = {nullptr};
//init stepper motor controller
StepperController::StepperController(const StepperConfig& cfg, const key_entry* key_positions_start, int key_arr_len,EventGroupHandle_t syncGroup)
    : config(cfg), next_key_ptr(key_positions_start), key_start(key_positions_start),key_end(key_positions_start+key_arr_len),syncStartEventGroup(syncGroup)
{    
    // Serial.printf("starting key pos: %d at %d\n", *next_key_ptr, next_key_ptr);
    pinMode(config.DIR_PIN, OUTPUT);
    pinMode(config.HOME_SWITCH_PIN, INPUT_PULLDOWN);
    // pinMode(config.step_pin, OUTPUT);

    //begin homing
    Serial.printf("----------------- homing %d -----------\n", config.RMT_CH+1); 
    home();
    // Serial.printf("-------------- homing done %d ---------\n", config.RMT_CH);

    
    //allocate once
    step_buffer_capacity = config.MAX_KEYS * config.STEPS_PER_KEY ;

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
        8192,        //task Stack size 
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

    if (stepper == nullptr || stepper->taskHandle == nullptr || stepper->homing==1) return;

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
    while (coordinatorTaskHandle == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
    // int current_key = 0; //assume starts at key 0 after homing
    while(1){

        // Wait coordinator command (with safety timeout)
        //prevents inf blocking if coordinator fails
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0){
            continue;
        }
        //check if restart needed 
        if(next_key_ptr >= key_end){

            rehome();
            // Serial.printf("Motor %d: starting song\n", config.RMT_CH + 1);
        }
        // else{
            //for testing
            // Serial.printf("-------------- MOVING to Key: %d\n", *next_key_ptr);
            // Serial.println("Key position: " + String(current_key));

        int key_diff = next_key_ptr->key_pos - current_key;
        
        //move to next key 
        if(key_diff != 0){
            //move (keys, dirrection, step freq. Hz)
            move_keys(abs(key_diff), (key_diff > 0) ? direction::LEFT : direction::RIGHT, 5000);
            // Wait until RMT transmission finishes (from ISR) - callback will unblock
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }   

        //look at next key position
        next_key_ptr++;
        // }


        //tell coordinator ready for next sequence  or rehome finished 
        xTaskNotifyGive(coordinatorTaskHandle);

    }  
    
}

//moves stepper a number of keys in given direction
//          then updates current_key position
//arguments: keys - number of keys to move
//           dirr  - direction RIGHT/LEFT
//           step_time - time between step in ms (default 4ms)
void StepperController::move_keys(int keys, direction dirr, uint16_t hz ){
    //ensure move is in bounds
    //todo: shitty check tbh
    if(keys < 0 || keys > config.MAX_KEYS){
        int target_key = current_key + keys * ((dirr == direction::RIGHT) ? -1 : 1);

        Serial.println("-------------- out of bounds move ---------------");
        Serial.printf("current key: %d, target key: %d, motor: %d\n", current_key, target_key, config.RMT_CH+1);
        return;
    }


    digitalWrite(config.DIR_PIN, dirr);
    ets_delay_us(5);  // ESP32-safe microsecond delay

    uint32_t steps = (keys * config.STEPS_PER_KEY > step_buffer_capacity) ? step_buffer_capacity : keys * config.STEPS_PER_KEY;
    // Serial.println("-------------- moving ---------------\n");
    for(int i = 0; i < steps; i++){
        step_buffer[i] = trapezoid(steps, i);
    }
    // send step waveform from rmt_item array, NON BLOCKING
    //start RMT engine, DMA begin outputting step pulses, returns immediately
    //interrupt (callback within ISR) raised when RMT item complete
    rmt_write_items(config.RMT_CH, step_buffer, steps, false);

    //update current key position
    current_key += keys * ((dirr == direction::RIGHT) ? -1 : 1);
}



void StepperController::populate_step_buffer(uint16_t steps, uint16_t hz )
{
    uint32_t half_period_us = 1000000UL / hz / 2;

    for(int i = 0; i < steps; i++){
        step_buffer[i].level0 = 1;
        step_buffer[i].duration0 = half_period_us;
        step_buffer[i].level1 = 0;
        step_buffer[i].duration1 = half_period_us;
    }
}

rmt_item32_t StepperController::trapezoid(int steps, int stepCount) {   
    double vel_m = 0.95; // (m/s)
    double acc_m = 12.0; // (m/s^2) 

    const double spm = 1600.0 / (2.0 * PI * 0.0175);
    double acc = spm * acc_m;  
    double vel = spm * vel_m;  

    double steps_to_max = (vel * vel) / (2.0 * acc);
    
    double actual_acc_steps = (steps_to_max < (steps / 2.0)) ? steps_to_max : (steps / 2.0);
    
    double c0 = 0.676 * 1000000.0 * sqrt(2.0 / acc);
    double actual_vel = sqrt(2.0 * acc * actual_acc_steps);
    double cmin = 1000000.0 / actual_vel;

    double cn;
    int n = stepCount + 1;

    if (n <= actual_acc_steps) {
        // Acceleration phase
        cn = c0 * (sqrt(n) - sqrt(n-1)); 
    } else if (n <= (steps - actual_acc_steps)) {
        // Cruise phase (only exists if move is long enough)
        cn = cmin;
    } else {
        // Deceleration phase
        int m = steps - n + 1;
        cn = c0 * (sqrt(m) - sqrt(m-1));
    }

    // Safety floor
    if (cn < cmin) cn = cmin;

    uint32_t half_period_us = (uint32_t)(cn / 2.0);

    rmt_item32_t item;
    item.level0 = 1; item.duration0 = half_period_us;    
    item.level1 = 0; item.duration1 = half_period_us;   
    return item;
}

//home to leftmost key 
void StepperController::home(){
    pinMode(config.STEP_PIN, OUTPUT);
    digitalWrite(config.DIR_PIN, direction::LEFT);
    //move left until hm switch is hit
    //make faster : reduce delay, but may cause missed steps and less accuracy
    while (digitalRead(config.HOME_SWITCH_PIN) == LOW) {
        digitalWrite(config.STEP_PIN, HIGH);
        ets_delay_us(100);
        // vTaskDelay(pdMS_TO_TICKS(1));    
        digitalWrite(config.STEP_PIN, LOW);
        // vTaskDelay(pdMS_TO_TICKS(1)); 
        ets_delay_us(100);
    }
    Serial.printf("--------------  motor %d homed ---------------\n", config.RMT_CH+1, next_key_ptr->key_pos);


    //update positoin
    if(config.RMT_CH == 0){current_key = 32;}
    else{current_key = 57;}

    digitalWrite(config.DIR_PIN, direction::RIGHT);
    //move to first key manually
    for(int i = 0; i < abs(current_key - next_key_ptr->key_pos) * config.STEPS_PER_KEY; i++){
        digitalWrite(config.STEP_PIN, HIGH);
        ets_delay_us(100); 
        digitalWrite(config.STEP_PIN, LOW);
        ets_delay_us(100);
    }
    Serial.printf("--------------  motor %d at start key %d ---------------\n", config.RMT_CH+1, next_key_ptr->key_pos);


    current_key = next_key_ptr->key_pos;
}

//re home steepper ( after RMT is set up )
//should not notify main ctrlr until rehoming complete 
void StepperController::rehome( ){
    Serial.printf("---- REHOME Motor %d ----\n", config.RMT_CH+1);
    homing = 1;

    digitalWrite(config.DIR_PIN, direction::LEFT);
    ets_delay_us(5); 


    uint16_t steps = 35; // small move  
    uint16_t hz = 5000; // fast move
    populate_step_buffer(steps, hz);

    while(digitalRead(config.HOME_SWITCH_PIN) == LOW) {
        //  true to wait for all items to be sent before returning
        rmt_write_items(config.RMT_CH, step_buffer, steps, homing == 1);

    }
    //stop RMT in case still running
    rmt_tx_stop(config.RMT_CH);
    // Serial.printf("Motor %d: home hit \n", config.RMT_CH + 1);
  
    //update position to home key (leftmost)
    current_key = (config.RMT_CH == 0) ? 32 : 57;
    digitalWrite(config.DIR_PIN, direction::LEFT);
    ets_delay_us(5); 
    // //move to start key here 
    
    int key_diff = key_start->key_pos - current_key;
    // Serial.printf("-------------- moving motor %d to start key %d, %d keys over ---------------\n", config.RMT_CH+1, next_key_ptr->key_pos, abs(key_diff));
    if(key_diff != 0) {
        digitalWrite(config.DIR_PIN, (key_diff > 0) ? direction::LEFT : direction::RIGHT);
        ets_delay_us(10);

        uint32_t total_steps_needed = abs(key_diff) * config.STEPS_PER_KEY;
        
        // Use a small, SAFE chunk of steps
        //  ensures we never overflow the buffer
        uint16_t chunk_size = 64; 
        populate_step_buffer(chunk_size, hz);

        // Send chunks until done
        uint32_t steps_sent = 0;
        while(steps_sent < total_steps_needed) {
            uint32_t to_send = (total_steps_needed - steps_sent > chunk_size) ? chunk_size : (total_steps_needed - steps_sent);
            // true = wait for completion so we don't overwhelm the RMT
            rmt_write_items(config.RMT_CH, step_buffer, to_send, true);
            steps_sent += to_send;
        }
    }
    current_key = key_start->key_pos; //update position to start key after move
    // Serial.printf("--------------  motor %d at start key %d ---------------\n", config.RMT_CH+1, *next_key_ptr);

    next_key_ptr = key_start; //reset song position to start after rehome
    // SYNC BARRIER ---
    const EventBits_t finger1Bit = (1 << 0);
    const EventBits_t finger2Bit = (1 << 1);
    const EventBits_t allReadyBits = finger1Bit | finger2Bit;
    EventBits_t thisBit = (config.RMT_CH == 0) ? finger1Bit : finger2Bit;
    next_key_ptr = key_start; //reset song position to start after rehome
    if (syncStartEventGroup != NULL) {
        //motor waitinghere 
        xEventGroupSync(
            syncStartEventGroup,
            thisBit,         
            allReadyBits,    
            portMAX_DELAY    
        );
    }

    //clear any notifications from homing process to prevent false triggers in main loop
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF);
    homing = 0; //re eneable normal operation

}