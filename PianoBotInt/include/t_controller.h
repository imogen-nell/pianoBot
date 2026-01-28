#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"

// Stepper pins & RMT channel can be customized per instance
struct StepperConfig {
    gpio_num_t STEP_PIN;
    gpio_num_t DIR_PIN;
    gpio_num_t HOME_SWITCH_PIN;
    rmt_channel_t RMT_CH;
    //all finges same stepsper key and max keys for now
    static constexpr int MAX_KEYS = 13; 
    static constexpr int STEPS_PER_KEY = 30 * 8;
};




class StepperController {
public:
    enum direction {RIGHT,LEFT};
    
    StepperController(const StepperConfig& cfg, int* key_positions_start, int key_arr_len);

    TaskHandle_t getTaskHandle() const { return taskHandle; }
    void setCoordinatorHandle(TaskHandle_t handle){this->coordinatorTaskHandle = handle;};


private:
    // hardware config
    StepperConfig config;

    // current position
    int current_key = 0;
    rmt_item32_t* active_buffer = nullptr;


    // key array pointers
    int* next_key_ptr; //moves through the array 
    int* const key_start; //const song array for now
    int* const key_end; //end of keys array

    // task handle
    TaskHandle_t taskHandle = nullptr;
    TaskHandle_t coordinatorTaskHandle = nullptr;

    // main stepper task loop
    void run();

    // helper
    void home();
    void move_keys(int keys, direction dirr, uint16_t hz = 10000, bool homing = false);
    static inline rmt_item32_t stepPulseAtHz(uint16_t hz);

    // FreeRTOS entry wrapper
    static void taskEntry(void* pvParameters);

    // RMT callback
    static void IRAM_ATTR rmt_tx_done_cb(rmt_channel_t channel, void* arg);
};
