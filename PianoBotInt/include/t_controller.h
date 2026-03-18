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
    static constexpr int MAX_KEYS = 25; 
    static constexpr int STEPS_PER_KEY = 35 * 4*3;

};




class StepperController {
public:
    enum direction {RIGHT, LEFT};
    
    StepperController(const StepperConfig& cfg, const int* key_positions_start, int key_arr_len, EventGroupHandle_t syncGroup);

    TaskHandle_t getTaskHandle() const { return taskHandle; }
    void setCoordinatorHandle(TaskHandle_t handle){this->coordinatorTaskHandle = handle;};
    
    static StepperController* instances[2]; // Array of pointers ( 2 for 2 fingers)
    static void IRAM_ATTR global_rmt_tx_done_cb(rmt_channel_t channel, void *arg);


private:
    //rehome sync
    EventGroupHandle_t syncStartEventGroup;
    // hardware config
    StepperConfig config;
    int homing = 0;

    // current position
    int current_key = 0;
    rmt_item32_t* step_buffer = nullptr;
    uint32_t step_buffer_capacity = 0;


    // key array pointers
    const int* next_key_ptr; //moves through the array 
    const int* key_start; //const song array for now
    const int* key_end; //end of keys array

    // task handle
    TaskHandle_t taskHandle = nullptr;
    TaskHandle_t coordinatorTaskHandle = NULL;

    // main stepper task loop
    void run();

    // helper
    void home();
    void move_keys(int keys, direction dirr, uint16_t hz = 10000, double acceleration=0.1);
    void populate_step_buffer(uint16_t steps, uint16_t hz);
    static inline rmt_item32_t trapezoid(int steps, int stepCount, direction dirr, double acceleration);
    void rehome();

    // FreeRTOS entry wrapper
    static void taskEntry(void* pvParameters);

    // RMT callback
    static void IRAM_ATTR rmt_tx_done_cb(rmt_channel_t channel, void* arg);
    //home button isr

    // void setupHomeInterrupt();//internal hardware setup
    // static void IRAM_ATTR home_switch_isr(void* arg); // static for gpio isr compatibilit
};
