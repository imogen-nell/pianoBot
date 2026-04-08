#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "keys.h"

// Stepper pins & RMT channel can be customized per instance
struct StepperConfig {
    gpio_num_t STEP_PIN;
    gpio_num_t DIR_PIN;
    gpio_num_t HOME_SWITCH_PIN;
    rmt_channel_t RMT_CH;
    int STEPS_PER_KEY;

    //all finges same stepsper key and max keys for now
    static constexpr int MAX_KEYS = 15; //was 25
    // static constexpr int STEPS_PER_KEY = 390; //400;//380;

};



class StepperController {
public:
    enum direction {RIGHT, LEFT};
    
    StepperController(const StepperConfig& cfg, const key_entry* key_positions_start, int key_arr_len, EventGroupHandle_t syncGroup);

    TaskHandle_t getTaskHandle() const { return taskHandle; }
    void setCoordinatorHandle(TaskHandle_t handle){this->coordinatorTaskHandle = handle;};
    
    static StepperController* instances[2]; // Array of pointers ( 2 for 2 fingers)
    static void IRAM_ATTR global_rmt_tx_done_cb(rmt_channel_t channel, void *arg);
    void set_physics(double short_vel, double short_accel, double long_vel, double long_accel){
        this->short_vel = short_vel;
        this->long_vel = long_vel;
        this->long_accel = long_accel;
        this->short_accel = short_accel;
    }

private:
    //rehome sync
    EventGroupHandle_t syncStartEventGroup;
    // hardware config
    StepperConfig config;
    //flag for isr to notify main ctrlr
    int isr_flag = 0;

    // current position .. transmission time 
    int current_key = 0;
    int curr_move_us = 0;

    //rmt signal buffers 
    rmt_item32_t* step_buffer = nullptr;
    rmt_item32_t* wait_buffer = nullptr; 

    uint32_t wait_buffer_capacity = 0;  
    uint32_t step_buffer_capacity = 0;
    


    // key array pointers
    const key_entry* next_key_ptr; //moves through the array 
    const key_entry* const key_start; //const song array for now
    const key_entry* const key_end; //end of keys array

    // task handle
    TaskHandle_t taskHandle = nullptr;
    TaskHandle_t coordinatorTaskHandle = NULL;

    // main stepper task loop
    void run();

    // helper
    void home();
    void move_keys(int keys, direction dirr, float time_ms = 20.0f);
    // void populate_step_buffer(uint16_t steps, uint16_t hz);
    std::pair<rmt_item32_t, int> trapezoid(int steps, int stepCount);
    double short_vel = 0.0; 
    double long_vel = 0.0;
    double short_accel = 0.0; 
    double long_accel = 0.0; 
    // void rehome();

    // FreeRTOS entry wrapper
    static void taskEntry(void* pvParameters);

    // RMT callback
    static void IRAM_ATTR rmt_tx_done_cb(rmt_channel_t channel, void* arg);
    //home button isr

    // void setupHomeInterrupt();//internal hardware setup
    // static void IRAM_ATTR home_switch_isr(void* arg); // static for gpio isr compatibilit
};
