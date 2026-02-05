#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/ledc.h"
#include "position_sensor.h"
#include "Arduino.h"

class VoiceCoilController {
public:
    // PWM values for finger positions
    enum PWM_t { UP = -100, DOWN = 200, REST = 0 };
    //constructor
    VoiceCoilController(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel,
                        float kp, float ki, float kd,
                        int* next_note_ptr, int notes_arr_len);

    TaskHandle_t getTaskHandle() const { return vcTaskHandle; }
    void setCoordinatorHandle(TaskHandle_t handle);
    
private:
    // --- Hardware ---
    const uint8_t PWM_PIN;
    const uint8_t DIR_PIN;
    const uint8_t PWM_CHANNEL;
    static constexpr uint32_t PWM_FREQ = 30000; // was 20kHz (audible range 20Hz-20Khz)
    static constexpr uint8_t PWM_RES = 8;

    // --- Task handle ---
    TaskHandle_t vcTaskHandle = nullptr;
    TaskHandle_t coordinatorTaskHandle = nullptr;
    
    // key array pointers
    int* next_note_ptr;
    int* start_addr;
    int* end_addr; //end of keys array

    // --- PID ---
    float Kp, Ki, Kd;
    float target_pos = 0.0f;
    float previous_error = 0.0f;
    float integral = 0.0f;
    static constexpr float dt = 0.002f;

    // --- Timers ---
    TimerHandle_t voice_coil_timer = nullptr; // 10 ms
    TimerHandle_t finger_up_timer = nullptr;  // 100 ms
    static constexpr uint32_t NOTE_DONE = (1 << 0);
    static constexpr uint32_t FINGER_UP_DONE = (1 << 1);

    // --- Private methods ---
    void init_timers();
    void send_pwm(int ctrl_pwm);
    void controllerTask();
    static void controllerTaskEntry(void* pvParameters);
    

    // --- Timer callbacks ---
    static void note_timer_cb(TimerHandle_t xTimer);
    static void finger_up_cb(TimerHandle_t xTimer);
};

// #pragma once
// #include <Arduino.h>
// //controller interface

// //** Initialize the PID controller with given gains */
// void init_controller(float kp, float ki, float kd);

// //** Set the target position for the controller */
// void set_target(float new_target);



// //set by sensor, read by controller
// extern volatile float current_position;


