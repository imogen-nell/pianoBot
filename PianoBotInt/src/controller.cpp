#include "controller.h"
#include "position_sensor.h"
#include "position_actuator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//pid vals
static float Kp = 0.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;


//controller state only accessed by controller task
static int target_volt = 255;
static float previous_error = 0.0f;
static float integral = 0.0f;

//task config
static int control_loop_delay_ms = 2; //default 2ms
static TaskHandle_t controllerTaskHandle = nullptr;

//shared variable from sensor
extern volatile float current_volt;  // extern is vvvvv important!

//pid 
static int compute_pid(int target, float current){
    // float error = target - current;
    // integral += error * (control_loop_delay_ms / 1000.0f); //integral intime 
    // float derivative = (error - previous_error) / (control_loop_delay_ms / 1000.0f);
    // previous_error = error;

    // float output = Kp * error + Ki * integral + Kd * derivative;
    // return static_cast<int>(output);
    return target;
}

//controller task
static void controllerTask(void* pvParameters){
    while(1){

        int control_signal = compute_pid(target_volt, current_volt);
        //TODO clamp here ? 
        set_pwm(control_signal);

        vTaskDelay(control_loop_delay_ms / portTICK_PERIOD_MS);
    }
}


//public API
void init_controller(float kp, float ki, float kd, int loop_delay_ms){
    Kp = kp;
    Ki = ki;
    Kd = kd;
    control_loop_delay_ms = loop_delay_ms;

    //initialize sensor and actuator
    init_sensor();
    init_actuator();

    //create controller taskf
    xTaskCreatePinnedToCore(
        controllerTask,
        "controllerTask",
        4096, //stack size
        nullptr, //params
        1, //higher priority
        &controllerTaskHandle, //task handle
        1 //core
    );
}

//set in main 
void set_target(int newTarget) {
    target_volt = newTarget;
}