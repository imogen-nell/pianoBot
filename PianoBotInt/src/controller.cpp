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
static volatile float target_pos = 0.0f;
static float previous_error = 0.0f;
static float integral = 0.0f;
static const float VREF = 3.3f; //reference voltage for hall sensor

//task config
static TaskHandle_t controllerTaskHandle = nullptr;

//shared variable from sensor

//pid 
//** Takes in target voltage for hall sensor
//return pwm control value -255 to 255
static int compute_pid(float target, float current){
    float dt = 2 / 1000.0f; //loop time in seconds
    float error = target - current;
    integral += error * dt; //integral intime = loop delay 2ms
    float derivative = (error - previous_error) / dt;
    previous_error = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    //clamp
    output = ((output)<(-1.0f)?(-1.0f):((output)>(1.0f)?(1.0f):(output)));
    int pwm_ctrl = output * 255;
    Serial.printf("PID,%lu,%.4f,%d\n", millis(), output, pwm_ctrl);
    return static_cast<int>(pwm_ctrl);
    // return (int) target;
}

// //pid helper
// static int target_volt_to_pwm(float target_volt) {
//     //for debugging
//     return int(target_volt);
//     // return (int) ((target_volt * 255) / VREF);
// }

//controller task
static void controllerTask(void* pvParameters){
    while(1){

        int control_signal = compute_pid(target_pos, current_volt);
        //TODO clamp here ? 
        set_pwm(control_signal);

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


//public API
void init_controller(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;

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
//sets target position for controller
void set_target(float newTarget) {
    target_pos = newTarget;
}

//set target helper
// float target_position_to_voltage() {
//     return target_volt;
// }