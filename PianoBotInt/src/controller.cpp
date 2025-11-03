#include "controller.h"
#include "position_sensor.h"
#include "position_actuator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//system vals
const float PRESSED = 1.81f;
const float RELEASED = 1.08f;

//pid vals
static float Kp = 0.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;


//controller state only accessed by controller task
static float target_pos = 0.0f;
static float previous_error = 0.0f;
// static  float previous_position = 0.0f;
static  float integral = 0.0f; 
// static const float VREF = 3.3f; //reference voltage for hall sensor
static const float dt = 0.002f; //loop time in seconds
//task config
static TaskHandle_t controllerTaskHandle = nullptr;

//init shared variables
volatile int ctrl_pwm = 0;
volatile float current_position = 0.0f;


//pid controller
//** Takes in target voltage for hall sensor
//return pwm control value -255 to 255
//target and current are VOLTAGES 
static void set_pwm(void){
    float error =  target_pos-current_position;
    integral +=   error * dt; //integral intime = loop delay 2ms
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    //clamp integral
    integral = ((integral)<(-0.5f)?(-0.5f):((integral)>(0.5f)?(0.5f):(integral)));

    float output = Kp * error + Ki * integral + Kd * derivative;
    //clamp integral error 


    //clamp ing
    output = ((output)<(-1.0f)?(-1.0f):((output)>(1.0f)?(1.0f):(output)));
    ctrl_pwm = (int) (output * 255);
}

//controller task
//reads sensor, computes control, sets actuator command
static void controllerTask(void* pvParameters){
    while(1){
        
        set_pwm();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


//public API
void init_controller(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;

    // //initialize linear motion sensor and actuator
    init_sensor();
    init_actuator();

    //create controller task
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


//sets target voltage (position) for controller
void set_target(float target_PWM) {
    //convert pwm to voltage 
    float m = (PRESSED-RELEASED)/255.0f;
    target_pos = target_PWM * m + RELEASED ; 
    Serial.printf("CTRL,%lu,%.4f\n", millis(),target_pos);

}
