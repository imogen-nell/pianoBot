#include "controller.h"
#include "position_sensor.h"
// #include "position_actuator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "global.h"

//system vals
const float PRESSED = 1.81f;
const float RELEASED = 1.08f;

//pid vals
static float Kp = 0.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;

static float target_pos = 0.0f;
static float previous_error = 0.0f;
static  float integral = 0.0f; 
static const float dt = 0.002f; //loop time in seconds


//task config
static TaskHandle_t controllerTaskHandle = nullptr;

//init shared variables
volatile int ctrl_pwm = 0;
volatile float current_position = 0.0f;


//esp32 pins for voice coil driver
const int PWM_PIN = 5;
const int DIR_PIN = 2;


 
//rtos task handle
static TaskHandle_t actuatorTaskHandle = nullptr;

enum PWM_t {
    UP = -255,
    DOWN = 255,
    REST = 0
};


//** reads target voltage for hall sensor
//uses pid to set pwm control value between -255 to 255 accroding to target voltage (proportional to position)
static void set_pwm(void){
    float error =  target_pos-current_position;
    integral +=   error * dt; //integral intime = loop delay 2ms
    float derivative = (error - previous_error) / dt;
    previous_error = error;

    //clamp integral
    integral = ((integral)<(-0.5f)?(-0.5f):((integral)>(0.5f)?(0.5f):(integral)));

    float output = Kp * error + Ki * integral + Kd * derivative;    

    //clamping
    output = ((output)<(-1.0f)?(-1.0f):((output)>(1.0f)?(1.0f):(output)));
    ctrl_pwm = (int) (output * 255);
}

//sets target voltage (position) for controller
void set_target(float target_PWM) {
    //convert pwm to voltage 
    float m = (PRESSED-RELEASED)/255.0f;
    target_pos = target_PWM * m + RELEASED ; 
    Serial.printf("CTRL,%lu,%.4f\n", millis(),target_pos);

}


//send relevant pwm signal to voice coil
//sets DIR and PWM  according to ctrl_pwm
void sendPwmTask(void* params){ //FreeRTOS mus return void  & accept single arg
    while(1){
        //ctrl_pwm shared variable, set by controller
        bool dir = ctrl_pwm <= 0; 
        digitalWrite(DIR_PIN, dir); 
        analogWrite(PWM_PIN, abs(ctrl_pwm)); 
        //for data logger
        Serial.printf("PID,%lu,%d\n", millis(), ctrl_pwm); 
        vTaskDelay(2 / portTICK_PERIOD_MS); // run every 2 ms
    }
}

//controller task
//reads sensor, computes control, sets actuator command
static void controllerTask(void* pvParameters){

    while(1){    
        //load next note 
        //take mutex
        if (xSemaphoreTake(ctrl_Mutex, portMAX_DELAY)) {
            //play note if next note exists
            // set_pwm();
            //testing
            // ctrl_pwm = (int) (-1 * 255);

            //go to fully up position while waiting/moving
            ctrl_pwm = PWM_t::DOWN;
    
            //release mutex
            xSemaphoreGive(ctrl_Mutex);
            vTaskDelay(pdMS_TO_TICKS(10));

        }
        //go to fully up position while waiting/moving
        ctrl_pwm = PWM_t::UP;
        // wait 
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}


//public API
void init_controller(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;

    //initialize pins to motor driver
    pinMode(DIR_PIN, OUTPUT);
    // //initialize linear motion sensor and actuator
    init_sensor();


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

    //create actuator task
    xTaskCreatePinnedToCore(
        sendPwmTask,
        "sendPwmTask",
        4096, //stack size, 4kB (could be less??)
        NULL, //params
        1, //priority
        &actuatorTaskHandle, //task handle
        0 //core 0: 
    );

}





