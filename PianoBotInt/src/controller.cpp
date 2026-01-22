#include "controller.h"
#include "position_sensor.h"
// #include "position_actuator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "global.h"
#include "driver/ledc.h"
#include "driver/adc.h"

//hall sensor voltage at respective positions
const float PRESSED = 1.81f;
const float RELEASED = 1.08f;

//pid vals
static float Kp = 0.0f;
static float Ki = 0.0f;
static float Kd = 0.0f;

static float target_pos = 0.0f;
static float previous_error = 0.0f;
static float integral = 0.0f; 
static const float dt = 0.002f; //loop time in seconds
// static int ctrl_pwm = 0; //should this be volatile ?


//task config
static TaskHandle_t controllerTaskHandle = nullptr;
static TaskHandle_t actuatorTaskHandle = nullptr;

//init shared variables
volatile float current_position = 0.0f;


//esp32 i/o pins for voice coil driver
#define PWM_PIN 5
#define DIR_PIN 2
#define PWM_CHANNEL 0
#define PWM_FREQ 20000 //20kHz
#define PWM_RES 8 //duty 0 to 255

//PWM control values for finger positions
enum PWM_t {
    UP = -255,
    DOWN = 255,
    REST = 0
};

// static void get_next_note(void){
//     //get next note from global notes array
//     // int next_note[] = ;
// }

//sends pwm signal to voice coil
//args: ctrl_pwm , between -255 to 255
void send_pwm(int ctrl_pwm){
    
    digitalWrite(DIR_PIN, ctrl_pwm <= 0); 
    
    uint8_t duty = (uint8_t) abs(ctrl_pwm);
    //uint32_t duty = map(abs(ctrl_pwm), 0, 255, 0, 4095); if need 12 bit res 
    ledcWrite(PWM_CHANNEL, duty);

    //for data logger
    // Serial.printf("PID,%lu,%d\n", millis(), ctrl_pwm); 
}

//**reads target voltage for hall sensor
//**compute pwm needed with PID //
//** sends correct pwm to motor 
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
    // ctrl_pwm = (int) (output * 255);
    send_pwm((int)( output * 255));
}

//sets target voltage (position) for controller
void set_target(float target_PWM) {
    //convert pwm to voltage 
    float m = (PRESSED-RELEASED)/255.0f;
    target_pos = target_PWM * m + RELEASED ; 
    Serial.printf("CTRL,%lu,%.4f\n", millis(),target_pos);

}


//controller task that plays notes 
static void controllerTask(void* pvParameters){

    while(1){    
        //load next note 
        
        //wait until finger is in position
        if (xSemaphoreTake(ctrl_Mutex, portMAX_DELAY)) { //waits indefinetly until mutex is avilable
            Serial.println("-------------- play key ---------------");

            //play note from notes
            while(* notes != 0){
                send_pwm(* notes);
                vTaskDelay(pdMS_TO_TICKS(10)); //hold note for 10ms
                notes++;    
            }
            notes++; //skip the 0 delimiter
            // Serial.printf("current note: %d\n", *notes);
            

            // //for testing /////
            // // send_pwm(PWM_t::DOWN);
            // send_pwm(-128);
            // vTaskDelay(pdMS_TO_TICKS(1500));
            //////////////////
                        
            //go to fully up position before releasing mutex to stepper
            send_pwm(PWM_t::UP);
            //wait until finger is fully up
            vTaskDelay(pdMS_TO_TICKS(100));

            //release mutex
            xSemaphoreGive(ctrl_Mutex);
            vTaskDelay(pdMS_TO_TICKS(10)); // Delay for stepper to get mutex
        }
    }
    
}

//initializes controller with pid values and begins key playing task
void init_controller(float kp, float ki, float kd){
    Kp = kp;
    Ki = ki;
    Kd = kd;

    //initialize pins to motor driver
    pinMode(DIR_PIN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);



    //initialize linear position sensor
    init_sensor();

    //create controller task, note playing
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





