#include "vc_controller.h"
#include "position_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "global.h"
#include "driver/ledc.h"
// #include "driver/adc.h"


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
TaskHandle_t vcTaskHandle = nullptr;

//init shared variables
volatile float current_position = 0.0f;


//esp32 i/o pins for voice coil driver
#define PWM_PIN 5
#define DIR_PIN 2
#define PWM_CHANNEL 0
#define PWM_FREQ 20000 //20kHz
#define PWM_RES 8 //duty 0 to 255

//one shot timers  
static TimerHandle_t voice_coil_timer;//10 ms
static TimerHandle_t finger_up_timer; //100 ms
#define NOTE_DONE     (1 << 0) //timer flag
#define FINGER_UP_DONE (1 << 1)


//PWM control values for finger positions
enum PWM_t {
    UP = -200,
    DOWN = 200,
    REST = 0
};

//sends pwm signal to voice coil
//args: ctrl_pwm , between -255 to 255
void send_pwm(int ctrl_pwm){
    
    digitalWrite(DIR_PIN, ctrl_pwm <= 0); 
    ledcWrite(PWM_CHANNEL,  abs(ctrl_pwm));

    //for data logger
    // Serial.printf("PID,%lu,%d\n", millis(), ctrl_pwm); 
}

//timer callback(mandatory)
static void note_timer_cb(TimerHandle_t xTimer)
{
    // Notify finger task that motion is complete
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //set correct flag
    xTaskNotifyFromISR(vcTaskHandle, NOTE_DONE, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void finger_up_cb(TimerHandle_t xTimer)
{
    BaseType_t hpw = pdFALSE;
    //sets flag
    xTaskNotifyFromISR(vcTaskHandle, FINGER_UP_DONE, eSetBits, &hpw);
    portYIELD_FROM_ISR(hpw);
}

//**reads target voltage for hall sensor
//**compute pwm needed with PID //
//** sends correct pwm to motor 
// static void set_pwm(void){
//     float error =  target_pos-current_position;
//     integral +=   error * dt; //integral intime = loop delay 2ms
//     float derivative = (error - previous_error) / dt;
//     previous_error = error;

//     //clamp integral
//     integral = ((integral)<(-0.5f)?(-0.5f):((integral)>(0.5f)?(0.5f):(integral)));

//     float output = Kp * error + Ki * integral + Kd * derivative;    

//     //clamping
//     output = ((output)<(-1.0f)?(-1.0f):((output)>(1.0f)?(1.0f):(output)));
//     // ctrl_pwm = (int) (output * 255);
//     send_pwm((int)( output * 255));
// }

//sets target voltage (position) for controller
// void set_target(float target_PWM) {
//     //convert pwm to voltage 
//     float m = (PRESSED-RELEASED)/255.0f;
//     target_pos = target_PWM * m + RELEASED ; 
//     Serial.printf("CTRL,%lu,%.4f\n", millis(),target_pos);

// }

//initializes finger pwm off timer
void finger_timer_init(void)
{
    voice_coil_timer = xTimerCreate(
        "note_timer",
        pdMS_TO_TICKS(10),
        pdFALSE,        // one-shot
        NULL,
        note_timer_cb //notifies done
    );
}

void finger_up_timer_init(void)
{
    finger_up_timer = xTimerCreate(
        "finger_up_timer",
        pdMS_TO_TICKS(500),
        pdFALSE,        // one-shot
        NULL,
        finger_up_cb //notifies done
    );
}

//controller task that plays notes 
static void controllerTask(void* pvParameters){

    while(1){    
        // Wait for command from main controller/coordinator
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //for testing
        Serial.printf("-------------- PLAY key at: %d\n", next_note_ptr- start_addr);

        //play all notes at current key position (without moving stepper) until delimiter 0 is hit
        while(* next_note_ptr != 0){
            //send to voice coil
            send_pwm(* next_note_ptr);

            //hold each input for 10ms
            xTimerStop(voice_coil_timer, 0);     // reset timer if already running
            xTimerStart(voice_coil_timer, 0);    // will lift finger after 10 ms
            
            // Wait until the 10 ms timer 
            xTaskNotifyWait(0, NOTE_DONE, NULL, portMAX_DELAY);

            next_note_ptr++;
        }
        //lift finger;
        send_pwm(PWM_t::UP);
        //give 100ms to go up
        xTimerStop(finger_up_timer, 0);     
        xTimerStart(finger_up_timer, 0);
        xTaskNotifyWait(0, FINGER_UP_DONE, NULL, portMAX_DELAY);

        next_note_ptr++; //skip the 0 delimiter
        //wrap around song
        if(next_note_ptr >= end_addr){
            Serial.println("Resetting song");
            next_note_ptr = start_addr; //reset notes to start of array
        }
                

        // Notify coordinator that finger is finished
        xTaskNotifyGive(coordinatorTaskHandle);
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

    //10sm timer, - note array inputs are for 10ms each
    finger_timer_init();
    finger_up_timer_init();
    //initialize linear position sensor
    init_sensor();

    //create controller task, note playing
    xTaskCreatePinnedToCore(
        controllerTask,
        "controllerTask",
        4096, //stack size
        nullptr, //params
        1, //higher priority
        &vcTaskHandle, //task handle
        1 //core
    );

}





