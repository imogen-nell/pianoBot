#include "vc_controller.h"

// Constructor
VoiceCoilController::VoiceCoilController(uint8_t pwm_pin, uint8_t dir_pin, uint8_t pwm_channel,
                                         float kp, float ki, float kd,
                                        int* start, int notes_arr_len)
    : PWM_PIN(pwm_pin), DIR_PIN(dir_pin), PWM_CHANNEL(pwm_channel),
      Kp(kp), Ki(ki), Kd(kd), 
      next_note_ptr(start), end_addr(start+notes_arr_len),start_addr(start)
{    
    //initialize pins to motor driver
    pinMode(DIR_PIN, OUTPUT);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);

    init_timers();

    //initialize linear position sensor
    init_sensor();

    //create controller task, note playing
    xTaskCreatePinnedToCore(
        &VoiceCoilController::controllerTaskEntry,
        "VoiceCoilTask",
        4096, //stack size
        this, //params
        1, //lower priority
        &vcTaskHandle, //task handle
        1 //core
    );

}

void VoiceCoilController::setCoordinatorHandle(TaskHandle_t handle) {
    this->coordinatorTaskHandle = handle;
}

//private methods 

//initializes finger pwm off timer
void VoiceCoilController::init_timers() {
    voice_coil_timer = xTimerCreate(
        "note_timer",
        pdMS_TO_TICKS(10),
        pdFALSE,        // one-shot
        this,
        note_timer_cb //notifies done
    );

    finger_up_timer = xTimerCreate(
        "finger_up_timer",
        pdMS_TO_TICKS(100),
        pdFALSE,        // one-shot
        this,
        finger_up_cb //notifies done
    );
}


//sends pwm signal to voice coil
//args: ctrl_pwm , between -255 to 255
void VoiceCoilController::send_pwm(int ctrl_pwm){
    
    digitalWrite(DIR_PIN, ctrl_pwm <= 0); 
    ledcWrite(PWM_CHANNEL,  abs(ctrl_pwm));

    //for data logger
    // Serial.printf("PID,%lu,%d\n", millis(), ctrl_pwm); 
}

//timer callback(mandatory)
void VoiceCoilController::note_timer_cb(TimerHandle_t xTimer)
{
    auto self = static_cast<VoiceCoilController*>(pvTimerGetTimerID(xTimer));
    // Notify finger task that motion is complete
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //set correct flag
    xTaskNotifyFromISR(self->vcTaskHandle, NOTE_DONE, eSetBits, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR VoiceCoilController::finger_up_cb(TimerHandle_t xTimer) {
    // get the VoiceCoilController instance
    auto self = static_cast<VoiceCoilController*>(pvTimerGetTimerID(xTimer));

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //sets flag
    xTaskNotifyFromISR(self->vcTaskHandle, FINGER_UP_DONE, eSetBits, &xHigherPriorityTaskWoken);
    xTaskNotifyFromISR(self->coordinatorTaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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


// task entry wrapper 
void VoiceCoilController::controllerTaskEntry(void* pvParameters) {
    static_cast<VoiceCoilController*>(pvParameters)->controllerTask();
}

//controller task that plays notes 
void VoiceCoilController::controllerTask() {

    while(1){    
        // Wait for command from main controller/coordinator
        if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0){
            continue;
        }

        //for testing
        // Serial.printf("-------------- PLAY key at: %d, %d\n", next_note_ptr- start_addr,*next_note_ptr );

        //play all notes at current key position (without moving stepper) until delimiter -5 is hit
        while(*next_note_ptr != 0){
            //send to voice coil
            send_pwm(*next_note_ptr);

            //hold each input for 10ms
            xTimerStop(voice_coil_timer, 0);     // reset timer if already running
            xTimerStart(voice_coil_timer, 0);    // will lift finger after 10 ms
            next_note_ptr++;
            // Wait until the 10 ms timer 
            uint32_t notifyValue =0;
            xTaskNotifyWait(0, NOTE_DONE, &notifyValue, portMAX_DELAY);
            // Make sure the timer actually fired
            // while ((notifyValue & NOTE_DONE) == 0) {
            //     xTaskNotifyWait(0, NOTE_DONE, &notifyValue, portMAX_DELAY);
            // }
            
        }
        //lift finger;
        send_pwm(PWM_t::UP);
        //give 100ms to go up
        xTimerStop(finger_up_timer, 0);     
        xTimerStart(finger_up_timer, 0);
        xTaskNotifyWait(0, FINGER_UP_DONE, NULL, portMAX_DELAY);

        next_note_ptr++; //skip the -5 delimiter
        //wrap around song
        if(next_note_ptr >= end_addr){
            // Serial.println("Resetting song");
            next_note_ptr = start_addr; //reset notes to start of array
        }
            
    }
    
}









