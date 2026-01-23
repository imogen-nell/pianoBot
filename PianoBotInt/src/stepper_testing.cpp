#include <Arduino.h>
#include <AccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stepper_tests.h"

// Define motor interface type (1 = driver with STEP/DIR pins)
AccelStepper stepper(1, 25, 26);

void back_and_forth_test(int DIR_PIN, int STEP_PIN){
    Serial.println("------------------back and for test ------------------");
    bool dirr = true;
    while(1){   
        digitalWrite(DIR_PIN, dirr); //false = right

        for(int i =0; i<250; i++){
            digitalWrite(STEP_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(2));    
            digitalWrite(STEP_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(2));    
        }
        dirr = !dirr; //reverse direction
        vTaskDelay(pdMS_TO_TICKS(70));
    };
}

void stepper_trap(double vmax, double acc){
    const double r = 0.0175; // pulley radius [m]
    double spm = 200/(2*PI*r); // steps per meter (whole step)
    double dist = 2*PI*r*2; // total distance travel = 2rev

    int vstep = (int) (vmax*spm); // max velocity in steps per sec
    int acc_step = (int) (acc*spm); // acc in steps per sec^2
    int dist_step = (int) (dist*spm); // distance travel in steps

    stepper.setMaxSpeed(vstep);
    stepper.setAcceleration(acc_step);
    // stepper.setMinPulseWidth(40); //

    stepper.moveTo(400); // move 
}
void stepper_debug() {
    Serial.print("Current pos: ");
    Serial.println(stepper.currentPosition());

    Serial.print("Target pos: ");
    Serial.println(stepper.targetPosition());

    Serial.print("Distance to go: ");
    Serial.println(stepper.distanceToGo());

    Serial.print("Max speed: ");
    Serial.println(stepper.maxSpeed());

    Serial.print("Acceleration: ");
    Serial.println(stepper.acceleration());
}

void stepper_run() {
    stepper.run();
    Serial.println("----moved----");
}


void max_speed_test(){
    Serial.println("-----Max speed testing----");
    double v = 0.09163*6.0; //50 rpm
    const double acc = 0.05;
    const double inc = 0.09163; // increase by 50rpm 
    stepper_trap(v, acc);
    // while(v <= 1.09956){ // <=600rpm
    //     stepper_trap(v, acc);
    //     v += inc;
    // }
}

void missed_step_test(int DIR_PIN, int STEP_PIN){
    // proceed for 1000 steps at various speeds
    Serial.println("-----Missed Step Testing ----");


}