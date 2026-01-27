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

void stepper_trap(double vmax, double acc, double dist){
    const double r = 0.0175; // pulley radius [m]
    const double factor = 8.0; // microstep setting (1/8)
    double spm = (200.0*factor)/(2.0*PI*r); // steps per meter

    int vstep = (int) (vmax*spm); // max velocity in steps per sec
    int acc_step = (int) (acc*spm); // acc in steps per sec^2
    int dist_step = (int) (dist*spm); // distance travel in steps

    stepper.setMaxSpeed(vstep);
    stepper.setAcceleration(acc_step);
    // stepper.setMinPulseWidth(40); // oscilloscope 

    stepper.move(dist_step); // move relative position
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
}

void max_acc_test(){
    Serial.println("-----Max acceleration testing----");

    double v = 0.09163*7; //400 rpm

    const double r = 0.0175; // pulley radius [m]
    const double rev = 2; //half rev
    double dist = 2.0*PI*r*rev;

    double acc = 3.0;
    const double inc = 0.5; // increase by .5m/s^2 

    while(acc <= 20.0){
        stepper_trap(v, acc, dist);

        Serial.print("Acceleration: ");
        Serial.println(acc);

        // BLOCKING WAIT: This makes the code wait until the move is finished
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        acc += inc;
        dist = -dist;
        delay(1000);
    }
}

void max_speed_test(){
    Serial.println("-----Max speed testing----");

    double v = 0.09163; //50 rpm
    const double inc = 0.09163; // increase by 50rpm 

    const double r = 0.0175; // pulley radius [m]
    const double rev = 2; //half rev
    double dist = 2.0*PI*r*rev;

    double acc = 5.0;

    while(v <= 0.09163*12.0){ // <=600rpm
        stepper_trap(v, acc, dist);

        Serial.print("Speed: ");
        Serial.println(v);

        // BLOCKING WAIT: This makes the code wait until the move is finished
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        v += inc;
        dist = -dist;
        delay(1000);
    }
}


void missed_step_test(int DIR_PIN, int STEP_PIN){
    // proceed for 1000 steps at various speeds
    Serial.println("-----Missed Step Testing ----");


}