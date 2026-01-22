#pragma once
#include <Arduino.h>
//controller interface

//** Initialize the PID controller with given gains */
void init_controller(float kp, float ki, float kd);

//** Set the target position for the controller */
void set_target(float new_target);


//read by actuator, set by controller
extern volatile int ctrl_pwm; //declare cariable defined elsewhere 
//set by sensor, read by controller
extern volatile float current_position;

