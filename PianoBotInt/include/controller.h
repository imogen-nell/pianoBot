#pragma once
#include <Arduino.h>



void init_controller(float kp, float ki, float kd);

void set_target(float new_target);

// void stop_controller();

//shared variable
// extern volatile float current_volt;
