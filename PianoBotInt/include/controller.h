#pragma once
#include <Arduino.h>



void init_controller(float kp, float ki, float kd, int loop_delay_ms);

void set_target(int new_target);

// void stop_controller();

//shared variable
extern volatile float current_volt;