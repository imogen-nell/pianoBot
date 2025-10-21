#pragma once
#include <Arduino.h>
//voice coil interface 


//TODO  add func defn 
void init_actuator();

void set_pwm(int pwm_value);

void stop_actuator();
