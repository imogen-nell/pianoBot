#pragma once
#include <Arduino.h>
//hall interface 

//** Initialize the hall sensor */
void init_sensor( );

//read by controller, set by sensor 
extern volatile float current_position;