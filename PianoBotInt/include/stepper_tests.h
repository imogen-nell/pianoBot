#pragma once
#include <Arduino.h>
#include<AccelStepper.h>


void back_and_forth_test(int DIR_PIN, int STEP_PIN);
extern AccelStepper stepper;
void max_speed_test(int DIR_PIN, int STEP_PIN);
void missed_step_test(int DIR_PIN, int STEP_PIN);