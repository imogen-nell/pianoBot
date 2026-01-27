#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

void back_and_forth_test(int DIR_PIN, int STEP_PIN);
extern AccelStepper stepper;
void max_speed_test();
void missed_step_test(int DIR_PIN, int STEP_PIN);
void stepper_trap(double vmax, double dist);
void stepper_debug();
void stepper_run();
