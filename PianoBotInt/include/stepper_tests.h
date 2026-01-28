#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

void back_and_forth_test(int DIR_PIN, int STEP_PIN);
extern AccelStepper stepper;
void max_speed_test();
void max_acc_test();
void missed_step_test();
void stepper_trap(double vmax, double acc, double dist);
void stepper_debug();
void stepper_run();
