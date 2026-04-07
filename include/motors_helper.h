#pragma once
#include <Arduino.h>
#include "config.h"

void encoders_init();
void motors_init();
void set_motors_rpm(float left_rpm, float right_rpm);  // Set target RPM for both motors
void get_motors_rpm(float *left_rpm, float *right_rpm); // Get current RPM of both motors
void get_motors_pos(float *left_pos, float *right_pos); // Get current position of both motors (rad)
void motors_update();