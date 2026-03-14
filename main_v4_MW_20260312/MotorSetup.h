#ifndef MotorSetup_H
#define MotorSetup_H

#include <Arduino.h>

void setup_motor();
void motor_move_to(int position);
int motor_get_position();
float motor_steps_per_rev();
int motor_deadband();

#endif