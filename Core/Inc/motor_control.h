/*
 * motor_control.h
 *
 *  Created on: 2 juill. 2023
 *      Author: Marc
 */

#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "stm32f4xx_hal.h" //uint8_t

uint8_t check_pitch_warning();
void DoPitchControl();
void DoMastControl();
void DriveMotorMast();

extern float pitch_auto_target;

#endif /* _MOTOR_CONTROL_H_ */
