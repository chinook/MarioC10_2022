/*
 * pitch.h
 *
 *  Created on: 13 juin 2023
 *      Author: Marc
 */

#ifndef _PITCH_H_
#define _PITCH_H_

#include "stm32f4xx_hal.h"

#define MAX_STEPS_PER_CMD 500
#define ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS ((float) 360 / 4096) //0.087890625
#define ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS_UPDATE_DEG_THRESHOLD ((float) 1 * ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS)
#define MAX_PITCH_VALUE_RAW 4095
#define HALF_MAX_PITCH_VALUE_RAW 2047
#define MAX_UP_PITCH_VALUE_DEGREE ((float) HALF_MAX_PITCH_VALUE_RAW * 0.75f)
#define MAX_DOWN_PITCH_VALUE_DEGREE ((float) HALF_MAX_PITCH_VALUE_RAW * -0.75f)

extern uint8_t warning_pitch_angle_close_to_up;
extern uint8_t warning_pitch_angle_close_to_down;

float CalcPitchAngle_raw_to_deg();

float CalcTSR();
float CalcPitchAuto();

void VerifyPitchTargetCmd(uint32_t target_pitch_abs);

void VerifyRopsCmd();
void SendPitchTargetCmd(uint32_t target_pitch_abs);
void SendPitchROPSCmd();
void SendPitchAngleCmd(float target_pitch);

// void SendPitchROPSCmd();


#endif /* _PITCH_H_ */
