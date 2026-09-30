/*
 * motor_control.c
 *
 *  Created on: Aug 17, 2025
 *      Author: thoma
 */

#include "motor_control.h"

#include <math.h>
#include <stdlib.h>

#include "main.h"
#include "can.h"
#include "sensors.h"
#include "pitch.h"

float func_moy_wind_direction();

#define PITCH_UPDATE_DEG_THRESHOLD 0.25f
#define MIN_ERROR_ANGLE 0.1f

uint8_t check_pitch_warning() {
	if (warning_pitch_angle_close_to_up == 1) {
		return MOTOR_DIRECTION_LEFT;
	} else if (warning_pitch_angle_close_to_down == 1) {
		return MOTOR_DIRECTION_RIGHT;
	} else {
		return 100;
	}
}

float pitch_auto_target = 0;
void DoPitchControl() {
	uint8_t direction = MOTOR_DIRECTION_STOP;
	uint8_t speed = 0;

	if (rops_status == 1) {
		pitch_auto_target = -90;
	} else {
		CalcPitchAuto();
		pitch_auto_target = sensor_data.pitch_auto_target;
		//pitch_auto_target += status_button_bg - status_button_bgg;
	}

	float delta_angle_pales = sensor_data.pitch_angle - pitch_auto_target;

	float delta_angle_pales_temp = delta_angle_pales *100;
	float ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS_UPDATE_DEG_THRESHOLD_temps = ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS_UPDATE_DEG_THRESHOLD *200;

	if (fabsf(delta_angle_pales_temp) > ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS_UPDATE_DEG_THRESHOLD_temps) { //pitch angle far from cmd
		if (delta_angle_pales > 0) {
			direction = MOTOR_DIRECTION_LEFT;
		} else {
			direction = MOTOR_DIRECTION_RIGHT;
		}

		if ((direction == MOTOR_DIRECTION_LEFT)
				&& (warning_pitch_angle_close_to_down == 1)) {
			direction = MOTOR_DIRECTION_STOP;
		} else if ((direction == MOTOR_DIRECTION_RIGHT)
				&& (warning_pitch_angle_close_to_up == 1)) {
			direction = MOTOR_DIRECTION_STOP;
		}

		//0 à 100% -> plus on est loin de pitch_auto_target, plus la vitesse est rapide
		//if (abs(delta_angle_pales) >= 5) {
			speed = 100;
		//} else {
			//speed = (uint8_t) (abs(delta_angle_pales) / 5 * 100);
		//}

	} else { //cmd is same as pitch angle
		direction = MOTOR_DIRECTION_STOP;
		speed = 0;
	}
	TransmitCAN(CAN_ID_CMD_MARIO_PITCH_DIRECTION, &direction, 4, 1);
	TransmitCAN(CAN_ID_CMD_MARIO_PITCH_SPEED, &speed, 4, 1);
}


#define WIND_SPEED_MAST_THRESHOLD 1.0f
#define WIND_DIR_MAST_HYSTERESIS 10.0f // -10degs, +10degs

void DoMastControl() {

	if (0) {
		//sensor_data.limit1 == 0 || sensor_data.limit2 == 0
		//return;
	}
	if (sensor_data.wind_speed_avg >= WIND_SPEED_MAST_THRESHOLD) {
		if (fabsf(sensor_data.wind_direction_avg) >= WIND_DIR_MAST_HYSTERESIS) {
			if (sensor_data.wind_direction_avg > 0.0f) {
				MotorMastSpeedDir(-100);
			} else {
				MotorMastSpeedDir(100);
			}
		}
	} else {
		MotorMastSpeedDir(0);
	}

}

