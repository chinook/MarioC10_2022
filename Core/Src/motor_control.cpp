

#include "motor_control.h"

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
void DoPitchControl()
{
	if (sensor_data.feedback_pitch_mode == MOTOR_MODE_AUTOMATIC) {
		uint8_t direction = MOTOR_DIRECTION_STOP;
		uint8_t speed = 0;

		if (rops_status == 1) {
			pitch_auto_target = -90;
		} else {
			pitch_auto_target = CalcPitchAuto();
			//pitch_auto_target += status_button_bg - status_button_bgg;
		}



		float delta_angle_pales = sensor_data.pitch_angle - pitch_auto_target;


		if (abs(delta_angle_pales) > ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS_UPDATE_DEG_THRESHOLD) //pitch angle far from cmd
		{
			if (delta_angle_pales > 0) {
				direction = MOTOR_DIRECTION_LEFT;
			} else {
				direction = MOTOR_DIRECTION_RIGHT;
			}

			//0 à 100% -> plus on est loin de pitch_auto_target, plus la vitesse est rapide
			if (abs(delta_angle_pales) >= 5) {
				speed = 100;
			} else { //entre 0 et 5 degrées d'écart
				speed = (uint8_t) ((abs(delta_angle_pales) / 5) * 100);
			}

			if ((direction == MOTOR_DIRECTION_LEFT) && (warning_pitch_angle_close_to_down == 1)) {
				direction = MOTOR_DIRECTION_STOP;
				speed = 0;
			} else if ((direction == MOTOR_DIRECTION_RIGHT) && (warning_pitch_angle_close_to_up == 1)) {
				direction = MOTOR_DIRECTION_STOP;
				speed = 0;
			}
		}
		else { //cmd is same as pitch angle
			direction = MOTOR_DIRECTION_STOP;
			speed = 0;
		}
		motor_direction_pitch = direction;
		motor_speed_pitch = speed;
		TransmitCAN(CAN_ID_CMD_MARIO_PITCH_DIRECTION, &direction, 4, 1);
		TransmitCAN(CAN_ID_CMD_MARIO_PITCH_SPEED, &speed, 4, 1);
	}
}



void DoMastControl()
{
#define WIND_SPEED_MAST_THRESHOLD 1.0f
#define WIND_DIR_MAST_HYSTERESIS 10.0f // -10degs, +10degs

	uint32_t dir_left = 0x200;
	uint32_t dir_right = 0x300;
	uint32_t dir_stop = 0x100;

	if (0) { //sensor_data.limit1 == 0 || sensor_data.limit2 == 0
		//TransmitCAN(MARIO_MAST_MANUAL_CMD, (uint8_t*)&dir_stop, 4, 1);
		//return;
	}
	if (sensor_data.wind_speed_avg >= WIND_SPEED_MAST_THRESHOLD) {
		if (abs(sensor_data.wind_direction_avg) >= WIND_DIR_MAST_HYSTERESIS) {
			if (sensor_data.wind_direction_avg > 0.0f) {
				TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, (uint8_t*)&dir_left, 4, 1);
			}
			else {
				TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, (uint8_t*)&dir_right, 4, 1);
			}
		}
	}
	else {
		TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, (uint8_t*)&dir_stop, 4, 1);
	}

}
