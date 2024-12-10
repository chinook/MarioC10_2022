

#include "motor_control.h"

#include <stdlib.h>

#include "main.h"
#include "can.h"
#include "sensors.h"
#include "pitch.h"

float func_moy_wind_direction();

#define PITCH_UPDATE_DEG_THRESHOLD 0.25f


void DoPitchControl()
{
	if (sensor_data.feedback_pitch_mode == MOTOR_MODE_AUTOMATIC)
	{
		float pitch_auto_target = CalcPitchAuto();

		if (abs(pitch_auto_target - sensor_data.pitch_angle) > PITCH_UPDATE_DEG_THRESHOLD)
		{
			float delta_angle_pales = CalcPitchAnglePales(TRUE) - pitch_auto_target;

	#define MIN_ERROR_ANGLE 0.1f
			if (abs(delta_angle_pales) > 0.1f)
			{
				if (pitch_done)
				{
					SendPitchAngleCmd(pitch_auto_target);
				}

			}
		}
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
