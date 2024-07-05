

#include "motor_control.h"

#include <stdlib.h>

#include "main.h"
#include "can.h"
#include "sensors.h"
#include "pitch.h"

#define PITCH_UPDATE_DEG_THRESHOLD 0.25f

void DoPitchControl()
{
	if (sensor_data.feedback_pitch_mode == MOTOR_MODE_AUTOMATIC)
	{
		// float new_pitch_target = CalcPitchAuto();
		float new_pitch_target = 0;
		// new_pitch_target = 0.0f;
		float pitch_auto_target = new_pitch_target;
		// VerifyPitchCmd();

		// if (abs(new_pitch_target - pitch_auto_target) > PITCH_UPDATE_THRESHOLD)
		if (abs(new_pitch_target - sensor_data.pitch_angle) > PITCH_UPDATE_DEG_THRESHOLD)
		{
			float delta_angle_pales = CalcPitchAnglePales(TRUE) - new_pitch_target;

	#define MIN_ERROR_ANGLE 0.1f
			if (abs(delta_angle_pales) > 0.1f)
			{
				if (pitch_done)
				{
					// Small delay inbetween commands for smoothness
					static uint32_t pitch_done_counter = 0;
					++pitch_done_counter;

	#define PITCH_DONE_WAIT_NUM 6
					if (pitch_done_counter >= PITCH_DONE_WAIT_NUM)
					{
						pitch_done_counter = 0;
						SendPitchAngleCmd(new_pitch_target);
						// SendPitchTargetCmd(PITCH_ABSOLUTE_ZERO);
					}
				}

			}
		}
	}
}

void DoMastControl()
{
#define WIND_SPEED_MAST_THRESHOLD 1.0f
#define WIND_DIR_MAST_HYSTERESIS 10.0f // -10degs, +10degs
	if (sensor_data.wind_speed_avg >= WIND_SPEED_MAST_THRESHOLD)
	{
		if (abs(sensor_data.wind_direction_avg) >= WIND_DIR_MAST_HYSTERESIS)
		{
			uint8_t dir = MOTOR_DIRECTION_LEFT;
			if (sensor_data.wind_direction_avg < 0.0f)
				dir = MOTOR_DIRECTION_RIGHT;
			TransmitCAN(MARIO_MAST_MANUAL_CMD, (uint8_t*)&dir, 4, 0);
			pitch_done = 0;
			delay_us(100);
		}
		else
		{
			uint8_t dir = MOTOR_DIRECTION_STOP;
			TransmitCAN(MARIO_MAST_MANUAL_CMD, (uint8_t*)&dir, 4, 0);
		}
	}
}
