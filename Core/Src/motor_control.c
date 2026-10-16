/*
 * motor_control.c
 *
 *  Created on: Aug 17, 2025
 *      Author: thoma
 *
 *  Edited on : Oct 1, 2026
 *  	Editor: Simon B.
 */

#include "motor_control.h"

#include <math.h>
#include <stdlib.h>

#include "main.h"
#include "can.h"
#include "sensors.h"
#include "pitch.h"

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

#define MAST_DEG_PER_SEC     1.0f   // À MESURER (deg/s du mât à vitesse constante, négatif si le signe est inversé)
#define MAST_LIMIT_FREE_MS   200    // délai sans métal avant de libérer la direction bloquée

float mast_angle = 0.0f;
uint8_t mast_calibrated = 0;           // 0 à chaque démarrage, jamais sauvegardé
static int8_t mast_applied_dir = 0;    // direction réellement appliquée au moteur (-1, 0, +1)
static int8_t mast_last_dir = 0;       // dernière direction non nulle appliquée
static int8_t mast_blocked_dir = 0;    // direction interdite par le limit switch (0 = aucune)
static uint32_t mast_last_tick = 0;
static uint32_t mast_limit_seen_tick = 0;

// Limit switch : 1 = rien, 0 = métal détecté
static uint8_t MastLimitActive(void) {
	return HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin) == GPIO_PIN_RESET;
}

// Remplace tous les appels directs à MotorMastSpeedDir()
void MastCommand(int8_t speed_dir) {
	int8_t dir = (speed_dir > 0) - (speed_dir < 0);
	if (dir != 0 && dir == mast_blocked_dir) {
		speed_dir = 0;
		dir = 0;
	}
	mast_applied_dir = dir;
	if (dir != 0) mast_last_dir = dir;
	MotorMastSpeedDir(speed_dir);
}

// À appeler à chaque tour de la state machine
void UpdateMastAngle(void) {
	uint32_t now = HAL_GetTick();
	float dt = (now - mast_last_tick) / 1000.0f;
	mast_last_tick = now;

	// --- Limit switch ---
	if (MastLimitActive()) {
		mast_limit_seen_tick = now;
		if (mast_blocked_dir == 0) {
			mast_blocked_dir = mast_last_dir;   // on mémorise d'où on vient
		}
		if (mast_applied_dir != 0 && mast_applied_dir == mast_blocked_dir) {
			mast_applied_dir = 0;
			MotorMastSpeedDir(0);               // stop, mais l'autre direction reste permise
		}
	} else if (mast_blocked_dir != 0 && (now - mast_limit_seen_tick) >= MAST_LIMIT_FREE_MS) {
		mast_blocked_dir = 0;                   // le switch ne voit plus de métal
	}

	// --- Calibration avec le bouton BG (premier appui seulement) ---
	if (!mast_calibrated) {
		if (status_button_bg == 1) {
			mast_angle = 0.0f;
			mast_calibrated = 1;
		}
		return;
	}

	// --- Calcul de l'angle par le temps ---
	mast_angle += mast_applied_dir * MAST_DEG_PER_SEC * dt;
}

void DoMastControl() {

	if (sensor_data.wind_speed_avg >= WIND_SPEED_MAST_THRESHOLD) {
		if (fabsf(sensor_data.wind_direction_avg) >= WIND_DIR_MAST_HYSTERESIS) {
			if (sensor_data.wind_direction_avg > 0.0f) {
				MastCommand(-100);
			} else {
				MastCommand(100);
			}
		}
	} else {
		MastCommand(0);
	}

}

