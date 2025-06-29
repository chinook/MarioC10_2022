

#include "pitch.h"
#include "main.h"

#include <stdio.h>
#include <cstdlib>

#include "chinook_can_ids.h"


#define KNOTS_TO_MS 0.514444f

//pour 12 bits



//pour 12 bits, c'est des valeurs de 0 à 4095
//doit être recalibrer À CHAQUE FOIS que le spider ou l'encodeur est démonté, sinon le 0 n'est plus bon
#define PITCH_ABSOLUTE_ZERO 1460
#define ENCODER_TO_PALES_RATIO (360.0f / 245.0f) // À VÉRIFIER pas mal bon environ 0,65 tour d'encodeur pour 1 tour de pale
#define PALE_RADIUS 0.879f


static float log_pitch[200] = {0};
static bool startup_filter_pitch_angle = 1;

bool filter_pitch_angle(float pitch_angle);
void log_pitch_angle(float pitch_angle);

float BoundAngleSemiCircle(float angle);
float CalcDeltaPitch(uint32_t reference_point);



float BoundAngleSemiCircle(float angle)
{
	if (angle < -180.0f)
		return angle + 360.0f;
	else if (angle > 180.0f)
		return angle - 360.0f;
	else
		return angle;
}

uint8_t warning_pitch_angle_close_to_up = 0;
uint8_t warning_pitch_angle_close_to_down = 0;

float CalcPitchAngle_raw_to_deg()
{
	//explications de la conversion des valeurs brutes de l'encodeur absolue vers un angle en degrée
	//0 360 milieu 180 -> -180 180 milieu 0
	//1 tour d'encodeur (360) égale 270 degrée pour un tour de pale : rotor C12 avant PFE H2025
	//45 315 milieu 180 -> -135 135 milieu 0
	//non linéaire avec au milieu pas de différence mais aux bornes 45 degrées de différence
	//
	//il faut placer l'angle des pales à 0, puis déterminer enregister cette valeur dans le code (changer le define PITCH_ABSOLUTE_ZERO)
	//si le milieu est 0, alors il est possible de multiplier l'angle par 0.75 (270 / 360) pour obtenir un décalage de 45 degrées au bornes,
																		//soit 180*0.75=135, on a bien un décalage de 45 degrées
	//en partant d'une valeur brute d'encodeur de 12bits, soit des valeurs de 0 à 4096,
	//il faut donc ne JAMAIS dépasser le tour complet, au risque de perdre le 90 degrée de décalage (ou plus selon le nombre de tour) si le microcontrolleur redémarre.
	//ARRET des moteurs si proche de +/-2048 par rapport au define PITCH_ABSOLUTE_ZERO
	//

	float pitch_encoder_centered = 0;
	if (PITCH_ABSOLUTE_ZERO <= HALF_MAX_PITCH_VALUE_RAW) { //0 à 2047
		if (sensor_data.pitch_encoder < (MAX_PITCH_VALUE_RAW - (HALF_MAX_PITCH_VALUE_RAW - PITCH_ABSOLUTE_ZERO))) {
			pitch_encoder_centered = (float) ((float) sensor_data.pitch_encoder + (float) HALF_MAX_PITCH_VALUE_RAW - (float) PITCH_ABSOLUTE_ZERO);
		} else {
			pitch_encoder_centered = (float) ((float) sensor_data.pitch_encoder + (float) HALF_MAX_PITCH_VALUE_RAW - (float) PITCH_ABSOLUTE_ZERO - (float) MAX_PITCH_VALUE_RAW);
		}
	} else if (PITCH_ABSOLUTE_ZERO > HALF_MAX_PITCH_VALUE_RAW) { //2048 à 4097
		if (sensor_data.pitch_encoder > (float) (PITCH_ABSOLUTE_ZERO - HALF_MAX_PITCH_VALUE_RAW)) {
			pitch_encoder_centered = (float) ((float) sensor_data.pitch_encoder - (float) ((float) PITCH_ABSOLUTE_ZERO - (float) HALF_MAX_PITCH_VALUE_RAW));
		} else {
			pitch_encoder_centered = (float) ((float) sensor_data.pitch_encoder + (float) MAX_PITCH_VALUE_RAW - (float) ((float) PITCH_ABSOLUTE_ZERO - (float) HALF_MAX_PITCH_VALUE_RAW));
		}
	}

	pitch_encoder_centered -= (float) HALF_MAX_PITCH_VALUE_RAW;

	float pitch_angle = (float) pitch_encoder_centered * (float) ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS; //si 2047 = 180
	pitch_angle = pitch_angle * (float) ENCODER_TO_PALES_RATIO;

	//warning bornes
	if (pitch_encoder_centered > MAX_UP_PITCH_VALUE_DEGREE) {
		warning_pitch_angle_close_to_up = 1;
	} else {
		warning_pitch_angle_close_to_up = 0;
	}

	if (pitch_encoder_centered < MAX_DOWN_PITCH_VALUE_DEGREE) {
		warning_pitch_angle_close_to_down = 1;
	} else {
		warning_pitch_angle_close_to_down = 0;
	}

	return pitch_angle;
}

bool filter_pitch_angle(float pitch_angle) {
	if (pitch_angle == 0) {
		return false;
	}
	float moy_pitch_angle = 0;
	for (int i = 0; i>10; i++) {
		moy_pitch_angle += log_pitch[i];
	}
	moy_pitch_angle /= 10;
	if ((pitch_angle > moy_pitch_angle - 5) && (pitch_angle < moy_pitch_angle + 5)) {
		return true;
	}
	else {
		return false;
	}
}

void log_pitch_angle(float pitch_angle) {
	for (int i = 198; i>=0; i--) {
		log_pitch[i+1] = log_pitch[i];
	}
	log_pitch[0] = pitch_angle;
}


float CalcTSR()
{
	static const float RPM_TO_RADS = 2 * PI / 60; //0.10472


	//float rotor_speed_omega = RPM_TO_RADS * sensor_data.rotor_rpm;
	float rotor_speed_omega = RPM_TO_RADS * 1000;

	//float wind_speed_ms = KNOTS_TO_MS * sensor_data.wind_speed;
	float wind_speed_ms = sensor_data.wind_speed;
	//float wind_speed_ms = 15;

	if (abs(wind_speed_ms) < MIN_EPSILON)
		return 0.0f;


	float tsr = (PALE_RADIUS * rotor_speed_omega) / wind_speed_ms; //(0.874*x) / 15 = 6

	if (tsr < MIN_EPSILON)
		return 0.0f;
	return tsr;
}

float CalcPitchAuto()
{
//polynome vérifié en compé C12 calculs bon aout 2024
#define ALGO_C12 1

	float tsr = CalcTSR();

	if (ALGO_C12)
	{
		float pitch_target = 0;
		if (tsr <= 2.5) {
			pitch_target = 17;
		}
		else if (tsr >= 10) {
			pitch_target = 1;
		}
		else {
			float tsr2 = tsr * tsr;
			float tsr3 = tsr2 * tsr;
			float tsr4 = tsr2 * tsr2;
			float tsr5 = tsr4 * tsr;
			float tsr6 = tsr4 * tsr2;
			float tsr7 = tsr6 * tsr;
			float tsr8 = tsr6 * tsr2;
			float tsr9 = tsr8 * tsr;
			float tsr10 = tsr8 * tsr2;
			float tsr11 = tsr10 * tsr;

			pitch_target = -440.017f +
						   	  435.068f * tsr  +
							  196.501f * tsr2 -
							  487.301f * tsr3 +
							  324.152f * tsr4 -
							  119.048f * tsr5 +
							  27.5701f * tsr6 -
							  4.19495f * tsr7 +
							  0.419413f * tsr8 -
							  0.0265633f *tsr9 +
							  0.000967116 * tsr10 -
							  0.000015428 * tsr11;
		}

		if (pitch_target <= -2) {
			pitch_target = -2;
		}
		else if (pitch_target >= 17) {
			pitch_target = 17;
		}

		pitch_target = -pitch_target; // - selon le sens de rotation des pâles / rops pales en drapeau

		return pitch_target;
	}
}
