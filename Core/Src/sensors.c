/*
 * sensors.c
 *
 *  Created on: Aug 17, 2025
 *      Author: thoma
 *
 *  Edited on : 26 septembre 2026
 *  	Editor: Simon B.
 */

#include "sensors.h"
#include "main.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

// Boutons-poussoirs de la carte (mis à jour par les interruptions, déclarés extern dans main.h)
uint8_t pb1_value = 0;
uint8_t pb2_value = 0;
uint8_t pb1_update = 0;
uint8_t pb2_update = 0;

// Debug : nombre d'impulsions de roue de la dernière période et maximum observé
volatile uint32_t debug_wheel_pulses_last = 0;
volatile uint32_t debug_wheel_pulses_max = 0;

// Drapeau levé par l'interruption ADC quand une conversion couple/charge est prête
uint8_t flag_IT_adc1_loadcell_torque = 0;

// Compteurs d'impulsions incrémentés par les interruptions EXTI (roue et rotor)
uint32_t wheel_rpm_counter = 0;
uint32_t rotor_rpm_counter = 0;

// Buffer circulaire des 4 dernières vitesses de vent (pour la moyenne glissante)
float wind_speeds[4];
int wind_speed_last_idx = 0;

// Convertit une direction de vent de [0..360°] vers [-180..180°]
float wind_direction_n180_0_p180(float wind_direction_0_360) {
	float wind_direction_corrected = wind_direction_0_360;

	if (wind_direction_0_360 > 180 && wind_direction_0_360 < 360) {
		wind_direction_corrected = wind_direction_0_360 - 360;
	}
	return wind_direction_corrected;
}

// Buffer de réception UART de la station météo (rempli octet par octet par l'interruption)
uint8_t rx_buff[128];
uint8_t index_buff;
uint8_t ws_receive_flag;
uint8_t ws_rx_byte[4];

// Compteur de debug : nombre de trames météo reçues
uint32_t test_ws_receive_flag = 0;

#define KNOTS_TO_MS 0.514444f   // conversion noeuds -> m/s
#define WIND_DIR_AVG_LEN 20     // taille de la moyenne glissante de direction du vent

// Lit la vitesse et la direction du vent depuis la station météo (trame NMEA "$IIMWV" sur UART5)
void ReadWeatherStation() {
	if (!ws_receive_flag)
		return;
	ws_receive_flag = 0;

	static char frame_begin[] = "$IIMWV";

	// Copie protégée du buffer de réception (l'interruption peut le modifier)
	__disable_irq();
	static uint8_t ws_message[128] = { 0 };
	memcpy(ws_message, rx_buff, sizeof(ws_message));
	__enable_irq();

	if (strlen((char*) ws_message) < 6)
		return;

	// Vérifie que la trame commence bien par "$IIMWV"
	char begin_frame[7] = { 0 };
	memcpy(begin_frame, ws_message, 6);
	if (0 != strcmp(begin_frame, frame_begin))
		return;

	// Extrait la direction (0-360°) et la vitesse (noeuds) depuis la trame
	char wind_dir_msg[6] = { 0 };
	char wind_speed_msg[6] = { 0 };
	memcpy(wind_dir_msg, &ws_message[7], 5);
	memcpy(wind_speed_msg, &ws_message[15], 5);

	float wind_dir = atof(wind_dir_msg);
	float wind_speed = atof(wind_speed_msg);

	// Conversions : vitesse en m/s, direction en [-180..180°]
	wind_speed = KNOTS_TO_MS * wind_speed;
	wind_dir = wind_direction_n180_0_p180(wind_dir);

	// Moyenne glissante de la direction (buffer circulaire de 20 valeurs)
	static float wind_dir_log[WIND_DIR_AVG_LEN] = { 0 };
	static int wind_dir_idx = 0;
	wind_dir_log[wind_dir_idx] = wind_dir;
	wind_dir_idx = (wind_dir_idx + 1) % WIND_DIR_AVG_LEN;

	float sum_dir = 0;
	for (int i = 0; i < WIND_DIR_AVG_LEN; i++) {
		sum_dir += wind_dir_log[i];
	}
	sensor_data.wind_direction_avg = sum_dir / WIND_DIR_AVG_LEN;

	// Moyenne glissante de la vitesse (buffer circulaire de 4 valeurs)
	wind_speeds[wind_speed_last_idx++] = wind_speed;
	if (wind_speed_last_idx >= 4)
		wind_speed_last_idx = 0;
	sensor_data.wind_speed_avg = (wind_speeds[0] + wind_speeds[1] + wind_speeds[2] + wind_speeds[3]) / 4.0f;

	// Valeurs instantanées
	sensor_data.wind_direction = wind_dir;
	sensor_data.wind_speed = wind_speed;
}

// Facteurs de conversion et offsets de calibration du couple et de la cellule de charge
static const float TORQUE_RAW_TO_VALUE = 0.0390625;
float calibration_torque = 0;
static const float LOADCELL_RAW_TO_VALUE = 1 / (4096 / 200 * 2.2);
float calibration_loadcell = 0;

// Convertit les valeurs brutes de l'ADC en couple (N·m) et charge, avec calibration
void ADC_Raw_to_Value(uint32_t torque_raw, uint32_t loadcell_raw) {
	sensor_data.torque = ((float) torque_raw * TORQUE_RAW_TO_VALUE) + calibration_torque;
	sensor_data.loadcell = ((float) loadcell_raw * LOADCELL_RAW_TO_VALUE)
			+ calibration_loadcell;
}

// Valeurs ADC lues et canal courant (alternance couple/charge en mode interruption)
uint32_t adc_value_pb0 = 0;
uint32_t adc_value_pb1 = 0;
uint8_t adc_channel = 0;

// Lit le couple (PB0/CH8) et la charge (PB1/CH9) via l'ADC en mode interruption (non bloquant)
void ReadTorqueLoadcellADC_IT() {
	if (flag_IT_adc1_loadcell_torque == 1) {
		flag_IT_adc1_loadcell_torque = 0;

		ADC_ChannelConfTypeDef sConfig = { 0 };
		if (adc_channel == 0) {
			adc_value_pb0 = HAL_ADC_GetValue(&hadc1); // Read PB0 (ADC1_IN8) TORQUE
			adc_channel = 1;

			// Configure pour la lecture de la charge (loadcell)
			sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
			sConfig.Channel = ADC_CHANNEL_9;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
				Error_Handler();
			}
		} else {
			adc_value_pb1 = HAL_ADC_GetValue(&hadc1); // Read PB1 (ADC1_IN9) LOADCELL
			adc_channel = 0;

			// Configure pour la lecture du couple (torque)
			sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = 1;
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
				Error_Handler();
			}
		}

		HAL_ADC_Start_IT(&hadc1);

		ADC_Raw_to_Value(adc_value_pb0, adc_value_pb1);
	}
}

// Calcule le RPM de la roue à partir du compteur d'impulsions (appelée toutes les 500 ms)
void ReadWheelRPM() {

#define RPM_WHEEL_CNT_TIME_INVERSE 2.0f // équivaut à diviser par 500 ms
#define WHEEL_CNT_PER_ROT 64.0f

	static const float wheel_counter_to_rpm_constant = (RPM_WHEEL_CNT_TIME_INVERSE / WHEEL_CNT_PER_ROT) * 60.0f;

	uint32_t pulses = wheel_rpm_counter;
	debug_wheel_pulses_last = pulses;
	if (pulses > debug_wheel_pulses_max) {
		debug_wheel_pulses_max = pulses;
	}

	sensor_data.wheel_rpm = (float) pulses * wheel_counter_to_rpm_constant;
	wheel_rpm_counter = 0;
}

// Calcule la vitesse du véhicule (m/s) à partir du RPM de la roue
void CalcVehicleSpeed() {

#define WHEEL_DIAMETER 18.625f // diamètre de la roue en pouces

	// PI * diamètre (circonférence) * 0.0254 (pouces->m) / 60 (RPM->m/s)
	// (Pour des km/h : ... * 0.0254f * 60.0f / 1000.0f)
	static const float wheel_rpm_to_speed = PI * WHEEL_DIAMETER * 0.0254f / 60.0f;

	sensor_data.vehicle_speed = sensor_data.wheel_rpm * wheel_rpm_to_speed;
}

// Ratios de la transmission NuVinci (output/input), valeurs issues du PFE sur la transmission
static const float GEAR_RATIOS[14] = {0.1116f, 0.1264f, 0.1440f, 0.1636f, 0.1856f, 0.2112f, 0.2400f, 0.2728f, 0.3096f, 0.3524f, 0.4000f, 0.4540f, 0.5168f, 0.5868f };

// Détermine le gear NuVinci courant (1 à 14) à partir du ratio roue/rotor
void CalcCurrentGear(){
	if(sensor_data.wheel_rpm > 0.1f && sensor_data.rotor_rpm > 0.1f){
		sensor_data.gear_ratio = sensor_data.wheel_rpm / sensor_data.rotor_rpm;

		// Cherche le ratio prédéfini le plus proche
		uint8_t closest = 0;
		float min_diff = fabsf(sensor_data.gear_ratio - GEAR_RATIOS[0]);
		for(int i = 1; i < 14; i++){
			float diff = fabsf(sensor_data.gear_ratio - GEAR_RATIOS[i]);
			if(diff < min_diff){
				min_diff = diff;
				closest = i;
			}
		}
		sensor_data.current_gear = closest + 1;  // gear 1 à 14
	} else {
		sensor_data.gear_ratio = 0.0f;
		sensor_data.current_gear = 0;  // 0 = indéterminé
	}
}

// Calcule l'efficacité de traction (%) = vitesse véhicule / vitesse vent
void CalcEfficiency(void) {
	if (sensor_data.wind_speed < 0.1f) {
		sensor_data.efficiency = 0.0f;   // évite la division par zéro
	} else {
		sensor_data.efficiency = (sensor_data.vehicle_speed / sensor_data.wind_speed) * 100.0f;
	}
}

// Calcule le RPM du rotor avec filtre anti-bruit (appelée toutes les 100 ms)
void ReadRotorRPM() {
#define ROTOR_CNT_PER_ROT 360.0f
#define RPM_ROTOR_CNT_TIME_INVERSE 10.0f // équivaut à diviser par 100 ms
	static const float rotor_counter_to_rpm_constant = (RPM_ROTOR_CNT_TIME_INVERSE
			/ ROTOR_CNT_PER_ROT) * 60.0f;

	float rotor_rpm = (float) rotor_rpm_counter * rotor_counter_to_rpm_constant;
	rotor_rpm_counter = 0;

	// Anti-bruit : ignore un saut > 500 RPM sauf s'il se répète 4 fois de suite
#define RPM_ROTOR_ABR_IGNORE_CNT 4
	static int ignore_counter = 0;
	if (fabsf(sensor_data.rotor_rpm - rotor_rpm) > 500) {
		ignore_counter++;
		if (ignore_counter == RPM_ROTOR_ABR_IGNORE_CNT) {
			ignore_counter = 0;
			sensor_data.rotor_rpm = rotor_rpm; // saut confirmé : on accepte
		}
		return; // sinon on ignore cette lecture
	}
	ignore_counter = 0;

	sensor_data.rotor_rpm = rotor_rpm;
}

#define log_encoder_raw_data_size 10

// Moyenne d'un tableau d'entiers (accumulateur en double pour la précision)
double calculate_moy_uint32(uint32_t *data, uint32_t size) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += data[i];
	}
	return sum / size;
}

// Écart-type d'un tableau d'entiers (mesure la dispersion autour de la moyenne)
double calculate_std_dev_uint32(double moy, uint32_t *data, uint32_t size) {
	double sum = 0;
	for (int i = 0; i < size; i++) {
		sum += pow(((double) data[i]) - moy, 2);
	}
	return sqrt(sum / size);
}

// Comparaison croissante de deux uint32_t, pour qsort()
int compare(const void *a, const void *b) {
	uint32_t x = *(const uint32_t*) a;
	uint32_t y = *(const uint32_t*) b;
	if (x < y) return -1;
	if (x > y) return 1;
	return 0;
}

// Médiane d'un tableau d'entiers (résiste bien aux valeurs aberrantes)
double calculate_median_uint32(uint32_t *data, uint32_t size) {
	uint32_t data_temp[log_encoder_raw_data_size] = { 0 };

	for (int i = 0; i < size; i++) {
		data_temp[i] = data[i];
	}

	qsort(data_temp, size, sizeof(uint32_t), compare);

	if (size % 2 == 0) {
		return (data_temp[size / 2 - 1] + data_temp[size / 2]) / 2.0; // pair : moyenne des 2 du milieu
	} else {
		return data_temp[size / 2]; // impair : élément du milieu
	}
}

// Historique des 10 dernières lectures brutes de l'encodeur ([0] = plus récente)
uint32_t log_encoder_raw_data[log_encoder_raw_data_size] = { 0 };

// Filtre anti-bruit : remplace une lecture aberrante (> 1 écart-type) par la médiane
uint32_t verify_new_encoder_raw_data(uint32_t encoder_raw_data) {
	// Décale l'historique et insère la nouvelle valeur en tête
	for (int i = log_encoder_raw_data_size - 1; i > 0; i--) {
		log_encoder_raw_data[i] = log_encoder_raw_data[i - 1];
	}
	log_encoder_raw_data[0] = encoder_raw_data;

	double moy = calculate_moy_uint32(log_encoder_raw_data, log_encoder_raw_data_size);
	double std_dev = calculate_std_dev_uint32(moy, log_encoder_raw_data, log_encoder_raw_data_size);
	double lower_bound = moy - std_dev;
	double upper_bound = moy + std_dev;

	// Hors bornes = aberrante : on la remplace par la médiane
	if ((encoder_raw_data < lower_bound) || (encoder_raw_data > upper_bound)) {
		encoder_raw_data = (uint32_t) calculate_median_uint32(log_encoder_raw_data,
				log_encoder_raw_data_size);
	}

	return encoder_raw_data;
}

// Lit l'encodeur absolu de l'angle des pales (SSI 12 bits en bit-banging) puis filtre le résultat
#define PITCH_ENCODER_BITS 12
uint32_t ReadPitchEncoder() {
	// Impulsion d'horloge initiale
	HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
	delay_us(1);
	HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);
	delay_us(1);

	// Lit les 12 bits, du plus fort au plus faible (MSB first)
	uint32_t encoder_raw_data = 0;
	for (int i = 0; i < PITCH_ENCODER_BITS; i++) {
		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
		delay_us(1);

		encoder_raw_data <<= 1;
		if (HAL_GPIO_ReadPin(Mast_Data_GPIO_Port, Mast_Data_Pin)) {
			encoder_raw_data |= 1;
		}

		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);
		delay_us(1);
	}

	return verify_new_encoder_raw_data(encoder_raw_data);
}

// Lit l'encodeur du mât (SSI 22 bits). Non appelée pour l'instant + partage les pins du pitch
uint32_t ReadMastEncoder() {
	uint32_t mast_data = 0;
	for (int i = 0; i < 22; ++i) {
		mast_data <<= 1;

		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);

		mast_data |= HAL_GPIO_ReadPin(Mast_Data_GPIO_Port, Mast_Data_Pin);
	}
	return mast_data;
}

