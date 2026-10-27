/*
 * telemetry.c
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 *
 *  Edited on : Oct 1, 2026
 *  	Editor: Simon B.
 */

#include "stm32f4xx_hal.h"

#include "fatfs.h"
#include "main.h"
#include "state_machine.h"
#include "pitch.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Separe un float en partie entiere et en 2 decimales (le printf de la STM32 ne gere pas %f).
#define SD_INT(v)  ((int)(v))
#define SD_DEC(v)  ((int)(((v) >= 0 ? (v) - (int)(v) : (int)(v) - (v)) * 100))

// Ecrit une ligne de donnees dans le fichier CSV de la session sur la carte micro SD.
// Au premier appel, cherche le prochain dataN.csv libre. L'en-tete est ecrit si le fichier est vide.
// LED2 allumee = montage SD echoue ; LED2/3/4 = code d'erreur FatFs (binaire) si l'ouverture echoue.
void SdLog_WriteRow(const SensorData *d) {
	UINT bw;
	char line[320];
	static char sd_filename[16] = "";  // Nom du fichier pour cette session

	FRESULT res_mount = f_mount(&SDFatFS, SDPath, 1);
	if (res_mount != FR_OK) {
		// SD mount failed — LED2 ON to signal error
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		return;
	}

	// Au premier appel, trouver le prochain numéro disponible
	if (sd_filename[0] == '\0') {
		for (int i = 1; i <= 9999; i++) {
			sprintf(sd_filename, "0:/data%d.csv", i);
			FRESULT res_check = f_open(&SDFile, sd_filename, FA_READ);
			if (res_check == FR_NO_FILE) {
				break;  // Ce numéro est libre
			}
			f_close(&SDFile);
		}
	}

	FRESULT res_open = f_open(&SDFile, sd_filename, FA_OPEN_APPEND | FA_WRITE | FA_READ);
	if (res_open != FR_OK) {
		// Affiche le code d'erreur en binaire sur LED2 (bit0), LED3 (bit1), LED4 (bit2)
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, (res_open & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, (res_open & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, (res_open & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else {
		// SD OK — LED2 OFF
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

		// Write CSV header only if file is empty (first run)
		if (f_size(&SDFile) == 0) {
			const char *header = "time,vehicle_speed,wind_speed,current_gear,gear_ratio,pitch_angle,pitch_auto_target,mast_angle,rotor_rpm,wind_direction,efficiency,tsr\r\n";
			f_write(&SDFile, header, strlen(header), &bw);
		}

		// Gather all values
		uint32_t time       = HAL_GetTick() / 1000;
		float    v_spd      = d->vehicle_speed;
		float    w_spd      = d->wind_speed;
		uint8_t  gear       = d->current_gear;
		float    gear_ratio = d->gear_ratio;
		float    pitch      = d->pitch_angle;
		float    pitch_tgt  = d->pitch_auto_target;
		float    mast       = d->mast_angle;
		float    rotor      = d->rotor_rpm;
		float    wind_dir   = d->wind_direction;
		float    eff        = d->efficiency;
		float    tsr        = d->tsr;

		// Write data row (no %f — use integer math for floats)
		sprintf(line,
			"%lu,%d.%02d,%d.%02d,%u,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d\r\n",
			time,
			SD_INT(v_spd),      SD_DEC(v_spd),
			SD_INT(w_spd),      SD_DEC(w_spd),
			gear,
			SD_INT(gear_ratio), SD_DEC(gear_ratio),
			SD_INT(pitch),      SD_DEC(pitch),
			SD_INT(pitch_tgt),  SD_DEC(pitch_tgt),
			SD_INT(mast),       SD_DEC(mast),
			SD_INT(rotor),      SD_DEC(rotor),
			SD_INT(wind_dir),   SD_DEC(wind_dir),
			SD_INT(eff),        SD_DEC(eff),
			SD_INT(tsr),        SD_DEC(tsr)
		);

		f_write(&SDFile, line, strlen(line), &bw);
		f_close(&SDFile);
	}
	f_mount(NULL, SDPath, 1);
}

// Convertit un float en chaine "entier.decimal" (decimal_precision decimales).
void FloatToString(float value, int decimal_precision, unsigned char *val) {
	int integer = (int) value;
	int decimal = (int) ((value - (float) integer) * (float) (pow(10, decimal_precision)));
	decimal = abs(decimal);

	sprintf((char*) val, "%d.%0*d", integer, decimal_precision, decimal);
}

// Envoie une trame de 12 octets sur l'UART : [0x77][0x11][id 16 bits][4 octets data][4 octets padding].
static void TransmitDataAcq(int id, int *data) {
	unsigned char msg_data[12];

	msg_data[0] = 0x77;
	msg_data[1] = 0x11;

	msg_data[2] = id & 0xFF;
	msg_data[3] = (id & 0xFF00) >> 8;

	msg_data[4] = (*data & 0xFF);
	msg_data[5] = (*data & 0xFF00) >> 8;
	msg_data[6] = (*data & 0xFF0000) >> 16;
	msg_data[7] = (*data & 0xFF000000) >> 24;

	msg_data[8] = 0;
	msg_data[9] = 0;
	msg_data[10] = 0;
	msg_data[11] = 0;

	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart1, (uint8_t*) &msg_data,sizeof(msg_data), 1);

	if (ret != HAL_OK) { // erreur TX UART
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
}

// Envoie les valeurs capteurs par UART (une trame par valeur).
// IDs : 1=Pitch angle, 2=Wind speed, 3=Wind direction, 4=Rotor RPM, 5=Wheel RPM, 6=TSR, 7=Torque
static void UartTxAcquisition() {
	TransmitDataAcq(1, (int*) &sensor_data.pitch_angle);
	TransmitDataAcq(2, (int*) &sensor_data.wind_speed);
	TransmitDataAcq(3, (int*) &sensor_data.wind_direction);
	TransmitDataAcq(4, (int*) &sensor_data.rotor_rpm);
	TransmitDataAcq(5, (int*) &sensor_data.wheel_rpm);
	TransmitDataAcq(6, (int*) &sensor_data.tsr);
	TransmitDataAcq(7, (int*) &sensor_data.torque);
}

static uint32_t tel_temp = 0;
uint32_t DoStateUartTx() {
	if (flag_telemetry) {
		flag_telemetry = 0;
		HAL_UART_Receive(&huart1, (uint8_t*) &tel_temp, sizeof(tel_temp), HAL_MAX_DELAY);
	}

	if (flag_uart_tx_send) {
		flag_uart_tx_send = 0;
		// Pour reactiver la telemetrie radio, decommenter :
		//UartTxAcquisition();
	}

	return STATE_ACQUISITION;
}
