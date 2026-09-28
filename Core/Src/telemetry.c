/*
 * telemetry.c
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 */

#include "stm32f4xx_hal.h"

#include "main.h"
#include "state_machine.h"
#include "pitch.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
