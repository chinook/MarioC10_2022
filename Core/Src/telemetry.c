/*
 * telemetry.c
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 */

#include "stm32f4xx_hal.h"

#include "main.h"
#include "state_machine.h"

void FloatToString(float value, int decimal_precision, unsigned char *val) {
	int integer = (int) value;
	int decimal = (int) ((value - (float) integer) * (float) (pow(10, decimal_precision)));
	decimal = abs(decimal);

	sprintf((char*) val, "%d.%d", integer, decimal);
}

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

	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*) &msg_data,
			sizeof(msg_data), 1);

	// unsigned char msg[1024] = { 0 };
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
	if (ret != HAL_OK) {
		// UART TX Error
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
	//delay_us(200);
}

static void UartTxAcquisition() {
//	float data[2];
//	uint32_t data_u32[2];
//	uint8_t data_u8[8];
//
//	data[0] = sensor_data.wind_speed;
//	data[1] = sensor_data.wind_direction;
//	// data[0] = 123.4f;
//	// data[1] = 1998.20f;
//	SendLoraFrame(0x101, (uint8_t*)&data);
//	delay_us(20);
//
//	data[0] = sensor_data.wheel_rpm;
//	data[1] = sensor_data.vehicle_speed;
//	SendLoraFrame(0x102, (uint8_t*)&data);
//	delay_us(20);
//
//	data[0] = sensor_data.rotor_rpm;
//	data[1] = sensor_data.torque;
//	SendLoraFrame(0x103, (uint8_t*)&data);
//	delay_us(20);
//
//	data[0] = sensor_data.loadcell;
//	data[1] = 0;
//	SendLoraFrame(0x104, (uint8_t*)&data);
//	delay_us(20);
//
//
//	data[0] = (float)sensor_data.pitch_encoder;
//	data[1] = sensor_data.pitch_angle;
//	SendLoraFrame(0x105, (uint8_t*)&data);
//	delay_us(20);
//
//
//	data_u32[0] = sensor_data.feedback_pitch_mode;
//	data_u32[1] = sensor_data.feedback_mast_mode;
//	SendLoraFrame(0x106, (uint8_t*)&data_u32);
//	delay_us(20);
//
//
//
//	uint32_t can_esr = CAN1->ESR;
//	data_u8[0] = (can_esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
//	data_u8[1] = (can_esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
//	data_u8[2] = (can_esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
//	data_u8[3] = (can_esr & CAN_ESR_BOFF_Msk) >> CAN_ESR_BOFF_Pos;
//	data_u8[4] = (can_esr & CAN_ESR_EPVF_Msk) >> CAN_ESR_EPVF_Pos;
//	data_u8[5] = (can_esr & CAN_ESR_EWGF_Msk) >> CAN_ESR_EWGF_Pos;
//	data_u8[6] = data_u8[7] = 0;
//	SendLoraFrame(0x107, (uint8_t*)&data_u8);
//	delay_us(20);
//
//
//	// sensor_data.wind_speed = 5.6f;
//	// sensor_data.rotor_rpm = 400.0f;
//
//	// Power + TSR
//	data[0] = sensor_data.power;
//	data[1] = CalcTSR();
//	SendLoraFrame(0x108, (uint8_t*)&data);
//	delay_us(20);
//
//	data_u8[0] = b_rops;
//	SendLoraFrame(0x109, (uint8_t*)&data_u8);
//	delay_us(20);
//
//
//	//pitch_auto_target = CalcPitchAuto();
//	//pitch_rops_target = 0.0f;
//	data[0] = pitch_auto_target;
//	data[1] = pitch_rops_target;
//	// data[0] = 34.78f;
//	// data[1] = 190.6f;
//	SendLoraFrame(0x10C, (uint8_t*)&data);
//	delay_us(20);

	// float data1 = 12.45f;
	TransmitDataAcq(1, (int*) &sensor_data.pitch_angle);
	//delay_us(10);

#define KNOTS_TO_MS 0.514444f
	float wind_speed_ms = KNOTS_TO_MS * sensor_data.wind_speed;
	TransmitDataAcq(2, (int*) &wind_speed_ms);
	//delay_us(10);

	float wind_dir_adj = sensor_data.wind_direction - 120.0f;
	TransmitDataAcq(3, (int*) &wind_dir_adj);
	//delay_us(10);

	TransmitDataAcq(4, (int*) &sensor_data.rotor_rpm);
	//delay_us(10);

	TransmitDataAcq(5, (int*) &sensor_data.wheel_rpm);
	//delay_us(10);

	float tsr2 = CalcTSR();
	TransmitDataAcq(6, (int*) &tsr2);
	//delay_us(10);

	TransmitDataAcq(7, (int*) &sensor_data.torque);
	///delay_us(10);

	return;
	//
	// 1 : Pitch angle
	// 2 : Wind Speed
	// 3 : Wind Direction
	// 4 : Rotor RPM
	// 5 : Wheel RPM
	// 6 : TSR
	//
	//sprintf(msg, "Wind Speed = %d,  Wind Direction = %d \n\r", 1234, 5678);

	// char msg[] = "Hello World! \n\r";

	typedef union Trame_ {
		struct {
			uint16_t header;
			uint16_t id;
			// uint8_t data[4];
			float data;
			uint8_t crc;
			uint8_t padding[3];
		};
		uint8_t trame_data[12];
	} Trame;

	struct DataTemp {
		Trame pitch_angle_msg;
		Trame wind_speed_msg;
		Trame wind_direction_msg;
		Trame rotor_rpm_msg;
		Trame wheel_rpm_msg;
		Trame tsr_msg;
	} msg_data;

	msg_data.pitch_angle_msg.header = 0x1177;
	msg_data.wind_speed_msg.header = 0x1177;
	msg_data.wind_direction_msg.header = 0x1177;
	msg_data.rotor_rpm_msg.header = 0x1177;
	msg_data.wheel_rpm_msg.header = 0x1177;
	msg_data.tsr_msg.header = 0x1177;

	msg_data.pitch_angle_msg.id = 1;
	msg_data.wind_speed_msg.id = 2;
	msg_data.wind_direction_msg.id = 3;
	msg_data.rotor_rpm_msg.id = 4;
	msg_data.wheel_rpm_msg.id = 5;
	msg_data.tsr_msg.id = 6;

	msg_data.pitch_angle_msg.data = sensor_data.pitch_angle;
	msg_data.wind_speed_msg.data = sensor_data.wind_speed;
	msg_data.wind_direction_msg.data = sensor_data.wind_direction;
	msg_data.rotor_rpm_msg.data = sensor_data.rotor_rpm;
	msg_data.wheel_rpm_msg.data = sensor_data.wheel_rpm;
	float tsr = CalcTSR();
	tsr = 12.78f;
	msg_data.tsr_msg.data = tsr;

	// uint32_t header = 0x1177;

	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*) &msg_data,
			sizeof(msg_data), 1);

	// unsigned char msg[1024] = { 0 };
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
	if (ret != HAL_OK) {
		// UART TX Error
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
	//delay_us(200);
}


static uint32_t tel_temp = 0;
uint32_t DoStateUartTx() {
	if (flag_telemetry) {
		flag_telemetry = 0;

		//HAL_UART_Transmit(&huart1, (uint8_t*) &live_counter, sizeof(live_counter),
				//HAL_MAX_DELAY);

		HAL_UART_Receive(&huart1, (uint8_t*) &tel_temp, sizeof(tel_temp),
		HAL_MAX_DELAY);
	}

	if (flag_uart_tx_send) {
		flag_uart_tx_send = 0;

		// UartTxAcquisition();
		// return STATE_ACQUISITION;

		//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
		// HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		// HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		static unsigned char wind_speed_str[20] = { 0 };
		static unsigned char wind_direction_str[20] = { 0 };
		FloatToString(sensor_data.wind_speed, 2, wind_speed_str);
		FloatToString(sensor_data.wind_direction, 2, wind_direction_str);

		static unsigned char rotor_rpm_str[20] = { 0 };
		static unsigned char wheel_rpm_str[20] = { 0 };
		FloatToString(sensor_data.rotor_rpm, 2, rotor_rpm_str);
		FloatToString(sensor_data.wheel_rpm, 2, wheel_rpm_str);

		static unsigned char torque_str[20] = { 0 };
		static unsigned char loadcell_str[20] = { 0 };
		FloatToString(sensor_data.torque, 2, torque_str);
		FloatToString(sensor_data.loadcell, 2, loadcell_str);

		// Compute angle from pitch encoder value
		//float pitch_angle = CalcPitchAnglePales(TRUE);
		//static unsigned char pitch_angle_str[20] = {0};
		//FloatToString(pitch_angle, 2, pitch_angle_str);

		//static char clear_cmd[] = "\33c\e[3J";
		//if (HAL_OK != HAL_UART_Transmit(&huart2, (uint8_t*)clear_cmd, strlen(clear_cmd), 1))
		//{
		//	// UART TX Error
		//	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		//}

		// float tsr = CalcTSR();
		static unsigned char tsr_str[20] = { 0 };
		// FloatToString(tsr, 4, tsr_str);

		// float pitch_auto_target = CalcPitchAuto();
		static unsigned char pitch_auto_target_str[20] = { 0 };
		// FloatToString(pitch_auto_target, 4, pitch_auto_target_str);
		// unsigned char msg[] = "Hello World! ";

		char msg[1024] = { 0 };
//		sprintf(msg, "Wind Speed = %s,  Wind Direction = %s \n\rRotor rpm = %s,  wheel_rpm = %s \n\rPitch = %d,  angle = %s \n\rTorque = %s,  Loadcell = %s \n\rMast mode = %d,  Pitch mode = %d \n\rTSR = %s,  Pitch auto target = %s\n\rROPS = %d,  ROPS drive pitch = %d \n\r",
//				wind_speed_str, wind_direction_str,
//				rotor_rpm_str, wheel_rpm_str,
//				sensor_data.pitch_encoder, pitch_angle_str,
//				torque_str, loadcell_str,
//				sensor_data.feedback_mast_mode, sensor_data.feedback_pitch_mode,
//				tsr_str, pitch_auto_target_str,
//				b_rops, sensor_data.feedback_pitch_rops);
		//sprintf(msg, "Wind Speed = %d,  Wind Direction = %d \n\r", 1234, 5678);

		// char msg[] = "Hello World! \n\r";

		//HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
//		if (ret != HAL_OK)
//		{
//			// UART TX Error
//			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
//		}

		//delay_us(200);

		uint32_t can_esr = CAN1->ESR;
		uint8_t can_rec = (can_esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
		uint8_t can_tec = (can_esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
		uint8_t can_lec = (can_esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
		uint8_t can_boff = (can_esr & CAN_ESR_BOFF_Msk) >> CAN_ESR_BOFF_Pos;
		uint8_t can_error_passive = (can_esr & CAN_ESR_EPVF_Msk) >> CAN_ESR_EPVF_Pos;
		uint8_t can_error_warning = (can_esr & CAN_ESR_EWGF_Msk) >> CAN_ESR_EWGF_Pos;

		char can_msg[512] = { 0 };
		static int test = 0;
		++test;
		sprintf(can_msg,
				"\n\r\n\rTEC = %d,  REC = %d  \n\rLast Error Code = %d \n\rBOFF = %d,  Error Passive = %d,  Error Warning = %d\n\r test = %d",
				can_tec, can_rec, can_lec, can_boff, can_error_passive, can_error_warning,
				test);

		//ret = HAL_UART_Transmit(&huart2, (uint8_t*)can_msg, strlen(can_msg), HAL_MAX_DELAY);
		// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
//		if (ret != HAL_OK)
//		{
//			// UART TX Error
//			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
//		}

	}

	return STATE_ACQUISITION;
}
