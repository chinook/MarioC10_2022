/*
 * state_machine.c
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 */
#include "state_machine.h"

#include "stm32f4xx_it.h"

#include "can.h"
#include "chinook_can_ids.h"
#include "main.h"
#include "motor_control.h"
#include "pitch.h"
#include "sensors.h"

uint32_t DoStateCan();

void delay_us(uint16_t delay16_us);

uint32_t current_state = STATE_INIT;

SensorData sensor_data;

uint8_t pitch_done = 0;

uint8_t rops_status = 0;

uint8_t test_buttons_volant = 0;

uint8_t flag_alive_led = 0;
uint8_t flag_uart_tx_send = 0;
uint8_t flag_telemetry = 0;
uint8_t flag_lora_tx_send = 0;
uint8_t flag_can_tx_send = 0;
// 100ms flags
uint8_t flag_acq_interval = 0;
// 50ms flags
uint8_t flag_wheel_rpm_process = 0;
uint8_t flag_rotor_rpm_process = 0;

uint8_t flag_motor_control = 0;
uint8_t flag_affichage_volant = 0;

uint32_t DoStateInit() {
	wheel_rpm_counter = 0;
	rotor_rpm_counter = 0;
	// rpm_counter_time = 0;

	index_buff = 0;
	ws_receive_flag = 0;

	timer_1ms_flag = 0;
	timer_100ms_flag = 0;
	timer_500ms_flag = 0;

	flag_alive_led = 0;
	flag_uart_tx_send = 0;
	flag_acq_interval = 0;
	flag_wheel_rpm_process = 0;
	flag_rotor_rpm_process = 0;
	flag_can_tx_send = 0;

	rops_status = 0;

	memset(&sensor_data, 0, sizeof(SensorData));

	// Causes strange bug where stm32 is still executing interrupts but not main code ....
	// Start interrupts
	HAL_UART_Receive_IT(&huart5, &ws_rx_byte[0], 1);

	//CanInit();

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim5);

	// Init ADC + calibration
	//HAL_ADCEx_Calibration_Start(&hadc1);

	// Enable USB TX
	HAL_GPIO_WritePin(USB_TX_EN_GPIO_Port, USB_TX_EN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(PB_USB_RESET_GPIO_Port, PB_USB_RESET_Pin, GPIO_PIN_SET);

	/*
	 HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);
	 HAL_Delay(2);
	 HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_RESET);
	 HAL_Delay(2);
	 HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);
	 HAL_Delay(2);
	 */

	return STATE_ACQUISITION;
}

uint32_t DoStateAcquisition() {

	ReadWeatherStation();

	if (flag_acq_interval) {
		flag_acq_interval = 0;

		// Read pitch and mast encoders
		sensor_data.pitch_encoder = ReadPitchEncoder();
		sensor_data.pitch_angle = CalcPitchAngle_raw_to_deg();

		//sensor_data.mast_encoder = ReadMastEncoder();
		//TODO: (Marc) Mast Encoder
		//sensor_data.mast_encoder = 0;

		// Read limit switches of mast
		//sensor_data.limit1 = HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin);
		//sensor_data.limit2 = HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin);

		// Read Torque + Loadcell adc
//#define ADC_RESOLUTION 65536.0f  // (16 bits)
//#define MAX_TORQUE 500.0f  // 500 N.m
//#define MAX_LOADCELL 400.0f  // 400 Lbs
		//static const float torque_adc_factor = MAX_TORQUE / ADC_RESOLUTION;
		//static const float loadcell_adc_factor = MAX_LOADCELL / ADC_RESOLUTION;
		//sensor_data.torque = ReadTorqueADC() * torque_adc_factor;
		// sensor_data.torque = 0.0f;

		//sensor_data.torque = ReadTorqueADC();
		//sensor_data.loadcell = ReadLoadcellADC();

		//ReadTorqueLoadcellADC();
		ReadTorqueLoadcellADC_IT();
	}

	if (flag_wheel_rpm_process) {
		flag_wheel_rpm_process = 0;

//#define RPM_WHEEL_CNT_TIME_INVERSE 2.0f // Same as dividing by 500ms
//#define WHEEL_CNT_PER_ROT 48.0f
//		static const float wheel_counter_to_rpm_constant = (RPM_WHEEL_CNT_TIME_INVERSE / WHEEL_CNT_PER_ROT) * 60.0f;
//
//		sensor_data.wheel_rpm = (float)wheel_rpm_counter * wheel_counter_to_rpm_constant;

		//ReadWheelRPM();

		wheel_rpm_counter = 0;

		// Compute vehicle speed
#define WHEEL_DIAMETER 6.2f;
		static const float wheel_rpm_to_speed = 3.1415926535f * WHEEL_DIAMETER
		;

		//sensor_data.wheel_rpm = (((float)wheel_rpm_counter / 0.2f)/48.0f)*60.0f;

		sensor_data.vehicle_speed = sensor_data.wheel_rpm * wheel_rpm_to_speed;
	}

	if (flag_rotor_rpm_process) {
		flag_rotor_rpm_process = 0;

		ReadRotorRPM();
	}

	return STATE_CHECK_ROPS;
}

uint32_t DoStateCheckROPS() {
#define ROTOR_RPM_ROPS 1300

	if (sensor_data.rotor_rpm > ROTOR_RPM_ROPS || status_button_bdd == 1) {
		rops_status = 1;
	}
	/*else
	 {
	 rops_status = 0;
	 }*/

	if (rops_status)
		return STATE_ROPS;
	else
		return STATE_MOTOR_CONTROL;
}

uint8_t motor_mode_pitch = MOTOR_MODE_MANUAL;
uint8_t motor_mode_mast = MOTOR_MODE_MANUAL;

int8_t motor_direction_pitch = 0;
int8_t motor_direction_mast = 0;

uint8_t motor_speed_pitch = 0;
uint8_t motor_speed_mast = 0;

uint32_t DoStateMotorControl() {
	if (flag_motor_control == 1) {
		flag_motor_control = 0;

		if (rops_status == 1) {
			motor_mode_pitch = MOTOR_MODE_AUTOMATIC;
		} else if (status_button_mg == 1) {
			motor_mode_pitch = MOTOR_MODE_AUTOMATIC;
		} else if (status_button_mg == 0) {
			motor_mode_pitch = MOTOR_MODE_MANUAL;
		} else {
			//wrong code in CAN from Volant
		}

		if (motor_mode_pitch == MOTOR_MODE_AUTOMATIC) {
			//if (sensor_data.feedback_pitch_mode == MOTOR_MODE_AUTOMATIC) {
			DoPitchControl();
			//} else {
			//	TransmitCAN(CAN_ID_CMD_MARIO_PITCH_MODE, &motor_mode_pitch, 4, 1);
			//}
		} else if (motor_mode_pitch == MOTOR_MODE_MANUAL) {
			//if (sensor_data.feedback_pitch_mode == MOTOR_MODE_MANUAL) {
			motor_direction_pitch = 0;
			if (status_button_hgg == 1)
				motor_direction_pitch--;
			if (status_button_hg == 1)
				motor_direction_pitch++;

			uint8_t direction = MOTOR_DIRECTION_STOP;
			switch (motor_direction_pitch) {
			case -1: {
				//if (warning_pitch_angle_close_to_down != 1) {
					direction = MOTOR_DIRECTION_RIGHT;
				//} else {
				//	direction = MOTOR_DIRECTION_STOP;
				//}
				break;
			}
			case 0: {
				direction = MOTOR_DIRECTION_STOP;
				break;
			}
			case 1: {
				//if (warning_pitch_angle_close_to_up != 1) {
					direction = MOTOR_DIRECTION_LEFT;
				//} else {
				//	direction = MOTOR_DIRECTION_STOP;
				//}
				break;
			}
			default:
				direction = MOTOR_DIRECTION_STOP;
			}
			TransmitCAN(CAN_ID_CMD_MARIO_PITCH_DIRECTION, &direction, 4, 1);

			motor_speed_pitch = 0;
			if (direction != MOTOR_DIRECTION_STOP) {
				motor_speed_pitch = 100;
			}

			TransmitCAN(CAN_ID_CMD_MARIO_PITCH_SPEED, &motor_speed_pitch, 4, 1);

			//} else {
			//	TransmitCAN(CAN_ID_CMD_MARIO_PITCH_MODE, &motor_mode_pitch, 4, 1);
			//}
		}

		////////////////////////////
		if (status_button_md == 1) {
			motor_mode_mast = MOTOR_MODE_AUTOMATIC;
		} else if (status_button_md == 0) {
			motor_mode_mast = MOTOR_MODE_MANUAL;
		} else {
			//wrong code in CAN from Volant
		}
		if (motor_mode_mast == MOTOR_MODE_AUTOMATIC) {
			DoMastControl();
		} else if (motor_mode_mast == MOTOR_MODE_MANUAL) {
			motor_direction_mast = 0;
			if (status_button_hd == 1)
				motor_direction_mast--;
			if (status_button_hdd == 1)
				motor_direction_mast++;

			switch (motor_direction_mast) {
			case -1: {
				MotorMastSpeedDir(-100);
				break;
			}
			case 0: {
				MotorMastSpeedDir(0);
				break;
			}
			case 1: {
				MotorMastSpeedDir(100);
				break;
			}
			default:
				MotorMastSpeedDir(0);
			}
		} else {
			MotorMastSpeedDir(0);
		}
	}

	return STATE_CAN;
}

#define MAX_ERROR_ROPS 80
uint32_t DoStateROPS() {

	// Stay in ROPS
	while (rops_status) {
		//delay_us(250);

		// Check for timer flags
		if (timer_1ms_flag) {
			timer_1ms_flag = 0;

			flag_affichage_volant = 1;
			flag_acq_interval = 1;
		}
		if (timer_50ms_flag) {
			timer_50ms_flag = 0;

			flag_motor_control = 1;
		}
		if (timer_100ms_flag) {
			timer_100ms_flag = 0;

			flag_rotor_rpm_process = 1;

		}
		if (timer_250ms_flag) {
			timer_250ms_flag = 0;

		}
		if (timer_500ms_flag) {
			timer_500ms_flag = 0;

			flag_wheel_rpm_process = 1;
			flag_alive_led = 1;
			//flag_uart_tx_send = 1;
		}

		// Alive led
		if (flag_alive_led) {
			flag_alive_led = 0;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}

		// Check angle is at rops

		// TODO: (Marc) Will not work if value loops around the zero, need proper bounds checking

		//if (std::fabs(sensor_data.pitch_encoder - PITCH_ABSOLUTE_ROPS) > MAX_ERROR_ROPS)
		//{
		//	SendPitchROPSCmd();
		//}

		//SendPitchAngleCmd(90);

		DoStateAcquisition();
		DoStateMotorControl();

		DoStateCan();
		//DoStateDataLogging();
		//DoStateUartTx();

		DoStateCheckROPS();
	}

	// Exit ROPS state
	//uint32_t rops_cmd = ROPS_DISABLE;
	//TransmitCAN(CAN_ID_CMD_MARIO_ROPS, (uint8_t*)&rops_cmd, 4, 0);

	return STATE_ACQUISITION;
}

float update_test = 0;

uint32_t DoStateCan() {
	//affichage volant 16 cmd au 1ms donc 16ms de refresh rate
	if (flag_affichage_volant == 1) // every 1ms
			{
		flag_affichage_volant = 0;

		//update_test += 1;
		//if (update_test >= 1) update_test = 0.1f;
		//update_test = test_buttons_volant;

		static uint32_t can_tx_state = 0;
		switch (can_tx_state) {
		case 0: {
			float turb_dir_value = test_ws_receive_flag + update_test + status_button_hgg;
			TransmitCAN(CAN_ID_MARIO_VAL_TURB_DIR, (uint8_t*) &turb_dir_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 1: {
			float turb_cmd_value = update_test + status_button_hg;
			TransmitCAN(CAN_ID_MARIO_VAL_TURB_CMD, (uint8_t*) &turb_cmd_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 2: {
			float wind_dir_value = (float) sensor_data.wind_direction + update_test
					+ status_button_hd;
			TransmitCAN(CAN_ID_MARIO_VAL_WIND_DIR, (uint8_t*) &wind_dir_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 3: {
			float speed_value = (float) sensor_data.wheel_rpm + update_test
					+ status_button_hdd;
			TransmitCAN(CAN_ID_MARIO_VAL_SPEED, (uint8_t*) &speed_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 4: {
			float tsr_value = CalcTSR() + update_test + status_button_mg;
			TransmitCAN(CAN_ID_MARIO_VAL_TSR, (uint8_t*) &tsr_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 5: {
			float gear_ratio_value = sensor_data.pitch_encoder + update_test
					+ status_button_md;
			TransmitCAN(CAN_ID_MARIO_VAL_GEAR_RATIO, (uint8_t*) &gear_ratio_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 6: {
			float rotor_speed_value = (float) sensor_data.rotor_rpm + update_test
					+ status_button_bgg;
			TransmitCAN(CAN_ID_MARIO_VAL_ROTOR_SPEED, (uint8_t*) &rotor_speed_value, 4,
					0);
			can_tx_state++;
			break;
		}
		case 7: {
			float rotor_rops_cmd_value = update_test + ROTOR_RPM_ROPS + status_button_bg;
			TransmitCAN(CAN_ID_MARIO_VAL_ROTOR_ROPS_CMD, (uint8_t*) &rotor_rops_cmd_value,
					4, 0);
			can_tx_state++;
			break;
		}
		case 8: {
			float pitch_value = (float) sensor_data.pitch_angle + update_test
					+ status_button_bd;
			//float pitch_value = (float)((sensor_data.pitch_encoder * ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS) + 0) + update_test;
			//float pitch_value = (float)sensor_data.pitch_encoder;
			TransmitCAN(CAN_ID_MARIO_VAL_PITCH, (uint8_t*) &pitch_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 9: {
			float efficiency_value = update_test + motor_mode_pitch + status_button_bdd;
			TransmitCAN(CAN_ID_MARIO_VAL_EFFICIENCY, (uint8_t*) &efficiency_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 10: {
			float wind_speed_value = (float) sensor_data.wind_speed + update_test;
			TransmitCAN(CAN_ID_MARIO_VAL_WIND_SPEED, (uint8_t*) &wind_speed_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 11: {
			float pitch_cmd_value = pitch_auto_target + update_test;
			TransmitCAN(CAN_ID_MARIO_VAL_PITCH_CMD, (uint8_t*) &pitch_cmd_value, 4, 0);
			can_tx_state++;
			break;
		}
		case 12: {
			float debug_log_1_value = (motor_mode_pitch * 100000) + update_test
					+ sensor_data.torque; //status_button_bgg + motor_mode_pitch +
			TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_1, (uint8_t*) &debug_log_1_value, 4,
					0);
			can_tx_state++;
			break;
		}
		case 13: {
			float debug_log_2_value = (motor_mode_mast * 100000) + update_test
					+ sensor_data.loadcell; //status_button_bg + motor_direction_pitch +
			TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_2, (uint8_t*) &debug_log_2_value, 4,
					0);
			can_tx_state++;
			break;
		}
		case 14: {
			float debug_log_3_value = (rops_status * 777) + update_test + motor_speed_pitch;
			TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_3, (uint8_t*) &debug_log_3_value, 4,
					0);
			can_tx_state++;
			break;
		}
		case 15: {
			float debug_log_4_value = status_button_debug + update_test;
			TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_4, (uint8_t*) &debug_log_4_value, 4,
					0);
			can_tx_state = 0;
			break;
		}
		default:
			// Unknown CAN ID
			can_tx_state = 0;
			break;
		}
	}

	return STATE_DATA_LOGGING;
}

uint32_t DoStateDataLogging() {
	return STATE_UART_TX;
}

void DoStateError() {
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
		delay_ms(250);
	}

	Error_Handler();
}

uint32_t counter_delay_us = 0;
void ExecuteStateMachine() {
	/*
	 while (1) {
	 if (timer_1ms_flag) {
	 timer_1ms_flag = 0;

	 counter_delay_us = 0;

	 }
	 delay_us(1);
	 counter_delay_us++;
	 }
	 */
	// Check for timer flags
	if (timer_1ms_flag) {
		timer_1ms_flag = 0;

		flag_affichage_volant = 1;
		flag_acq_interval = 1;
		flag_motor_control = 1;
	}
	if (timer_50ms_flag) {
		timer_50ms_flag = 0;

		//flag_motor_control = 1;
	}
	if (timer_100ms_flag) {
		timer_100ms_flag = 0;

		flag_rotor_rpm_process = 1;
	}
	if (timer_250ms_flag) {
		timer_250ms_flag = 0;

	}
	if (timer_500ms_flag) {
		timer_500ms_flag = 0;

		flag_alive_led = 1;

		flag_wheel_rpm_process = 1;
		flag_uart_tx_send = 1;

		//flag_telemetry = 1;
	}

	// Alive led
	if (flag_alive_led) {
		flag_alive_led = 0;
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}

	switch (current_state) {
	case STATE_INIT:
		current_state = DoStateInit();
		break;
	case STATE_ACQUISITION:
		current_state = DoStateAcquisition();
		break;
	case STATE_CHECK_ROPS:
		current_state = DoStateCheckROPS();
		break;
	case STATE_ROPS:
		current_state = DoStateROPS();
		break;
	case STATE_MOTOR_CONTROL:
		current_state = DoStateMotorControl();
		break;
	case STATE_CAN:
		current_state = DoStateCan();
		break;
	case STATE_DATA_LOGGING:
		current_state = DoStateDataLogging();
		break;
	case STATE_UART_TX:
		current_state = DoStateUartTx();
		break;
	case STATE_ERROR:
		DoStateError();
		// In case we get out of error handling, restart
		current_state = DoStateInit();
		break;
	default:
		current_state = DoStateInit();
		break;
	};
}

void delay_us(uint16_t delay16_us) {
	htim1.Instance->CNT = 0;
	while (htim1.Instance->CNT < delay16_us)
		;
}

void delay_ms(uint16_t delay16_ms) {
	while (delay16_ms > 0) {
		htim1.Instance->CNT = 0;
		delay16_ms--;
		while (htim1.Instance->CNT < 1000)
			;
	}
}
