/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "chinook_can_ids.h"

#include "motor_control.h"
#include "pitch.h"
#include "sensors.h"
#include "can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TODO: (Marc) Address of ADC for torque/loadcell
#define ADC_I2C_ADDR 0x99

enum STATES
{
	STATE_INIT = 0,
	STATE_ACQUISITION,
	STATE_CHECK_ROPS,
	STATE_MOTOR_CONTROL,
	STATE_CAN,
	STATE_DATA_LOGGING,
	STATE_UART_TX,

	STATE_ROPS,

	STATE_ERROR = 0xFF

};

#define FALSE 0
#define TRUE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

//SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t current_state = STATE_INIT;

SensorData sensor_data;

//uint32_t wheel_rpm_counter;
//uint32_t rotor_rpm_counter;
//uint32_t rpm_counter_time;

// Weather station
//uint8_t rx_buff[128];
//uint8_t index_buff;
//uint8_t ws_receive_flag;

//uint8_t aRxBuffer[256];

//uint8_t ws_rx_byte;
//uint8_t ws_rx_byte[4];

uint8_t timer_1ms_flag;
uint8_t timer_100ms_flag;
uint8_t timer_250ms_flag;
uint8_t timer_500ms_flag;

// 500ms flags
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

// CAN variables
//uint8_t txData[8];
//uint8_t rxData[8];

// uint8_t can1_recv_flag = 0;
//CAN_TxHeaderTypeDef pTxHeader;
//CAN_RxHeaderTypeDef pRxHeader;
//uint32_t txMailbox;

//uint8_t pb1_value = 0;
//uint8_t pb2_value = 0;
//uint8_t pb1_update = 0;
//uint8_t pb2_update = 0;

uint8_t pitch_done = 0;

uint8_t rops_status = 0;

uint8_t test_buttons_volant = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void ExecuteStateMachine();

uint32_t DoStateInit();
uint32_t DoStateAcquisition();
uint32_t DoStateCheckROPS();
uint32_t DoStateMotorControl();
uint32_t DoStateROPS();
uint32_t DoStateCan();
uint32_t DoStateDataLogging();
uint32_t DoStateUartTx();

void DoStateError();

void FloatToString(float value, int decimal_precision, unsigned char* val);

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority);

void delay_us(uint16_t delay16_us);
void delay_ms(uint16_t delay16_ms);



int main(void)
{
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();


  flag_weather_station = 0;

  current_state = STATE_INIT;

  while (1) {
	  ExecuteStateMachine();
  }

}

uint16_t timer_50ms_counter = 0;
uint16_t timer_100ms_counter = 0;
uint16_t timer_250ms_counter = 0;
uint16_t timer_500ms_counter = 0;

uint8_t flag_motor_control = 0;
uint8_t flag_affichage_volant = 0;

void ExecuteStateMachine()
{
	// Check for timer flags
	if (timer_1ms_flag) {
		timer_1ms_flag = 0;
		timer_50ms_counter++;
		timer_100ms_counter++;
		timer_250ms_counter++;
		timer_500ms_counter++;


		flag_affichage_volant = 1;
		flag_acq_interval = 1;
	}
	if (timer_50ms_counter >= 50) {
		timer_50ms_counter = 0;

		flag_motor_control = 1;
	}
	if (timer_100ms_counter >= 100) {
		timer_100ms_counter = 0;

		flag_rotor_rpm_process = 1;
		flag_weather_station = 1;
	}
	if (timer_250ms_counter >= 250) {
		timer_250ms_counter = 0;

	}
	if (timer_500ms_counter >= 500) {
		timer_500ms_counter = 0;
		flag_alive_led = 1;


		flag_wheel_rpm_process = 1;
		flag_uart_tx_send = 1;

		//flag_telemetry = 1;
	}

	// Alive led
	if (flag_alive_led)
	{
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




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void delay_us(uint16_t delay16_us)
{
	htim1.Instance->CNT = 0;
	while (htim1.Instance->CNT < delay16_us);
}

void delay_ms(uint16_t delay16_ms)
{
	while(delay16_ms > 0)
	{
		htim1.Instance->CNT = 0;
		delay16_ms--;
		while (htim1.Instance->CNT < 1000);
	}
}

static float BoundAngleSemiCircle(float angle)
{
	if (angle < -180.0f)
		return angle + 360.0f;
	else if (angle > 180.0f)
		return angle - 360.0f;
	else
		return angle;
}

#define MAX_STEPS_PER_CMD 500

void SendROPSCmdCan(uint32_t rops_cmd)
{
	TransmitCAN(CAN_ID_CMD_MARIO_ROPS, (uint8_t*)&rops_cmd, 4, 0);
	//delay_us(100);
}
void SendPitchCmdCan(int nb_steps)
{
	TransmitCAN(CAN_ID_CMD_MARIO_PITCH_SPEED, (uint8_t*)&nb_steps, 4, 0);
	pitch_done = 0;
	//delay_us(100);
}

HAL_StatusTypeDef ADC_SendI2C(uint8_t addr, uint8_t reg, uint16_t data)
{
	uint8_t buf[3];

	buf[0] = reg;
	buf[1] = (data >> 8) & 0xFF;
	buf[2] = data & 0xFF;

	return HAL_I2C_Master_Transmit(&hi2c3, addr, buf, sizeof(buf), HAL_MAX_DELAY);
}

uint16_t ADC_ReadI2C(uint8_t addr, uint8_t reg)
{
	HAL_StatusTypeDef ret;

	uint8_t cmd[1];
	cmd[0] = reg;

	ret = HAL_I2C_Master_Transmit(&hi2c3, addr, cmd, sizeof(cmd), HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	// Reading 16bits for INA
	uint8_t buf[2];
	ret = HAL_I2C_Master_Receive(&hi2c3, addr, buf, sizeof(buf), HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		// TODO
	}

	// return buf[0];
	uint16_t result = (buf[0] << 8) | buf[1];
	return result;
}

float SimulateTSR()
{
	float random = ((float) rand()) / (float) RAND_MAX;
	float diff = 40.0f;
	float r = random * diff;
	return r;
}

uint32_t ReadPitchEncoder2()
{
	// SSI works from 100kHz to about 2MHz
	uint32_t pitch_data = 0;
	for(int i = 0; i < 22; ++i)
	// for(int i = 0; i < 12; ++i)
	{
		pitch_data <<= 1;

		HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_RESET);
		// for (int i = 0; i < 100; ++i) {} // Wait 10 us
		//delay_us(2);

		HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_SET);
		// for (int i = 0; i < 100; ++i) {} // Wait 10 us
		//delay_us(2);

		pitch_data |= HAL_GPIO_ReadPin(Pitch_Data_GPIO_Port, Pitch_Data_Pin);
	}

  	return pitch_data;
}

void FloatToString(float value, int decimal_precision, unsigned char* val)
{
	int integer = (int)value;
	int decimal = (int)((value - (float)integer) * (float)(pow(10, decimal_precision)));
	decimal = abs(decimal);

	sprintf((char*)val, "%d.%d", integer, decimal);
}

uint32_t DoStateInit()
{
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

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);

    // Init ADC + calibration
    //HAL_ADCEx_Calibration_Start(&hadc1);

	// Enable USB TX
	HAL_GPIO_WritePin(USB_TX_EN_GPIO_Port, USB_TX_EN_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(USB_RESET_GPIO_Port, USB_RESET_Pin, GPIO_PIN_SET);

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

uint32_t DoStateAcquisition()
{

	ReadWeatherStation();

	if (flag_acq_interval)
	{
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

	if (flag_wheel_rpm_process)
	{
		flag_wheel_rpm_process = 0;

//#define RPM_WHEEL_CNT_TIME_INVERSE 2.0f // Same as dividing by 500ms
//#define WHEEL_CNT_PER_ROT 48.0f
//		static const float wheel_counter_to_rpm_constant = (RPM_WHEEL_CNT_TIME_INVERSE / WHEEL_CNT_PER_ROT) * 60.0f;
//
//		sensor_data.wheel_rpm = (float)wheel_rpm_counter * wheel_counter_to_rpm_constant;

		ReadWheelRPM();

		wheel_rpm_counter = 0;

		// Compute vehicle speed
		#define WHEEL_DIAMETER 6.2f;
		static const float wheel_rpm_to_speed = 3.1415926535f * WHEEL_DIAMETER;

		sensor_data.vehicle_speed = sensor_data.wheel_rpm * wheel_rpm_to_speed;
	}

	if (flag_rotor_rpm_process)
	{
		flag_rotor_rpm_process = 0;

		ReadRotorRPM();
	}

	return STATE_CHECK_ROPS;
}

uint32_t DoStateCheckROPS()
{
#define ROTOR_RPM_ROPS 1300

	if (sensor_data.rotor_rpm > ROTOR_RPM_ROPS || status_button_bdd == 1)
	{
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

uint8_t motor_direction_pitch = 0;
int8_t motor_direction_mast = 0;

uint8_t motor_speed_pitch = 0;
uint8_t motor_speed_mast = 0;

uint32_t DoStateMotorControl()
{
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
			TransmitCAN(CAN_ID_CMD_MARIO_PITCH_MODE, &motor_mode_pitch, 4, 1);
		}
		else if (motor_mode_pitch == MOTOR_MODE_MANUAL) {
			if (sensor_data.feedback_pitch_mode == MOTOR_MODE_MANUAL) {
				motor_direction_pitch = 2;
				if (status_button_hgg == 1) motor_direction_pitch-=2;
				if (status_button_hg == 1) motor_direction_pitch+=2;

				uint8_t direction = MOTOR_DIRECTION_STOP;
				switch (motor_direction_pitch) {
					case MOTOR_DIRECTION_LEFT: {
						if (warning_pitch_angle_close_to_down != 1) {
							direction = MOTOR_DIRECTION_LEFT;
						} else {
							direction = MOTOR_DIRECTION_STOP;
						}
						break;
					} case MOTOR_DIRECTION_STOP: {
						direction = MOTOR_DIRECTION_STOP;
						break;
					} case MOTOR_DIRECTION_RIGHT: {
						if (warning_pitch_angle_close_to_up != 1) {
							direction = MOTOR_DIRECTION_RIGHT;
						} else {
							direction = MOTOR_DIRECTION_STOP;
						}
						break;
					} default :
						direction = MOTOR_DIRECTION_STOP;
				}
				motor_direction_pitch = direction;
				TransmitCAN(CAN_ID_CMD_MARIO_PITCH_DIRECTION, &motor_direction_pitch, 4, 1);
				motor_speed_pitch = 100;
				TransmitCAN(CAN_ID_CMD_MARIO_PITCH_SPEED, &motor_speed_pitch, 4, 1);
			} else {
				TransmitCAN(CAN_ID_CMD_MARIO_PITCH_MODE, &motor_mode_pitch, 4, 1);
			}
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
			if (sensor_data.feedback_mast_mode == MOTOR_MODE_AUTOMATIC) {
				DoMastControl();
			} else {
				TransmitCAN(CAN_ID_CMD_MARIO_MAST_MODE, &motor_mode_mast, 4, 1);
			}
		}
		else if (motor_mode_mast == MOTOR_MODE_MANUAL) {
			if (sensor_data.feedback_mast_mode == MOTOR_MODE_MANUAL) {
				motor_direction_mast = 0;
				if (status_button_hd == 1) motor_direction_mast--;
				if (status_button_hdd == 1) motor_direction_mast++;

				switch (motor_direction_mast) {
					case -1: {
						uint8_t cmd = MOTOR_DIRECTION_LEFT;
						TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, &cmd, 4, 1);
						break;
					} case 0: {
						uint8_t cmd = MOTOR_DIRECTION_STOP;
						TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, &cmd, 4, 1);
						break;
					} case 1: {
						uint8_t cmd = MOTOR_DIRECTION_RIGHT;
						TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, &cmd, 4, 1);
						break;
					} default :
						uint8_t cmd = MOTOR_DIRECTION_STOP;
						TransmitCAN(CAN_ID_CMD_MARIO_MAST_DIRECTION, &cmd, 4, 1);
				}
			} else {
				TransmitCAN(CAN_ID_CMD_MARIO_MAST_MODE, &motor_mode_mast, 4, 1);
			}
		}
	}

	return STATE_CAN;
}

uint32_t DoStateROPS()
{

	// Stay in ROPS
	while (rops_status)
	{
		//delay_us(250);

		// Check for timer flags
		if (timer_1ms_flag) {
			timer_1ms_flag = 0;
			timer_50ms_counter++;
			timer_100ms_counter++;
			timer_250ms_counter++;
			timer_500ms_counter++;

			flag_affichage_volant = 1;
			flag_acq_interval = 1;
		}
		if (timer_50ms_counter >= 50) {
			timer_50ms_counter = 0;

			flag_motor_control = 1;
		}
		if (timer_100ms_counter >= 100) {
			timer_100ms_counter = 0;

			flag_rotor_rpm_process = 1;

		}
		if (timer_250ms_counter >= 250) {
			timer_250ms_counter = 0;

			flag_weather_station = 1;
		}
		if (timer_500ms_counter >= 500) {
			timer_500ms_counter = 0;

			flag_wheel_rpm_process = 1;
			flag_alive_led = 1;
			//flag_uart_tx_send = 1;
		}

		// Alive led
		if (flag_alive_led)
		{
			flag_alive_led = 0;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}

		// Check angle is at rops
 #define MAX_ERROR_ROPS 80
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
static uint32_t live_counter = 0;

uint32_t DoStateCan()
{
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
				float turb_dir_value = update_test + status_button_hgg;
				TransmitCAN(CAN_ID_MARIO_VAL_TURB_DIR, (uint8_t*)&turb_dir_value, 4, 0);
				can_tx_state++;
				break;
			} case 1: {
				float turb_cmd_value = update_test + status_button_hg;
				TransmitCAN(CAN_ID_MARIO_VAL_TURB_CMD, (uint8_t*)&turb_cmd_value, 4, 0);
				can_tx_state++;
				break;
			} case 2: {
				float wind_dir_value = (float)sensor_data.wind_direction + update_test + status_button_hd;
				TransmitCAN(CAN_ID_MARIO_VAL_WIND_DIR, (uint8_t*)&wind_dir_value, 4, 0);
				can_tx_state++;
				break;
			} case 3: {
				float speed_value = (float)sensor_data.wheel_rpm + update_test + status_button_hdd;
				TransmitCAN(CAN_ID_MARIO_VAL_SPEED, (uint8_t*)&speed_value, 4, 0);
				can_tx_state++;
				break;
			} case 4: {
				float tsr_value = CalcTSR() + update_test + status_button_mg;
				TransmitCAN(CAN_ID_MARIO_VAL_TSR, (uint8_t*)&tsr_value, 4, 0);
				can_tx_state++;
				break;
			} case 5: {
				float gear_ratio_value = update_test + status_button_md;
				TransmitCAN(CAN_ID_MARIO_VAL_GEAR_RATIO, (uint8_t*)&gear_ratio_value, 4, 0);
				can_tx_state++;
				break;
			} case 6: {
				float rotor_speed_value = (float)sensor_data.rotor_rpm + update_test + status_button_bgg;
				TransmitCAN(CAN_ID_MARIO_VAL_ROTOR_SPEED, (uint8_t*)&rotor_speed_value, 4, 0);
				can_tx_state++;
				break;
			} case 7: {
				float rotor_rops_cmd_value = update_test + ROTOR_RPM_ROPS + status_button_bg;
				TransmitCAN(CAN_ID_MARIO_VAL_ROTOR_ROPS_CMD, (uint8_t*)&rotor_rops_cmd_value, 4, 0);
				can_tx_state++;
				break;
			} case 8: {
				float pitch_value = (float)sensor_data.pitch_angle + update_test + status_button_bd;
				//float pitch_value = (float)((sensor_data.pitch_encoder * ABSOLUTE_ENCODER_RESOLUTION_ANGLE_12BITS) + 0) + update_test;
				//float pitch_value = (float)sensor_data.pitch_encoder;
				TransmitCAN(CAN_ID_MARIO_VAL_PITCH, (uint8_t*)&pitch_value, 4, 0);
				can_tx_state++;
				break;
			} case 9: {
				float efficiency_value = update_test + motor_mode_pitch + status_button_bdd;
				TransmitCAN(CAN_ID_MARIO_VAL_EFFICIENCY, (uint8_t*)&efficiency_value, 4, 0);
				can_tx_state++;
				break;
			} case 10: {
				float wind_speed_value = (float)sensor_data.wind_speed + update_test;
				TransmitCAN(CAN_ID_MARIO_VAL_WIND_SPEED, (uint8_t*)&wind_speed_value, 4, 0);
				can_tx_state++;
				break;
			} case 11: {
				float pitch_cmd_value = pitch_auto_target + update_test;
				TransmitCAN(CAN_ID_MARIO_VAL_PITCH_CMD, (uint8_t*)&pitch_cmd_value, 4, 0);
				can_tx_state++;
				break;
			} case 12: {
				float debug_log_1_value = update_test + sensor_data.torque; //status_button_bgg + motor_mode_pitch +
				TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_1, (uint8_t*)&debug_log_1_value, 4, 0);
				can_tx_state++;
				break;
			} case 13: {
				float debug_log_2_value = update_test + sensor_data.loadcell; //status_button_bg + motor_direction_pitch +
				TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_2, (uint8_t*)&debug_log_2_value, 4, 0);
				can_tx_state++;
				break;
			} case 14: {
				float debug_log_3_value = update_test + live_counter + motor_speed_pitch;
				TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_3, (uint8_t*)&debug_log_3_value, 4, 0);
				can_tx_state++;
				break;
			} case 15: {
				float debug_log_4_value = status_button_debug + update_test;
				TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_4, (uint8_t*)&debug_log_4_value, 4, 0);
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

uint32_t DoStateDataLogging()
{
	return STATE_UART_TX;
}

static void TransmitDataAcq(int id, int* data)
{
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

	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)&msg_data, sizeof(msg_data), 1);

	// unsigned char msg[1024] = { 0 };
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
	if (ret != HAL_OK)
	{
		// UART TX Error
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
	//delay_us(200);
}

static void UartTxAcquisition()
{
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
	TransmitDataAcq(1, (int*)&sensor_data.pitch_angle);
	//delay_us(10);

#define KNOTS_TO_MS 0.514444f
	float wind_speed_ms = KNOTS_TO_MS * sensor_data.wind_speed;
	TransmitDataAcq(2, (int*)&wind_speed_ms);
	//delay_us(10);

	float wind_dir_adj = sensor_data.wind_direction - 120.0f;
	TransmitDataAcq(3, (int*)&wind_dir_adj);
	//delay_us(10);

	TransmitDataAcq(4, (int*)&sensor_data.rotor_rpm);
	//delay_us(10);

	TransmitDataAcq(5, (int*)&sensor_data.wheel_rpm);
	//delay_us(10);

	float tsr2 = CalcTSR();
	TransmitDataAcq(6, (int*)&tsr2);
	//delay_us(10);

	TransmitDataAcq(7, (int*)&sensor_data.torque);
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

	typedef union Trame_
	{
	  struct
	  {
	    uint16_t header;
	    uint16_t id;
	    // uint8_t data[4];
	    float data;
	    uint8_t crc;
	    uint8_t padding[3];
	  };
	  uint8_t trame_data[12];
	} Trame;

	struct DataTemp
	{
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

	HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, (uint8_t*)&msg_data, sizeof(msg_data), 1);

	// unsigned char msg[1024] = { 0 };
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), HAL_MAX_DELAY);
	// HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart2, msg, strlen(msg), 1);
	if (ret != HAL_OK)
	{
		// UART TX Error
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	}
	//delay_us(200);
}

static uint32_t tel_temp = 0;
uint32_t DoStateUartTx()
{
	if (flag_telemetry) {
		flag_telemetry = 0;

		live_counter++;
		HAL_UART_Transmit(&huart1, (uint8_t*)&live_counter, sizeof(live_counter), HAL_MAX_DELAY);

		HAL_UART_Receive(&huart1, (uint8_t*)&tel_temp, sizeof(tel_temp), HAL_MAX_DELAY);
		live_counter -= tel_temp;
		tel_temp = 0;
	}

	if (flag_uart_tx_send)
	{
		flag_uart_tx_send = 0;

		// UartTxAcquisition();
		// return STATE_ACQUISITION;


		//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
		// HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		// HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

		static unsigned char wind_speed_str[20] = {0};
		static unsigned char wind_direction_str[20] = {0};
		FloatToString(sensor_data.wind_speed, 2, wind_speed_str);
		FloatToString(sensor_data.wind_direction, 2, wind_direction_str);

		static unsigned char rotor_rpm_str[20] = {0};
		static unsigned char wheel_rpm_str[20] = {0};
		FloatToString(sensor_data.rotor_rpm, 2, rotor_rpm_str);
		FloatToString(sensor_data.wheel_rpm, 2, wheel_rpm_str);

		static unsigned char torque_str[20] = {0};
		static unsigned char loadcell_str[20] = {0};
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
		static unsigned char tsr_str[20] = {0};
		// FloatToString(tsr, 4, tsr_str);

		// float pitch_auto_target = CalcPitchAuto();
		static unsigned char pitch_auto_target_str[20] = {0};
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
		sprintf(can_msg, "\n\r\n\rTEC = %d,  REC = %d  \n\rLast Error Code = %d \n\rBOFF = %d,  Error Passive = %d,  Error Warning = %d\n\r test = %d",
				can_tec, can_rec, can_lec, can_boff, can_error_passive, can_error_warning, test);

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

void DoStateError()
{
	__disable_irq();
	while (1)
	{
		HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
		delay_ms(250);
	}

	Error_Handler();
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
  /* USER CODE BEGIN ADC1_Init 2 */

  HAL_ADC_Start_IT(&hadc1); //start interrrupt ADC1

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  /*
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  */
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  HAL_TIM_Base_Start(&htim1);

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  //htim2.Init.Period = 100;  == 1ms
  //htim2.Init.Period = 5000; == 50ms
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 480;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 480;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 4800;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  __HAL_RCC_USART1_CLK_ENABLE();

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pitch_Clock_Pin|LORA_EN_Pin|LORA_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, USB_RESET_Pin|USB_TX_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_WARNING_Pin|LED_ERROR_Pin|LED_CANA_Pin|LED_CANB_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Pitch_Clock_Pin LORA_EN_Pin LORA_RESET_Pin */
  GPIO_InitStruct.Pin = Pitch_Clock_Pin|LORA_EN_Pin|LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT1_Pin LIMIT2_Pin LORA_INT_Pin Rotor_RPM_Pin
                           Wheel_RPM_Pin */
  GPIO_InitStruct.Pin = LORA_INT_Pin|Rotor_RPM_Pin
                          |Wheel_RPM_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LIMIT1_Pin|LIMIT2_Pin;
    // GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Pitch_Data_Pin */
  GPIO_InitStruct.Pin = Pitch_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pitch_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Mast_Data_Pin */
  GPIO_InitStruct.Pin = Mast_Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Mast_Data_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Mast_Clock_Pin */
  GPIO_InitStruct.Pin = Mast_Clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Mast_Clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_RESET_Pin USB_TX_EN_Pin */
  GPIO_InitStruct.Pin = USB_RESET_Pin|USB_TX_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_CS_Pin */
  /*
  GPIO_InitStruct.Pin = LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);
  */

  /*Configure GPIO pins : LED_WARNING_Pin LED_ERROR_Pin LED_CANA_Pin LED_CANB_Pin
                           LED4_Pin LED3_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED_WARNING_Pin|LED_ERROR_Pin|LED_CANA_Pin|LED_CANB_Pin
                          |LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2_Pin PB1_Pin */
  GPIO_InitStruct.Pin = PB2_Pin|PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DETECT_Pin */
  GPIO_InitStruct.Pin = SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

