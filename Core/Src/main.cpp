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

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
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

uint8_t timer_50ms_flag;
uint8_t timer_100ms_flag;
uint8_t timer_250ms_flag;
uint8_t timer_500ms_flag;

// 500ms flags
uint8_t flag_alive_led = 0;
uint8_t flag_uart_tx_send = 0;
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

uint8_t b_rops = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
//static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
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

//void ProcessCanMessage();
//void CAN_ReceiveFifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifo);

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

void ExecuteStateMachine()
{
	// Check for timer flags
	static int timer_250ms_counter = 0;
	if (timer_50ms_flag)
	{
		flag_acq_interval = 1;

		flag_can_tx_send = 1;
		timer_50ms_flag = 0;

		timer_250ms_counter++;

		// flag_can_tx_send = 1;
		// flag_rpm_process = 1;
	}
	if (timer_250ms_counter == 5)
	{
		//flag_uart_tx_send = 1;


		timer_250ms_counter = 0;
	}
	if (timer_100ms_flag)
	{
		timer_100ms_flag = 0;

		flag_acq_interval = 1;
		flag_rotor_rpm_process = 1;
		flag_weather_station = 1;
	}
	if (timer_500ms_flag)
	{
		timer_500ms_flag = 0;

		//flag_uart_tx_send = 1;
		flag_wheel_rpm_process = 1;
		flag_alive_led = 1;
		flag_uart_tx_send = 1;
	}

	// Alive led
	if (flag_alive_led)
	{
		flag_alive_led = 0;
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}

	switch (current_state)
	{
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


//float CalcPitchAnglePales(uint8_t bound_angle)
//{
//	float delta_pitch = CalcDeltaPitch(PITCH_ABSOLUTE_ZERO);
//
//#define ENCODER_TO_PALES_RATIO (3.0f / 2.0f)
//	static const float PITCH_TO_ANGLE_RATIO = ENCODER_TO_PALES_RATIO * (360.0f / (float)MAX_PITCH_VALUE);
//	float pitch_angle = (float)delta_pitch * PITCH_TO_ANGLE_RATIO;
//
//	// Bound angle between -180 and 180 degrees
//	if (bound_angle)
//		pitch_angle = BoundAngleSemiCircle(pitch_angle);
//
//	return pitch_angle;
//}

//void SendPitchAngleCmd(float target_pitch)
//{
//	// Bounds checking
//	// if (target_angle_pales < 180.0f || target_angle_pales > 180.0f)
//	//	return;
//	float target_angle_pales = BoundAngleSemiCircle(target_pitch);
//
//	// Calculate current pitch angle from ABSOLUTE ZERO
//	// float current_pitch_angle_pales = (3.0f / 2.0f) * pitch_to_angle(current_pitch);
//	float angle_pales = CalcPitchAnglePales(FALSE);
//	angle_pales = BoundAngleSemiCircle(angle_pales);
//
//	// Calculate the delta pitch angle for the stepper motor
//	float delta_angle_pales = target_angle_pales - angle_pales;
//	delta_angle_pales = BoundAngleSemiCircle(delta_angle_pales);
//
//	static const float angle_pales_to_encoder_angle = (2.0f / 3.0f);
//	float delta_angle_encoder = angle_pales_to_encoder_angle * delta_angle_pales;
//
//	// Convert the target angle to stepper steps
//	// static const float angle_mov_per_step = 1.8f / 293.89f;
//	// int nb_steps = (int)(delta_pitch_angle_encodeur / angle_mov_per_step);
//	static const float angle_mov_per_step_inv = 293.89f / 1.8f;
//	int nb_steps = (int)(delta_angle_encoder * angle_mov_per_step_inv);
//
//	// Set a maximum to the number of steps so that we don't overshoot too much
//	// Plus, its safer in case of angle error
//	//if(abs(nb_steps) > 300) nb_steps = 300;
//	//nb_steps *= -1;
//
//
//	if(abs(nb_steps) > MAX_STEPS_PER_CMD)
//	{
//		if(nb_steps < 0)
//			nb_steps = -MAX_STEPS_PER_CMD;
//		else
//			nb_steps = MAX_STEPS_PER_CMD;
//	}
//	// nb_steps *= -1;
//	// TODO: Add checks and validation of steps
//
//	// Send the command to the stepper drive
//	// Only send cmd if stepper has finished last command
//	// Stepper drive will notice us when done with the pitch_done CAN message.
//	//if(true || pitch_done)
//	if(pitch_done)
//	{
//		// uint32_t nb_steps_cmd = (int)
//		TransmitCAN(MARIO_PITCH_CMD, (uint8_t*)&nb_steps, 4, 0);
//		pitch_done = 0;
//		delay_us(100);
//	}
//}

void SendROPSCmdCan(uint32_t rops_cmd)
{
	TransmitCAN(MARIO_ROPS_CMD, (uint8_t*)&rops_cmd, 4, 0);
	//delay_us(100);
}
void SendPitchCmdCan(int nb_steps)
{
	TransmitCAN(MARIO_PITCH_CMD, (uint8_t*)&nb_steps, 4, 0);
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

//static void ReadTorqueLoadcellADC()
//{
//	ADC_ChannelConfTypeDef sConfigChannel8 = {0};
//	sConfigChannel8.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//	sConfigChannel8.Channel = ADC_CHANNEL_8;
//	sConfigChannel8.Rank = 1;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfigChannel8) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1);
//	uint16_t adc_torque = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);
//
//
//	ADC_ChannelConfTypeDef sConfigChannel9 = {0};
//	sConfigChannel9.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//	sConfigChannel9.Channel = ADC_CHANNEL_9;
//	sConfigChannel9.Rank = 1;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfigChannel9) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 1);
//	uint16_t adc_loadcell = HAL_ADC_GetValue(&hadc1);
//	HAL_ADC_Stop(&hadc1);
//
//	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
//	static const float ADC_TO_TORQUE = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 160.0f / 4095.0f;
//	sensor_data.torque = (float)adc_torque * ADC_TO_TORQUE;
//
//	static const float ADC_TO_LOADCELL = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 500.0f / 4095.0f;
//	sensor_data.loadcell = (float)adc_loadcell * ADC_TO_LOADCELL;
//}

//float ReadTorqueADC()
//{
//
//	ADC_ChannelConfTypeDef sConfig = {0};
//	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//	sConfig.Channel = ADC_CHANNEL_8;
//    sConfig.Rank = 1;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//    	Error_Handler();
//    }
//
//
//    HAL_ADC_Start(&hadc1);
//
//	HAL_ADC_PollForConversion(&hadc1, 1);
//	uint16_t adc_result = HAL_ADC_GetValue(&hadc1);
//
//	HAL_ADC_Stop(&hadc1);
//
//	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
//	static const float ADC_TO_TORQUE = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 160.0f / 4095.0f;
//	return (float)adc_result * ADC_TO_TORQUE;
//	// return adc_result;
//}

//float ReadLoadcellADC()
//{
//
//	ADC_ChannelConfTypeDef sConfig = {0};
//	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//	sConfig.Channel = ADC_CHANNEL_9;
//	sConfig.Rank = 2;
//
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//
//	HAL_ADC_Start(&hadc1);
//
//	HAL_ADC_PollForConversion(&hadc1, 1);
//	uint16_t adc_result = HAL_ADC_GetValue(&hadc1);
//
//	HAL_ADC_Stop(&hadc1);
//
//	static const float IAA_VDC_TO_ADC_V = 3.3f / 5.0f;
//	static const float ADC_TO_LOADCELL = IAA_VDC_TO_ADC_V * (5.0f/5.095f) * 500.0f / 4095.0f;
//	return (float)adc_result * ADC_TO_LOADCELL;
//}

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
		delay_us(2);

		HAL_GPIO_WritePin(Pitch_Clock_GPIO_Port, Pitch_Clock_Pin, GPIO_PIN_SET);
		// for (int i = 0; i < 100; ++i) {} // Wait 10 us
		delay_us(2);

		pitch_data |= HAL_GPIO_ReadPin(Pitch_Data_GPIO_Port, Pitch_Data_Pin);
	}



  	return pitch_data;
}

//uint32_t ReadMastEncoder()
//{
//
//	uint32_t mast_data = 0;
//	for(int i = 0; i < 22; ++i)
//	{
//		mast_data <<= 1;
//
//		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
//		for (int i = 0; i < 20; ++i) {} // Wait 10 us
//		// delay_us(10);
//
//		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);
//		for (int i = 0; i < 20; ++i) {} // Wait 10 us
//		// delay_us(10);
//
//		mast_data |= HAL_GPIO_ReadPin(Mast_Data_GPIO_Port, Mast_Data_Pin);
//	}
//
//	return mast_data;
//
//	//HAL_GPIO_TogglePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin);
//	//return 0;
//}


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

	timer_50ms_flag = 0;
	timer_100ms_flag = 0;
	timer_500ms_flag = 0;

	flag_alive_led = 0;
	flag_uart_tx_send = 0;
	flag_acq_interval = 0;
	flag_wheel_rpm_process = 0;
	flag_rotor_rpm_process = 0;
	flag_can_tx_send = 0;

	b_rops = 0;

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
	/*if (ws_receive_flag)
	{
		static char frame_begin[] = "$IIMWV";

		// static int Led_Toggle=0;
		// HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,Led_Toggle);
		// Led_Toggle=!Led_Toggle;

		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);


		__disable_irq();

		ws_receive_flag = 0;
		index_buff = 0;

		static uint8_t ws_message[128] = {0};
		memcpy(ws_message, rx_buff, sizeof(ws_message));

		__enable_irq();

		// HAL_UART_Transmit(&huart2, ws_message, strlen(ws_message), HAL_MAX_DELAY);

		if (strlen((char*)ws_message) >= 6)
		{
			char begin_frame[7]={0};
			memcpy(begin_frame, ws_message, 6);
			if (0 == strcmp(begin_frame, frame_begin))
			{
				char wind_dir_msg[6] = {0};
				char wind_speed_msg[6] = {0};

				memcpy(wind_dir_msg, &ws_message[7], 5);
				memcpy(wind_speed_msg, &ws_message[15], 5);

				float wind_dir = atof(wind_dir_msg);
				float wind_speed = atof(wind_speed_msg);

				sensor_data.wind_direction = wind_dir;
				sensor_data.wind_speed = wind_speed;
			}
		}
	}
	*/
	ReadWeatherStation();

	/*
	static uint8_t rops_hack = 0;
	if (sensor_data.rotor_rpm >= 100)
	{
		// ACTIVATE ROPS
		rops_hack = 1;
	}

	static uint8_t first = 1;
	if (rops_hack && first)
	{
		first = 0;
		rops_hack = 0;

		uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
		TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
		delay_us(100);

		HAL_Delay(50);

		// int nb_steps = 12000;
		for (int i = 0; i < 120; ++i)
		{
			uint32_t pitch_mode = MOTOR_MODE_AUTOMATIC;
			TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode, 4, 0);
			delay_us(500);

			int nb_steps = 100;
			TransmitCAN(MARIO_PITCH_CMD, (uint8_t*)&nb_steps, 4, 0);
			delay_us(100);

			HAL_Delay(50);
		}

		pitch_done = 0;
		delay_us(100);

		while (1)
		{
		}

		return STATE_MOTOR_CONTROL;
	}
	*/


	if (flag_acq_interval)
	{
		flag_acq_interval = 0;

		// Read pitch and mast encoders
	sensor_data.pitch_encoder = ReadPitchEncoder();
		sensor_data.pitch_angle = CalcPitchAnglePales(false);
		// uint32_t pitch_val2 = ReadPitchEncoder2();

		sensor_data.mast_encoder = ReadMastEncoder();
		//TODO: (Marc) Mast Encoder
		sensor_data.mast_encoder = 0;

		// Read limit switches of mast
		sensor_data.limit1 = HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin);
		sensor_data.limit2 = HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin);

		// Read Torque + Loadcell adc
//#define ADC_RESOLUTION 65536.0f  // (16 bits)
//#define MAX_TORQUE 500.0f  // 500 N.m
//#define MAX_LOADCELL 400.0f  // 400 Lbs
		//static const float torque_adc_factor = MAX_TORQUE / ADC_RESOLUTION;
		//static const float loadcell_adc_factor = MAX_LOADCELL / ADC_RESOLUTION;
		//sensor_data.torque = ReadTorqueADC() * torque_adc_factor;
		// sensor_data.torque = 0.0f;


		// sensor_data.torque = ReadTorqueADC();
		// sensor_data.loadcell = ReadLoadcellADC();

		ReadTorqueLoadcellADC();
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

		// Process rpm counters
//#define ROTOR_CNT_PER_ROT 360.0f
//#define RPM_ROTOR_CNT_TIME_INVERSE 10.0f // Same as dividing by 100ms
		// static const float rotor_counter_to_rpm_constant = (RPM_ROTOR_CNT_TIME_INVERSE / ROTOR_CNT_PER_ROT) * 60.0f;


		// sensor_data.wheel_rpm = (float)wheel_rpm_counter * wheel_counter_to_rpm_constant;
		// sensor_data.wheel_rpm = (float)wheel_rpm_counter;
		// sensor_data.rotor_rpm = (float)rotor_rpm_counter * rotor_counter_to_rpm_constant;
		// sensor_data.rotor_rpm = (float)rotor_rpm_counter;


		ReadRotorRPM();
		// sensor_data.rotor_rpm = GetRotorRPM();
		// rotor_rpm_counter = 0;
		static int tsr_counter = 0;
		if (tsr_counter++ >= 20)
		{
			tsr_counter = 0;
			sensor_data.tsr = SimulateTSR();
		}
	}

	// Check ROPS
//	if (sensor_data.rotor_rpm >= ROTOR_RPM_ROPS)
//	{
//		// ACTIVATE ROPS
//		//b_rops = 1;
//		//return STATE_ROPS;
//	}

	return STATE_CHECK_ROPS;
}

uint32_t DoStateCheckROPS()
{
#define ROTOR_RPM_ROPS 1300
	if (sensor_data.rotor_rpm > ROTOR_RPM_ROPS)
	{
		b_rops = 1;
	}
	else
	{
		b_rops = 0;
	}

	if (b_rops)
		return STATE_ROPS;
	else
		return STATE_MOTOR_CONTROL;
}

uint32_t DoStateMotorControl()
{
#define AUTO_MAT 	0
#define AUTO_PITCH 	0

	if (AUTO_MAT == 1) {
		if (sensor_data.feedback_mast_mode != MOTOR_MODE_AUTOMATIC) {
			uint8_t mast_mode = MOTOR_MODE_AUTOMATIC;
			TransmitCAN(MARIO_MAST_MODE_CMD, &mast_mode, 4, 1);
		}
		DoMastControl();
	}
	else {
		if (sensor_data.feedback_mast_mode != MOTOR_MODE_MANUAL) {
			uint8_t mast_mode = MOTOR_MODE_MANUAL;
			TransmitCAN(MARIO_MAST_MODE_CMD, &mast_mode, 4, 1);
		}
		/*
		if (sensor_data.limit1 == 0 || sensor_data.limit2 == 0) {
			uint32_t dir_stop = 0x100;
			TransmitCAN(MARIO_MAST_MANUAL_CMD, (uint8_t*)&dir_stop, 4, 1);
		}
		*/
		/*
		uint32_t dir_left = MOTOR_DIRECTION_LEFT;
		uint32_t dir_right = MOTOR_DIRECTION_RIGHT;
		uint32_t dir_stop = MOTOR_DIRECTION_STOP;

		static uint32_t manual_motor_id = MARIO_MAST_MANUAL_CMD;

		if(GPIO_PIN_SET == HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin)) { // PD_14 -- PB2
			//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
			//HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			if (pb2_value == 1) {
				pb2_value = 0;
				// HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

				TransmitCAN(manual_motor_id, (uint8_t*)&dir_stop, 4, 1);
			}
		}
		if (GPIO_PIN_SET == HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin)) {// PD_15 -- PB1
			//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			//HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			if (pb1_value == 1) {
				pb1_value = 0;
				// HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

				TransmitCAN(manual_motor_id, (uint8_t*)&dir_stop, 4, 1);
			}
		}

		if (pb1_update) {
			pb1_update = 0;
			TransmitCAN(manual_motor_id, (uint8_t*)&dir_left, 4, 0);
		}
		if (pb2_update) {
			pb2_update = 0;
			TransmitCAN(manual_motor_id, (uint8_t*)&dir_right, 4, 0);
		}
		*/
	}



	if (AUTO_PITCH == 1)
	{
		if (sensor_data.feedback_pitch_mode != MOTOR_MODE_AUTOMATIC) {
			uint8_t pitch_mode = MOTOR_MODE_AUTOMATIC;
			TransmitCAN(MARIO_PITCH_MODE_CMD, &pitch_mode, 4, 1);
		}

		DoPitchControl();
	}
	else {
		if (sensor_data.feedback_mast_mode != MOTOR_MODE_MANUAL)
		{
			uint8_t mast_mode = MOTOR_MODE_MANUAL;
			TransmitCAN(MARIO_MAST_MODE_CMD, &mast_mode, 4, 1);
		}
	}


	return STATE_CAN;
}

uint32_t DoStateROPS()
{

	// Stay in ROPS
	while (b_rops)
	{
		delay_us(250);

		// Check for timer flags
		if (timer_50ms_flag)
		{
			timer_50ms_flag = 0;

			// flag_can_tx_send = 1;
			// flag_rpm_process = 1;
		}
		if (timer_100ms_flag)
		{
			timer_100ms_flag = 0;

			flag_acq_interval = 1;
			flag_wheel_rpm_process = 1;
			flag_rotor_rpm_process = 1;
			flag_can_tx_send = 1;
		}
		if (timer_500ms_flag)
		{
			timer_500ms_flag = 0;

			flag_alive_led = 1;
			flag_uart_tx_send = 1;
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

		if (std::fabs(sensor_data.pitch_encoder - PITCH_ABSOLUTE_ROPS) > MAX_ERROR_ROPS)
		{
			SendPitchROPSCmd();
		}

		SendPitchAngleCmd(90);
		DoStateAcquisition();
		DoStateMotorControl();

		DoStateCan();
		DoStateDataLogging();
		DoStateUartTx();

		DoStateCheckROPS();
	}

	// Exit ROPS state
	uint32_t rops_cmd = ROPS_DISABLE;
	TransmitCAN(MARIO_ROPS_CMD, (uint8_t*)&rops_cmd, 4, 0);
	delay_us(100);

	// uint32_t pitch_mode_cmd = MOTOR_MODE_MANUAL;
	// TransmitCAN(MARIO_PITCH_MODE_CMD, (uint8_t*)&pitch_mode_cmd, 4, 0);
	// delay_us(100);

	return STATE_ACQUISITION;
}

uint32_t DoStateCan()
{
	static float temp = 0.10f;

	// DEBUG DEBUG -- CAN Volant

	if (1) // flag_can_tx_send Sent every 100ms
	{
		// temp += 0.10f;

		flag_can_tx_send = 0;
		/*
		static uint8_t uint_buffer_test[] = { 111, 112, 113, 114, 115, 116, 117, 118, 119, 210 };
		static float float_buffer_test[] = { 9.10f, 9.20f, 9.30f, 9.40f, 9.50f, 9.60f, 9.70f, 9.80f, 9.90f };
		float_buffer_test[0] += temp;
		float_buffer_test[1] += temp;
		float_buffer_test[2] += temp;

		static int rising = 1;
		if (rising)
		{
			float_buffer_test[3] += temp;
			if (float_buffer_test[3] > 100.0f)
				rising = false;
		}
		else //falling
		{
			float_buffer_test[3] -= temp;
			if (float_buffer_test[3] < 10.0f)
				rising = true;
		}

		float_buffer_test[4] += temp;
		float_buffer_test[5] += temp;
		float_buffer_test[6] += temp;
		float_buffer_test[7] += temp;
		float_buffer_test[8] += temp;

		uint8_t uint_buffer_index = 0;
		uint8_t float_buffer_index = 0;
		*/
		static float dec_test = 0.0f;
		dec_test += 0.0001f;
		if (dec_test >= 0.009f)
			dec_test = 0.0001f;

		static float tsr_refresh = 10;
		tsr_refresh += 10;
		if (tsr_refresh >= 100)
			tsr_refresh = 10;

		static uint32_t can_tx_state = 0;
		if (can_tx_state == 0) {
				float pitch_raw = CalcTSR() + dec_test;
				TransmitCAN(MARIO_MAST_ANGLE, (uint8_t*)&pitch_raw, 4, 0);
				can_tx_state++;
		} else if (can_tx_state == 1) {
				float wind_spd = (float)sensor_data.wind_speed + dec_test;
				TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&wind_spd, 4, 0);
				can_tx_state++;
		} else if (can_tx_state == 2) {
				float pitch_enc = (float)sensor_data.pitch_angle + dec_test;
				TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&pitch_enc, 4, 0);
				can_tx_state++;
		} else if (can_tx_state == 3) {
				float rpm_raw = (float)sensor_data.rotor_rpm + dec_test;
				TransmitCAN(MARIO_ROTOR_RPM, (uint8_t*)&rpm_raw, 4, 0);
				can_tx_state++;
		} else if (can_tx_state == 4) {
				float pitch2 = (float)sensor_data.wheel_rpm + dec_test;
				TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&pitch2, 4, 0);
				can_tx_state++;
		} else if (can_tx_state == 5) {
				float wind_dir = (float)sensor_data.wind_direction + dec_test;
				TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&wind_dir, 4, 0);
				can_tx_state = 0;
		}

		//delay_ms(2);

		// TransmitCAN(MARIO_WIND_SPEED, (uint8_t*)&float_buffer_test[4], 4, 0);

		//delay_ms(2);

		// TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&(float_buffer_test[0]), 4, 0);
		// TransmitCAN(MARIO_PITCH_ANGLE, (uint8_t*)&sensor_data.pitch_angle, 4, 0);
		/*
		static float tsr_refresh_1000 = 1000;
		tsr_refresh_1000 += 1000;
		if (tsr_refresh_1000 >= 10000)
			tsr_refresh_1000 = 1000;
		if (sensor_data.pitch_angle < 0) {
			tsr_refresh_1000 = -tsr_refresh_1000;
		}
		*/
		// float pitch_enc = (float)sensor_data.rotor_rpm + dec_test;

		//delay_us(100);
		//delay_ms(2);

		// sensor_data.pitch_encoder = 2401;
		// float rpm_test = (float)sensor_data.pitch_encoder + dec_test;

		//delay_us(100);
		//delay_ms(2);

		// sensor_data.wheel_rpm = float_buffer_test[2];
		//TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&sensor_data.wheel_rpm, 4, 0);
		// float pitch2 = (float)sensor_data.pitch_angle * (float)sensor_data.wheel_rpm;
		// float pitch2 = ((float)(sensor_data.wheel_rpm)+1.0f) * 2.0f;

		// TransmitCAN(MARIO_WHEEL_RPM, (uint8_t*)&float_buffer_test[2], 4, 0);
		//delay_us(100);
		//delay_ms(2);

		// TransmitCAN(MARIO_WIND_DIRECTION, (uint8_t*)&float_buffer_test[3], 4, 0);

		//delay_us(100);
		//delay_ms(2);

		// delay_ms(10);


		//TransmitCAN(MARIO_TORQUE, (uint8_t*)&float_buffer_test[5], 4, 0);
		//delay_us(5);

		//TransmitCAN(MARIO_LOADCELL, (uint8_t*)&float_buffer_test[6], 4, 0);
		//delay_us(5);

		// TODO: (Marc) Batt voltage + Batt current
		// TODO: (Marc) Limit switch

		//static const uint8_t mode_test_manual = MOTOR_MODE_MANUAL;
		//static const uint8_t mode_test_automatic = MOTOR_MODE_AUTOMATIC;

		//TransmitCAN(MARIO_PITCH_MODE_FEEDBACK, (uint8_t*)&mode_test_manual, 4, 0);
		//delay_us(5);

		//TransmitCAN(MARIO_MAST_MODE_FEEDBACK, (uint8_t*)&mode_test_automatic, 4, 0);
		//delay_us(5);

		// static const uint8_t rops_test = ROPS_ENABLE;
		//static const uint8_t rops_test = 1;
		//TransmitCAN(MARIO_ROPS_FEEDBACK, (uint8_t*)&rops_test, 4, 0);
		//delay_us(5);

		// Also send the turbine rpm value to the drive motor for ROPS detection
		// TransmitCAN(MARIO_MOTOR_ROTOR_RPM, (uint8_t*)&sensor_data.rotor_rpm, 4, 0);
		// delay_us(50);

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
	delay_us(200);
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
	delay_us(10);

#define KNOTS_TO_MS 0.514444f
	float wind_speed_ms = KNOTS_TO_MS * sensor_data.wind_speed;
	TransmitDataAcq(2, (int*)&wind_speed_ms);
	delay_us(10);

	float wind_dir_adj = sensor_data.wind_direction - 120.0f;
	TransmitDataAcq(3, (int*)&wind_dir_adj);
	delay_us(10);

	TransmitDataAcq(4, (int*)&sensor_data.rotor_rpm);
	delay_us(10);

	TransmitDataAcq(5, (int*)&sensor_data.wheel_rpm);
	delay_us(10);

	float tsr2 = CalcTSR();
	TransmitDataAcq(6, (int*)&tsr2);
	delay_us(10);

	TransmitDataAcq(7, (int*)&sensor_data.torque);
	delay_us(10);

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
	delay_us(200);
}

uint32_t DoStateUartTx()
{
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



		delay_us(200);

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


/* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if (huart->Instance == UART5)
//  {
//    // Transmit one byte with 100 ms timeout
//    // HAL_UART_Transmit(&huart5, &byte, 1, 100);
//	//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//
//	/*
//	static char buf[128];
//	static uint8_t index_buf = 0;
//	buf[index_buf++] = ws_rx_byte;
//	if (index_buf == 30)
// 	{
//		  buf[index_buf] = 0;
//		  index_buf = 0;
//		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
//	}
//	*/
//
//	//HAL_UART_Transmit(&huart2, &ws_rx_byte, sizeof(ws_rx_byte), HAL_MAX_DELAY);
//
//	// static char test_buffer[128] = {0};
//	// static uint8_t test_index = 0;
//
//	// rx_buff[index_buff] = ws_rx_byte;
//	//index_buff++;
//
//	rx_buff[index_buff++] = ws_rx_byte[0];
//	if(ws_rx_byte[0] == '\n')
//	{
//		rx_buff[index_buff] = '\0';
//		index_buff = 0;
//		ws_receive_flag = 1;
//		// HAL_UART_Transmit(&huart2, test_buffer, strlen(test_buffer), HAL_MAX_DELAY);
//	}
//
//
//    // Restart interrupt for next byte
//	HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart5, &ws_rx_byte[0], 1);
//	if (ret != HAL_OK)
//	{
//		// Do something to reset the UART ?
//		// HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
//		if (ret == HAL_BUSY)
//		{
//			// Already waiting for bytes ??
//			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
//		}
//	}
//
//	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
//	// HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
//  }
//}

/*
void ProcessCanMessage()
{
	typedef union BytesToType_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
		uint32_t uint_val;
		float float_val;
	} BytesToType;
	static BytesToType bytesToType;


	// Indicate CAN working with CAN led
	HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);

	// Technically CAN data can be 8 bytes but we only send 4-bytes data to the motor driver
	// uint32_t upper_can_data = rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24);
	uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16) | (rxData[3] << 24);

	if (pRxHeader.StdId == DRIVEMOTOR_PITCH_MODE_FEEDBACK)
	{
		sensor_data.feedback_pitch_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_MAST_MODE_FEEDBACK)
	{
		sensor_data.feedback_mast_mode = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_PITCH_DONE)
	{
		pitch_done = 1;
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_PITCH_FAULT_STALL)
	{
		sensor_data.pitch_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_MAST_FAULT_STALL)
	{
		sensor_data.mast_fault_stall = (can_data & 0xFF);
	}
	else if (pRxHeader.StdId == BACKPLANE_TOTAL_VOLTAGE)
	{

	}
	else if (pRxHeader.StdId == BACKPLANE_TOTAL_CURRENT)
	{

	}
	else if (pRxHeader.StdId == VOLANT_MANUAL_ROPS_CMD)
	{
		uint8_t rops_data = (can_data & 0xFF);
		if (rops_data == ROPS_ENABLE)
			b_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			b_rops = 0;
		else
		{
			// Unknown value received for ROPS command
			// Assume it was meant to activate ROPS
			b_rops = 1;
		}
	}
	else if (pRxHeader.StdId == DRIVEMOTOR_ROPS_FEEDBACK)
	{
		uint8_t rops_data = (can_data & 0xFF);
		sensor_data.feedback_pitch_rops = rops_data;

//		if (rops_data == ROPS_ENABLE)
//			sensor_data.feedback_pitch_rops = 1;
//		else if (rops_data == ROPS_DISABLE)
//			sensor_data.feedback_pitch_rops = 0;
//		else
//			sensor_data.feedback_pitch_rops = rops_data;

	}
	else if (pRxHeader.StdId == BACKPLANE_DUMMY_TRAFFIC_MARIO)
	{
		// Dummy traffic
	}
	else
	{
		// Unknown CAN ID
	}
}
*/

//void CAN_ReceiveFifoCallback(CAN_HandleTypeDef* hcan, uint32_t fifo)
//{
//	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);
//	for (int i = 0; i < num_messages; ++i)
//	{
//		if (HAL_CAN_GetRxMessage(hcan, fifo, &pRxHeader, rxData) != HAL_OK)
//		{
//			Error_Handler();
//		}
//
//		ProcessCanMessage();
//	}
//}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
//{
//	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
//	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO0);
//}
//
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
//{
//	HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
//	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO1);
//}
//
//void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
//{
//	// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
//	HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
//}

/*
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	typedef union RxToInt_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
	} RxToInt;
	static RxToInt rxToInt;

	if (hcan->Instance == CAN1)
	{
		// Indicate CAN working with CAN led
		HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);

		if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, rxData) != HAL_OK)
		{
			Error_Handler();
		}

		if (pRxHeader.StdId == MARIO_PITCH_MODE_CMD)
		{
			uint8_t pitch_mode = rxData[0];
		}
		else if (pRxHeader.StdId == MARIO_MAST_MODE_CMD)
		{
			uint8_t mast_mode = rxData[0];
		}
		else
		{
			// Unknown CAN ID
		}
	}
	else
	{
		// Wrong CAN interface
	}
}
*/

//HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority)
//{
//	// CAN_TxHeaderTypeDef msg;
//	pTxHeader.StdId = id;
//	pTxHeader.IDE = CAN_ID_STD;
//	pTxHeader.RTR = CAN_RTR_DATA;
//	pTxHeader.DLC = size; // Number of bytes to send
//	pTxHeader.TransmitGlobalTime = DISABLE;
//
//	uint8_t found_mailbox = 0;
//	for (int i = 0; i < 25; ++i)
//	{
//		// Check that mailbox is available for tx
//		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
//		{
//			found_mailbox = 1;
//			break;
//		}
//		// Otherwise wait until free mailbox
//		// for (int j = 0; j < 500; ++j) {}
//		delay_us(50);
//	}
//	if (!found_mailbox)
//	{
//		// HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
//		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
//	}
//
//	if (with_priority)
//	{
//		// If message is important, make sure no other messages are queud to ensure it will be sent after any other
//		// values that could override it.
//		for (int i = 0; i < 10; ++i)
//		{
//			// Check that all 3 mailboxes are empty
//			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3)
//				break;
//			// Otherwise wait until 3 free mailbox
//			// for (int j = 0; j < 500; ++j) {}
//			delay_us(50);
//		}
//	}
//
//	uint32_t mb;
//	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
//	if (ret != HAL_OK)
//	{
//		HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_SET);
//		return ret;
//	}
//
//	// Successful transmit
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//
//	// Update the error led if had a successful can write
//	HAL_GPIO_WritePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin, GPIO_PIN_RESET);
//	// ToggleLed(LED_CAN);
//	return ret;
//}


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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
/*
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  */

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

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 480;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
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
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pin = LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN 4 */
// EXTI Line External Interrupt ISR Handler CallBack

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_14) // PD_14 -- PB2
    {
    	//HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    	pb2_value = 1;

    	pb2_update = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_15) // PD_15 -- PB1
    {
    	//HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    	// HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    	pb1_value = 1;

    	pb1_update = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_0) // Rotor RPM
    {
    	rotor_rpm_counter++;
    }
    else if (GPIO_Pin == GPIO_PIN_1) // Wheel RPM
    {
    	wheel_rpm_counter++;
    }
    else if (GPIO_Pin == GPIO_PIN_3) // Limit 1
    {
    	sensor_data.limit1 = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_4) // Limit 2
    {
    	sensor_data.limit2 = 1;
    }

}
*/
/* USER CODE END 4 */

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

