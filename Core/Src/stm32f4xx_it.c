/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint8_t timer_1ms_flag = 0;
uint8_t timer_50ms_flag = 0;
uint8_t timer_100ms_flag = 0;
uint8_t timer_250ms_flag = 0;
uint8_t timer_500ms_flag = 0;

uint8_t timer_50ms_counter = 0;
uint8_t timer_100ms_counter = 0;
uint8_t timer_250ms_counter = 0;
uint16_t timer_500ms_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void MotorMastSpeedDir(int8_t speed_dir);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Rotor_RPM_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  rotor_rpm_counter++;

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Wheel_RPM_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

	wheel_rpm_counter++;

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(LIMIT1_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(LIMIT2_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles ADC1 global interrupt.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

	flag_IT_adc1_loadcell_torque = 1;

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

	timer_1ms_flag = 1;
	if (timer_50ms_counter > 50) {
		timer_50ms_counter = 0;
		timer_50ms_flag = 1;
	} else {
		timer_50ms_counter++;
	}

	if (timer_100ms_counter > 100) {
		timer_100ms_counter = 0;
		timer_100ms_flag = 1;
	} else {
		timer_100ms_counter++;
	}

	if (timer_250ms_counter > 250) {
		timer_250ms_counter = 0;
		timer_250ms_flag = 1;
	} else {
		timer_250ms_counter++;
	}

	if (timer_500ms_counter > 500) {
		timer_500ms_counter = 0;
		timer_500ms_flag = 1;
	} else {
		timer_500ms_counter++;
	}

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

	//ad36_reading_encoder_IT();

	// sensor_data.wheel_rpm = (((float)wheel_rpm_counter / 0.2f)/48.0f)*60.0f;
	// sensor_data.rotor_rpm = (((float)rotor_rpm_counter/ 0.2f)/360.0f)*60.0f;

	//wheel_rpm_counter = 0;
	//rotor_rpm_counter = 0;

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

	ReadWheelRPM();

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PB2_Pin);
  HAL_GPIO_EXTI_IRQHandler(PB1_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

    	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    		if ((UART5->SR & UART_IT_RXNE)) {
    			if (index_buff >= 128)
    				index_buff = 0;
    			uint8_t rbyte = huart5.Instance->DR;
    			rx_buff[index_buff] = rbyte;
    			index_buff++;
    			if(rbyte == '$')
    			{
    				// HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    				ws_receive_flag = 1;
    			}


    			// HAL_UART_EnableIt
    			//_HAL_UART_SEND_REQ(&huart5, UART_RXDATA_FLUSH_REQUEST);
    			__HAL_UART_ENABLE_IT(&huart5,UART_IT_RXNE);
    		}
    		if (UART5->SR & USART_SR_ORE)
    		{
    			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    		  //__HAL_UART_SEND_REQ(&huart4, UART_RXDATA_FLUSH_REQUEST);
    		  //__HAL_UART_ENABLE_IT(&huart4,UART_IT_ORE);
    		}

  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

	//DriveMotorMast();

	/*
	 static uint32_t counter_test_speed = 0;
	 static int8_t test_speed = 0;
	 if (counter_test_speed > 10000) {
	 counter_test_speed = 0;
	 MotorMastSpeedDir(test_speed);


	 test_speed++;
	 } else {
	 counter_test_speed++;
	 }*/

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
uint8_t speed_raw = 25;
uint8_t counter = 0;
void DriveMotorMast() {
	if (counter >= 50) {
		counter = 0;

		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	} else {
		counter++;
	}

	if (counter > speed_raw) {
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	}
}

//converti -100 à 100 vers les valeurs de voltage analogique (1V6 à 3V3)
void MotorMastSpeedDir(int8_t speed_dir) {
	if (speed_dir > 100) {
		speed_dir = 100;
	}
	if (speed_dir < -100) {
		speed_dir = -100;
	}

	if (speed_dir < 0) {
		HAL_GPIO_WritePin(mast_dir_GPIO_Port, mast_dir_Pin, GPIO_PIN_RESET);
		speed_dir = -speed_dir;  // Flip the sign instead of adding 100
	} else {
		HAL_GPIO_WritePin(mast_dir_GPIO_Port, mast_dir_Pin, GPIO_PIN_SET);
	}

	if (speed_dir != 0) {
		// Map from 0–100 to 25–50
		speed_raw = (speed_dir * 25) / 100 + 25;
		HAL_GPIO_WritePin(mast_speed_GPIO_Port, mast_speed_Pin, GPIO_PIN_SET);
	} else {
		speed_raw = 0;
		HAL_GPIO_WritePin(mast_speed_GPIO_Port, mast_speed_Pin, GPIO_PIN_RESET);
	}

}

/*
int8_t i = 0;
uint8_t clock = 0;
uint8_t sample = 0;
uint16_t encoder_raw_data = 0;
//called each 1/2us
void ad36_reading_encoder_IT() {
	if (i > 12 * 2) {
		encoder_raw_data = 0;
		i = -6; //us waiting time / 2
		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);
	} else if (i == 0) {
		i++;
		HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);
	} else if (i > 0) {
		if (sample == 1) {
			sample = 0;
			encoder_raw_data <<= 1;
			if (HAL_GPIO_ReadPin(Mast_Data_GPIO_Port, Mast_Data_Pin)) {
				encoder_raw_data |= 1;
			}
		}

		if ((i % 2) == 0) { //i == 2, 4, 6, ...
			i++;

			if (clock == 0) {
				clock = 1;
				sample = 1;
				HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_SET);

			} else {
				clock = 0;
				HAL_GPIO_WritePin(Mast_Clock_GPIO_Port, Mast_Clock_Pin, GPIO_PIN_RESET);

			}

		}
	}
}
*/

/* USER CODE END 1 */

