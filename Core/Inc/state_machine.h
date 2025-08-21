/*
 * state_machine.h
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "stm32f4xx_hal.h"

enum STATES {
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

extern uint8_t flag_telemetry;
extern uint8_t flag_uart_tx_send;

void ExecuteStateMachine();

void delay_us(uint16_t delay16_us);

#endif /* INC_STATE_MACHINE_H_ */
