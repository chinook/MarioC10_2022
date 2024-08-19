/*
 * can.h
 *
 *  Created on: 15 juin 2023
 *      Author: Marc
 */

#ifndef _CAN_H_
#define _CAN_H_

#include "stm32f4xx_hal.h"
#include "chinook_can_ids.h"

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority);

void MX_CAN1_Init(void);

#endif /* _CAN_H_ */
