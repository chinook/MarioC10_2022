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

extern uint8_t status_button_hgg;
extern uint8_t status_button_hg;
extern uint8_t status_button_hd;
extern uint8_t status_button_hdd;
extern uint8_t status_button_mg;
extern uint8_t status_button_md;
extern uint8_t status_button_bgg;
extern uint8_t status_button_bg;
extern uint8_t status_button_bd;
extern uint8_t status_button_bdd;


HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority);

void MX_CAN1_Init(void);

#endif /* _CAN_H_ */
