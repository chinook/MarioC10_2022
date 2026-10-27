/*
 * telemetry.h
 *
 *  Created on: Aug 16, 2025
 *      Author: thoma
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include "main.h"

void SdLog_WriteRow(const SensorData *d);
void FloatToString(float value, int decimal_precision, unsigned char *val);
void UartTxAcquisition();
uint32_t DoStateUartTx();

#endif /* INC_TELEMETRY_H_ */
