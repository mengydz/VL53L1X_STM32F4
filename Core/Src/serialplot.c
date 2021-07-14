/*
 * serialplot.c
 *
 *  Created on: 2021年7月14日
 *      Author: 21954
 */

#include "serialplot.h"
#include "usart.h"

SerialPlotFrame frame={
	.frameHeader1 = 0xAA,
	.frameHeader2 = 0xBB,
};

void SerialPlotFramePlotWord(int16_t fdata,int16_t fdata2)
{
	frame.fdata[0] = fdata;
	frame.fdata[1] = fdata2;
	HAL_UART_Transmit(&huart1, (uint8_t *)&frame, sizeof(SerialPlotFrame), 0xFFFF);
}
