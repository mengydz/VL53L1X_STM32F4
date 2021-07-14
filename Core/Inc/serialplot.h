/*
 * serialplot.h
 *
 *  Created on: 2021年7月14日
 *      Author: 21954
 */

#ifndef INC_SERIALPLOT_H_
#define INC_SERIALPLOT_H_

#ifdef __cplusplus
	extern "C"{
#endif

#include "stm32f4xx_hal.h"

typedef struct{
	uint8_t frameHeader1;
	uint8_t frameHeader2;
	int16_t	fdata[2];
}__attribute__((packed))SerialPlotFrame;

extern SerialPlotFrame frame;

void SerialPlotFramePlotWord(int16_t fdata,int16_t fdata2);

#ifdef __cplusplus
	}
#endif



#endif /* INC_SERIALPLOT_H_ */
