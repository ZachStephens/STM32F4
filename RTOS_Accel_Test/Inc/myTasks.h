/*
 * myTasks.h
 *
 *  Created on: Sep 9, 2018
 *      Author: quant
 */

#ifndef MYTASKS_H_
#define MYTASKS_H_
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

osThreadId defaultTaskHandle;
osThreadId AccelTaskHandle;
osThreadId SerialTaskHandle;
osMessageQId AccelDataHandle;

void StartDefaultTask(void const * argument);
void StartAccelTask(void const * argument);
void StartSerialTask(void const * argument);


//volatile unsigned long x = 0, y = 0;

struct AccelRawData{
	 int16_t osaX;
	 int16_t osaY;
	 int16_t osaZ;
}AccelMsg;



#endif /* MYTASKS_H_ */
