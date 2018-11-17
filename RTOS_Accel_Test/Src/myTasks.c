/*
 * myTasks.c
 *
 *  Created on: Sep 9, 2018
 *      Author: quant
 */

#include "myTasks.h"
#include "arm_math.h"
#include "stdio.h"
#include "stdlib.h"


extern SPI_HandleTypeDef hspi1;

extern UART_HandleTypeDef huart3;



void floatbuff(char*,float,unsigned char);


void StartAccelTask(void const * argument)
{
	uint8_t registerAddress, data;
		uint8_t osaXH,  osaYH, osaZH  ;//,osaYL,osaXL,osaZL; //Variables for AXES
		struct AccelRawData * rawData; //Variables for puting LOW and HIGH byte together - but I read HIGHs only anyway

		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET); //Chip select
		registerAddress = 0x20; //for CTRL_REG4
		HAL_SPI_Transmit(&hspi1, &registerAddress, 1, 50);
		data = 0x1F; //axis enable, dont update until you get new values, 3,125Hz output
		HAL_SPI_Transmit(&hspi1, &data, 1, 50);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET); //Chip select
		HAL_Delay(50);


		for(;;){


			vTaskDelay(pdMS_TO_TICKS(50) );
			//HAL_UART_Transmit(&huart3,"Accel\r\n",7,100);
			//assert()
			//vTaskSuspendAll();
			//taskDISABLE_INTERRUPTS();
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			registerAddress = 0x29+0x80;
			HAL_SPI_Transmit(&hspi1, &registerAddress, 1, 50);
			HAL_SPI_Receive(&hspi1, &osaXH, 1, 200);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_Delay(10);

			//READING HIGH BYTE OF Y AXIS
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			registerAddress = 0x2B+0x80;
			HAL_SPI_Transmit(&hspi1, &registerAddress, 1, 50);
			HAL_SPI_Receive(&hspi1, &osaYH, 1, 200);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_Delay(10);


			//THIS ONE SHOULD NOT BE COMENTED, BUT WHEN I UNCOMENT IT SO IT READS VALUE IT DOES NOT WORK
			//IT DOES NOT MATTER IF IT IS THIS Z AXIS OR ANY OTHER ONE
			//READING HIGH BYTE OF Z AXIS
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			registerAddress = 0x2D+0x80;
			HAL_SPI_Transmit(&hspi1, &registerAddress, 1, 50);
			HAL_SPI_Receive(&hspi1, &osaZH, 1, 200);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);


			HAL_Delay(10);
			//taskENABLE_INTERRUPTS();
			//xTaskResumeAll();
			//LCD_clear();

			//Convert to 16 bit int
			AccelMsg.osaX = (int16_t)osaXH <<8;  //| osaXL;
			AccelMsg.osaY = (int16_t)osaYH <<8;  //| osaYL;
			AccelMsg.osaZ = (int16_t)osaZH <<8;  //| osaZL;

			rawData = &AccelMsg;
			xQueueSend( AccelDataHandle,&rawData, 10 );


		}
}



/* StartSerialTask function */
void StartSerialTask(void const * argument)
{
			struct AccelRawData* rawData ;
			float accelerationX, accelerationY, accelerationZ; //Result of calculation in g
			//Char array for being able to print them on LCD
			char toPrintY[7] = {0};
			char toPrintX[7] = {0};
			char toPrintZ[7] = {0};
				

			for(;;){




				vTaskDelay(pdMS_TO_TICKS(100) );
				while(!xQueueReceive( AccelDataHandle, &rawData, 5))
						;



				accelerationX = ((2.0/65535.0) * rawData->osaX)*2;
				accelerationY = ((2.0/65535.0) * rawData->osaY)*2;
				accelerationZ = ((2.0/65535.0) * rawData->osaZ)*2;

				
				floatbuff(toPrintX,accelerationX, 3);
				floatbuff(toPrintY,accelerationY, 3);
				floatbuff(toPrintZ,accelerationZ, 3);



				//Printing X value
				HAL_UART_Transmit(&huart3,"X:",2,100);
				HAL_UART_Transmit(&huart3,toPrintX,6,100);
				HAL_UART_Transmit(&huart3,"g ",2,100);

				//Printing Y value

				HAL_UART_Transmit(&huart3,"Y:",2,100);
				HAL_UART_Transmit(&huart3,toPrintY,6,100);
				HAL_UART_Transmit(&huart3,"g ",2,100);


				//Printing Z value
				HAL_UART_Transmit(&huart3,"Z:",2,100);
				HAL_UART_Transmit(&huart3,toPrintZ,6,100);
				HAL_UART_Transmit(&huart3,"g\r\n",3,100);
			}
}


void floatbuff(char* dest,float Fval,uint8_t precision){
	uint8_t destlen = precision + 2 + 1; 
	float src = Fval;
	uint8_t pos = 0;
	float dec_pos = 10;
	/*Take car of negative*/
	if(src < 0){ 
		dest[pos++] ='-';
		src *=-1;
	}
	
	while(pos < destlen){
		float src_tmp = 0;
		if(src >= dec_pos){
			src_tmp = src / dec_pos;
			src -= (unsigned char)src_tmp*dec_pos;
		}

		/*convert digit to char*/
		dest[pos++] = 48 + (unsigned char) src_tmp;		
			
		if(dec_pos == 1) dest[pos++] = '.';

		/*compare next decimal place*/
		dec_pos /=10;
		
		
	}
		
}
