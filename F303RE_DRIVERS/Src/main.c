/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */



#include "stm32f303xx.h"

void shittyDelay(void)
{
	for(uint32_t i =0;i<500000/2;i++);
}
int main(void)
{


	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPOIO_INPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPOIO_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPOIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);





	while (1)
	{
//		GPIO_TogglePin(GPIOA,GPIO_PIN_NO_5);

		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, SET);
		shittyDelay();
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, RESET);
		shittyDelay();
	}
   return 0;
}
