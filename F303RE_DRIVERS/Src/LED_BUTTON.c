/*
 * LED_BUTTON.c
 *
 *  Created on: 10 paź 2022
 *      Author: piotr
 */

#include "stm32f303xx.h"

void shittyDelay(void)
{
	for(uint32_t i =0;i<500000/2;i++);
}
int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;





	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPOIO_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPOIO_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPOIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPOIO_INPUT;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPOIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == RESET)
		{
//			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, RESET);
			shittyDelay();
			GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
		}
//
//		else
//		{
//			GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_5, SET);
//		}
	}


	return 0;
}
