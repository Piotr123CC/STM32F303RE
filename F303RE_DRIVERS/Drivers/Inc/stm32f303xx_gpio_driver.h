/*
 * stm32f303xx_gpio_driver.h
 *
 *  Created on: Oct 7, 2022
 *      Author: piotr
 */

#ifndef INC_STM32F303XX_GPIO_DRIVER_H_
#define INC_STM32F303XX_GPIO_DRIVER_H_

#include "stm32f303xx.h"


typedef struct
{
	uint8_t GPIO_PinNumber;				/*<possible values form @GPIO_PIN_NO		>*/
	uint8_t GPIO_PinMode;     			/*<possible values form @GPIO_PIN_MODES  >*/
	uint8_t GPIO_PinSpeed;	 			/*<possible values form @GPIO_PIN_SPEED	>*/
	uint8_t GPIO_PinPuPdControl; 		/*<Possible values form @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType; 			/*< possible values form @GPIO_PIN_OTYPER >*/
	uint8_t GPIO_PinAltFunMode;

}GPIO_Config_t;


/*Handle structure*/

typedef struct
{
	//pointer to hold the base address of the GPIO pheripheral
	GPIO_RegDef_t *pGPIOx; 	/* GPIOA, GPIOB.... GPIOx*
								   * This holds the base address of the GPIO port to which the pin belongs
								   */
	GPIO_Config_t GPIO_PinConfig; /*This hold GPIO pin configuration*/


}GPIO_Handle_t;

/*Pheripheral Clock setup*/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*init and DeInit functions*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data read/write functions*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*IRQ Configuration and ISR handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQhandling(uint8_t PinNumber);


/*
 *@GPIO_PIN_NO numbers
 */
#define GPIO_PIN_NO_0  			0
#define GPIO_PIN_NO_1  			1
#define GPIO_PIN_NO_2  			2
#define GPIO_PIN_NO_3  			3
#define GPIO_PIN_NO_4  			4
#define GPIO_PIN_NO_5  			5
#define GPIO_PIN_NO_6  			6
#define GPIO_PIN_NO_7  			7
#define GPIO_PIN_NO_8  			8
#define GPIO_PIN_NO_9  			9
#define GPIO_PIN_NO_10  		10
#define GPIO_PIN_NO_11  		11
#define GPIO_PIN_NO_12  		12
#define GPIO_PIN_NO_13  		13
#define GPIO_PIN_NO_14  		14
#define GPIO_PIN_NO_15  		15

/*
 * @GPIO_PIN_MODES
 * Macros GPIO pin modes
 */

#define GPOIO_INPUT 			0
#define GPOIO_OUTPUT 			1
#define GPOIO_AF 				2
#define GPOIO_ANALOG 			3
#define GPOIO_IT_FT				4
#define GPOIO_IT_RT 			5
#define GPOIO_IT_RFT 			6


/*@GPIO_PIN_OTYPER
 * GPIO pin possible output types
 */
#define GPOIO_PP 				0   //Push-pull
#define GPOIO_OP 				1	//Open-drain

/*@GPIO_PIN_PUPD
 * GPIO pin pull down/up configuration macros
 */

#define GPOIO_NO_PUPD			0
#define GPOIO_PU				1
#define GPOIO_PD				2


/*@GPIO_PIN_SPEED
 * GPIO pin speed configuration macros
 */

#define GPOIO_LOW_SPEED			0
#define GPOIO_MEDIUM_SPEED		1
#define GPOIO_HIGH_SPEED		3



#endif /* INC_STM32F303XX_GPIO_DRIVER_H_ */
