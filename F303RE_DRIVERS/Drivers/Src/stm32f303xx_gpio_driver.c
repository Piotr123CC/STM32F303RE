/*
 * stm32f303xx_gpio_driver.c
 *
 *  Created on: Oct 7, 2022
 *      Author: piotr
 */


#include "stm32f303xx_gpio_driver.h"

/*Pheripheral Clock setup*/
/**
 * @fn GPIO_PeriClockControl
 * @brief  -This function  Enables of disables pheripheral clock for the given GPIO port
 *
 * @param pGPIOx ---  Base address of the gpio pheripheral
 * @param ENABLE or DISABLE macros
 * @return none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

		else
		{
			if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}

		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}

		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}

		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}

		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}

		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		}
	}
}

/*init and DeInit functions*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{


	uint32_t tmp=0;
	//1. MODE configuartion
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPOIO_ANALOG )
	{
		tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ); // clearing bit
		pGPIOHandle->pGPIOx->MODER |= tmp; 		// setting bit
	}

	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPOIO_IT_FT)
		{
			EXTI->FTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPOIO_IT_RT)
		{
			EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPOIO_IT_RFT)
		{
			EXTI->RTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR1 |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}


		//configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG1->EXTICR[temp1] = (portcode << (temp2 * 4));
		// enable the exti interrupt delivery using IMR
		EXTI->IMR1 |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	tmp = 0;
	//2. SPEED configuartion
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OSPEEDR |= tmp;

	tmp =0;
	//3. PULL UP/DOWN configuartion
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->PUPDR |= tmp;

	tmp = 0;
	//4. OUTPUT TYPE configuartion
	tmp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER |= tmp;

	tmp =0;
	//5. ALTERNATE FUNCTION configuartion
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPOIO_AF)
	{
		uint8_t tmp1;

		tmp1 =  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		if (tmp1 == 1)
		{
			pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			pGPIOHandle->pGPIOx->AFR[1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * tmp1));
		}
		else
		{
			pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			pGPIOHandle->pGPIOx->AFR[0] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * tmp1));
		}
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}

	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}

	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}

	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}

	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}

	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}


/**
 * @fn GPIO_ReadFromInputPin
 * @brief
 *
 * @param pGPIOx  	-	GPIO_ port  A, B,...
 * @param PinNumber - 	GPIO_PIN number 0,1,... 15
 * @return - 			0 or 1
 */
/*Data read/write functions*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/**
 * @fn uint16_t GPIO_ReadFromInputPort
 * @brief
 *
 * @param pGPIOx - 		GPIO_ port  A, B,...
 * @return
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/**
 * @fn void GPIO_WriteToOutputPin
 * @brief
 *
 * @param pGPIOx 	- 	GPIO_ port  A, B,...
 * @param PinNumber -	GPIO_PIN number 0,1,... 15
 * @param Value		-   GPIO_PIN_SET or GPIO_PIN_RESET
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= 1<<PinNumber ;
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber) ;
	}
}


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value ;
}

/**
 * @fn void GPIO_TogglePin
 * @brief 				-  This function changes state of pin with every call
 *
*@param pGPIOx 			- 	GPIO_ port  A, B,...
 * @param PinNumber 	-	GPIO_PIN number 0,1,... 15
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= 1<<PinNumber;
}

/*IRQ Configuration and ISR handling*/
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <= 64)
		{
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if (IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ISER2 |= (1<<(IRQNumber%64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <= 64)
		{
			*NVIC_ICER1 |= (1<<(IRQNumber%32));
		}
		else if (IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ICER2 |= (1<<(IRQNumber%64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQPriority, uint8_t IRQNumber)
{
	//1. Let's find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shiftAmount = (8 * iprx_section) + (8 - NBR_PRIO_BITS_IMPLEMENTED);
	*(NVIC_PRIO_BASE_ADDR +(iprx*4)) |= (IRQPriority << shiftAmount);
}



void GPIO_IRQhandling(uint8_t PinNumber)
{
	if (EXTI->PR1  & (1<<PinNumber))
	{
		EXTI->PR1 |= (1<<PinNumber);
	}
}



