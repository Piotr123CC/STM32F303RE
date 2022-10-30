/*
 * stm32f303xx.h
 *
 *  Created on: Oct 7, 2022
 *      Author: piotr
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_


#include "stdint.h"


#define __vo volatile

/***********************************************************STARTLORicessir Specific details***********************/

/*
 * ARM cortex Mx Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0 					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 					((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4 					((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5 					((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6 					((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7 					((__vo uint32_t*)0xE000E11C)

/*
 * ARM cortex Mx Processor NVIC ICERx register addresses
 */

#define NVIC_ICER0					((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0XE000E18C)
#define NVIC_ICER4					((__vo uint32_t*)0XE000E190)
#define NVIC_ICER5					((__vo uint32_t*)0XE000E194)
#define NVIC_ICER6					((__vo uint32_t*)0XE000E198)
#define NVIC_ICER7					((__vo uint32_t*)0XE000E19C)


/*Base addresses of Flash and SRAM memories*/
#define FLASH_BASEADDR 				0x08000000U // Main memory  256Kb
#define SRAM1BASEADDR 				0x20000000U	// Just one SRAM
#define SRAM 						SRAM1BASEADDR // 40Kb
#define ROM 						0x1FFFD800U //System memory

/*AHBx and APBx Bus Pherpipheral base addresses*/
#define PERIPH_BASE 				0x40000000U
#define APB1PHERPH_BASEADDR 		PERIPH_BASE
#define APB2PHERPH_BASEADDR 		0x40010000U
#define AHB1PHERPH_BASEADDR 		0x40020000U
#define AHB2PHERPH_BASEADDR 		0x48000000U
#define AHB3PHERPH_BASEADDR 		0x50000000U
#define AHB4PHERPH_BASEADDR 		0x60000000U

/*Base addresses of pherpipherals which are hanging on AHB4 bus */
#define FMC_control_registers_BASEADDR		0xA0000000U
#define FMC_banks_3_and_4_BASEADDR			0x80000000U
#define FMC_banks_1_and_2_BASEADDR			(AHB4PHERPH_BASEADDR + 0x0000)


/*Base addresses of pherpipherals which are hanging on AHB3 bus */

#define ADC3_ADC4_BASEADDR			(AHB3PHERPH_BASEADDR+0x0400)
#define ADC1_ADC2_BASEADDR			(AHB3PHERPH_BASEADDR+0x0000)

/*Base addresses of pherpipherals which are hanging on AHB2 bus */
#define GPIOH_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x1C00)
#define GPIOG_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x1800)
#define GPIOF_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x1400)
#define GPIOE_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x1000)
#define GPIOD_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x0C00)
#define GPIOC_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x0800)
#define GPIOB_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x0400)
#define GPIOA_BASEADDR 				(AHB2PHERPH_BASEADDR + 0x0000)

/*Base addresses of pherpipherals which are hanging on AHB1 bus */
#define TSC_BASEADDR 				(APB1PHERPH_BASEADDR + 0x4000)
#define CRC_BASEADDR				(APB1PHERPH_BASEADDR + 0x3000)
#define FLASH_INTERFACE_BASEADDR	(APB1PHERPH_BASEADDR + 0x2000)
#define RCC_BASEADDR				(AHB1PHERPH_BASEADDR + 0x1000)
#define DMA2_BASEADDR				(APB1PHERPH_BASEADDR + 0x0400)
#define DMA1_BASEADDR				(APB1PHERPH_BASEADDR + 0x0000)

/*Base addresses of pherpipherals which are hanging on AHB2 bus */
#define TIM20_BASEADDR				(APB2PHERPH_BASEADDR+0x5000)
#define	TIM17_BASEADDR				(APB2PHERPH_BASEADDR+0x4800)
#define	TIM16_BASEADDR				(APB2PHERPH_BASEADDR+0x4400)
#define	TIM15_BASEADDR				(APB2PHERPH_BASEADDR+0x4000)
#define	SPI4_BASEADDR				(APB2PHERPH_BASEADDR+0x3C00)
#define	USART1_BASEADDR				(APB2PHERPH_BASEADDR+0x3800)
#define	TIM8_BASEADDR				(APB2PHERPH_BASEADDR+0x3400)
#define	SPI1_BASEADDR				(APB2PHERPH_BASEADDR+0x3000)
#define	TIM1_BASEADDR				(APB2PHERPH_BASEADDR+0x2C00)
#define	EXTI_BASEADDR				(APB2PHERPH_BASEADDR+0x0400)
#define	SYSCFG_BASEADDR  (APB2PHERPH_BASEADDR+0x0000)

/*Base addresses of pherpipherals which are hanging on APB1 bus */
#define I2C3_BASEADDR				(APB1PHERPH_BASEADDR + 0X7800)
#define DAC1_BASEADDR				(APB1PHERPH_BASEADDR + 0X7400)
#define PWR_BASEADDR				(APB1PHERPH_BASEADDR + 0X7000)
#define bxCAN_BASEADDR_BASEADDR		(APB1PHERPH_BASEADDR + 0X6400)
#define USB_CAN_SRAM_BASEADDR		(APB1PHERPH_BASEADDR + 0X6000)
#define USB_device_FS_BASEADDR		(APB1PHERPH_BASEADDR + 0X5C00)
#define I2C2_BASEADDR				(APB1PHERPH_BASEADDR + 0X5800)
#define I2C1_BASEADDR				(APB1PHERPH_BASEADDR + 0X5400)
#define UART5_BASEADDR				(APB1PHERPH_BASEADDR + 0X5000)
#define UART4_BASEADDR				(APB1PHERPH_BASEADDR + 0X4C00)
#define USART3_BASEADDR				(APB1PHERPH_BASEADDR + 0X4800)
#define USART2_BASEADDR				(APB1PHERPH_BASEADDR + 0X4400)
#define I2S3ext_BASEADDR			(APB1PHERPH_BASEADDR + 0X4000)
#define SPI3_I2S3_BASEADDR			(APB1PHERPH_BASEADDR + 0X3C00)
#define SPI2_I2S2_BASEADDR			(APB1PHERPH_BASEADDR + 0X3800)
#define I2S2ext_BASEADDR			(APB1PHERPH_BASEADDR + 0X3400)
#define IWDG_BASEADDR				(APB1PHERPH_BASEADDR + 0X3000)
#define WWDG_BASEADDR				(APB1PHERPH_BASEADDR + 0X2C00)
#define RTC_BASEADDR				(APB1PHERPH_BASEADDR + 0X2800)
#define TIM7_BASEADDR				(APB1PHERPH_BASEADDR + 0X1400)
#define TIM6_BASEADDR				(APB1PHERPH_BASEADDR + 0X1000)
#define TIM4_BASEADDR				(APB1PHERPH_BASEADDR + 0X0800)
#define TIM3_BASEADDR				(APB1PHERPH_BASEADDR + 0X0400)
#define TIM2_BASEADDR				(APB1PHERPH_BASEADDR + 0X0000)


/******************** PHERPIPHERAL REGISTER DEFINITION STRUCTURES **********************/

/* pherpipheral register definition for GPIO */
typedef struct
{
	 __vo uint32_t MODER;			/*GPIO port mode register   					ADDR_OFFSET 0x00 */
	 __vo uint32_t OTYPER;	 		/*GPIO port output type register   				ADDR_OFFSET 0x04 */
	 __vo uint32_t OSPEEDR;			/*GPIO port output speed register   			ADDR_OFFSET 0x08 */
	 __vo uint32_t PUPDR;			/*GPIO port pull-up/pull-down register  		ADDR_OFFSET 0x0C */
	 __vo uint32_t IDR;				/*GPIO port input data register   				ADDR_OFFSET 0x10 */
	 __vo uint32_t ODR;				/*GPIO port output data register   				ADDR_OFFSET 0x14 */
	 __vo uint32_t BSRR;			/*GPIO port bit set/reset register    			ADDR_OFFSET 0x18 */
	 __vo uint32_t LCKR;			/*GPIO port configuration lock register  		ADDR_OFFSET 0x1C */
	 __vo uint32_t AFR[2];			/*GPIO alternate function low/high register  	AFR[0] - AFRL, AFR[1] - AFRH   ADDR_OFFSET 0x20 */
	 __vo uint32_t BRR;				/*GPIO port bit reset register   				ADDR_OFFSET 0x28 */
}GPIO_RegDef_t;

/* pherpipheral register definition for RCC */

typedef struct
{
	__vo uint32_t CR; 					/*Clock control register     ADDR_OFFSET 0x00*/
	__vo uint32_t CFGR; 				/*Clock configuration register     ADDR_OFFSET 0x04*/
	__vo uint32_t CIR; 					/*Clock interrupt register     ADDR_OFFSET 0x08*/
	__vo uint32_t APB2RSTR; 			/*APB2 peripheral reset register     ADDR_OFFSET 0xC*/
	__vo uint32_t APB1RSTR; 			/*APB1 peripheral reset register 	 ADDR_OFFSET 0x10*/
	__vo uint32_t AHBENR; 				/*AHB peripheral clock enable register     ADDR_OFFSET 0x14*/
	__vo uint32_t APB2ENR; 				/*APB2 peripheral clock enable register     ADDR_OFFSET 0x18*/
	__vo uint32_t APB1ENR; 				/*APB1 peripheral clock enable register     ADDR_OFFSET 0x1C*/
	__vo uint32_t BDCR; 				/*RTC domain control register     ADDR_OFFSET 0x20*/
	__vo uint32_t CSR; 					/*Control/status register     ADDR_OFFSET 0x24*/
	__vo uint32_t AHBRSTR; 				/*AHB peripheral reset register     ADDR_OFFSET 0x28*/
	__vo uint32_t CFGR2; 				/*Clock configuration register 2     ADDR_OFFSET 0x2C*/
	__vo uint32_t CFGR3; 				/*Clock configuration register 3     ADDR_OFFSET 0x30*/

}RCC_RegDeg_t;




/*
 * pheripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t EXTI_IMR1;	/*Interrupt mask register OFFSET 0x00    */
	__vo uint32_t EXTI_EMR1;	/*Event mask register register OFFSET 0x04    */
	__vo uint32_t EXTI_RTSR1;	/*Rising trigger selection register OFFSET 0x08    */
	__vo uint32_t EXTI_FTSR1;	/*Falling trigger selection registe 0x0C    */
	__vo uint32_t EXTI_SWIER1;	/*Software interrupt event register 0x10    */
	__vo uint32_t EXTI_PR1;		/*Pending register OFFSET 0x14    */
	__vo uint32_t RESERVED[2]; 	/*8 bytes reserved 0x18-1C */
	__vo uint32_t EXTI_IMR2;	/*Interrupt mask register 2 OFFSET 0x20    */
	__vo uint32_t EXTI_EMR2;	/*Event mask register 2 OFFSET 0x24    */
	__vo uint32_t EXTI_RTSR2;	/*Rising trigger selection register 2 OFFSET 0x28    */
	__vo uint32_t EXTI_FTSR2;	/*Rising trigger selection register 2 OFFSET 0x2C    */
	__vo uint32_t EXTI_SWIER2;	/*Software interrupt event register 2 OFFSET 0x030    */
	__vo uint32_t EXTI_PR2;		/*Pending register 2 OFFSET 0x34    */

}EXTI_RegDef_t;






/* pherpiheral definitions (Pherpiheral base addresses typecasted to xxx_RegDef_t */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)



/*
 * pheripheral register definition structure for sysconfig
 *
 */

typedef struct{
	__vo uint32_t CFGR1; 		/*SYSCFG configuration register 1  OFFSET 0x00*/
	__vo uint32_t RCR; 			/*SYSCFG CCM SRAM protection register  OFFSET 0x04*/
	__vo uint32_t EXTICR[4]; 		/*SYSCFG external interrupt configuration register 1  OFFSET 0x08*/
	__vo uint32_t CFGR2; 		/*SYSCFG configuration register 2  OFFSET 0x18*/
	__vo uint32_t RESERVED[11];			/*RESERVED SPACE 1C-0x47  for */
	__vo uint32_t CFGR4; 		/*SYSCFG configuration register 4  OFFSET 0x48*/
	__vo uint32_t CFGR3; 		/*SYSCFG configuration register 3  OFFSET 0x50*/
}SYSCFG_RegDef_t;


#define 	RCC   	((RCC_RegDeg_t*)RCC_BASEADDR)

#define 	EXTI  	((EXTI_RegDef_t*)EXTI_BASEADDR)

#define  	SYSCFG1 	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*********************************ENABLE CLOCK MACROS*****************/

/* Clock Enable Macros for GPIOx pheripherals*/

#define GPIOA_PCLK_EN() 	(RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN() 	(RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN() 	(RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN() 	(RCC->AHBENR |= (1<<20))
#define GPIOE_PCLK_EN() 	(RCC->AHBENR |= (1<<21))
#define GPIOF_PCLK_EN() 	(RCC->AHBENR |= (1<<22))
#define GPIOG_PCLK_EN() 	(RCC->AHBENR |= (1<<23))
#define GPIOH_PCLK_EN() 	(RCC->AHBENR |= (1<<16))

/* Clock Enable Macros for PWR */
#define PWR_PCLK_EN() 		(RCC->APB1ENR |= (1<<28))


/* Clock Enable Macros for SystemConfig */
#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |= (1<<0))


/* Clock Enable Macros for I2Cx pheripherals*/
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |= (1<<30))


/* Clock Enable Macros for SPIx pheripherals*/
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() 		(RCC->APB2ENR |= (1<<15))


/* Clock Enable Macros for UART/USARTx pheripherals*/
#define USART1_PCLK_EN() 	(RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN() 	(RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() 	(RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN() 	(RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN() 	(RCC->APB1ENR |= (1<<20))

/*********************************DISABLE CLOCK MACROS*****************/

/* Clock Disable Macros for GPIOx pheripherals*/

#define GPIOA_PCLK_DI() 	(RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI() 	(RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI() 	(RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI() 	(RCC->AHBENR &= ~(1<<20))
#define GPIOE_PCLK_DI() 	(RCC->AHBENR &= ~(1<<21))
#define GPIOF_PCLK_DI() 	(RCC->AHBENR &= ~(1<<22))
#define GPIOG_PCLK_DI() 	(RCC->AHBENR &= ~(1<<23))
#define GPIOH_PCLK_DI() 	(RCC->AHBENR &= ~(1<<16))

/* Clock Disable Macros for PWR */
#define PWR_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<28))


/* Clock Disable Macros for SystemConfig */
#define SYSCFG_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<0))


/* Clock Disable Macros for I2Cx pheripherals*/
#define I2C1_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<30))


/* Clock Disable Macros for SPIx pheripherals*/
#define SPI1_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() 		(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI() 		(RCC->APB2ENR &= ~(1<<15))


/* Clock Disable Macros for UART/USARTx pheripherals*/
#define USART1_PCLK_DI() 	(RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() 	(RCC->APB1ENR &= ~(1<<20))


/*
 * Clock rest registers macros
 */

#define GPIOA_REG_RESET() 		do{ GPIOA_PCLK_EN(); 	GPIOA_PCLK_DI();}while(0)
#define GPIOB_REG_RESET() 		do{ GPIOB_PCLK_EN(); 	GPIOB_PCLK_DI();}while(0)
#define GPIOC_REG_RESET()		do{ GPIOC_PCLK_EN(); 	GPIOC_PCLK_DI();}while(0)
#define GPIOD_REG_RESET() 		do{ GPIOD_PCLK_EN(); 	GPIOD_PCLK_DI();}while(0)
#define GPIOE_REG_RESET() 		do{ GPIOE_PCLK_EN(); 	GPIOE_PCLK_DI();}while(0)
#define GPIOF_REG_RESET() 		do{ GPIOF_PCLK_EN(); 	GPIOF_PCLK_DI();}while(0)
#define GPIOG_REG_RESET() 		do{ GPIOG_PCLK_EN(); 	GPIOG_PCLK_DI();}while(0)
#define GPIOH_REG_RESET() 		do{ GPIOH_PCLK_EN(); 	GPIOH_PCLK_DI();}while(0)

#define GPIO_BASEADDR_TO(x)	   ((x == GPIOA)?0:\
								(x == GPIOB)?1:\
								(x == GPIOC)?2:\
								(x == GPIOD)?3:\
								(x == GPIOE)?4:\
								(x == GPIOF)?5:\
								(x == GPIOG)?6:\
								(x == GPIOH)?7:0)


/*
 * Vector table
 */

#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2_TS 		8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10 		40



/*
 * Some generic macros
 */

#define ENABLE 				1
#define DISABLE 			0
#define SET  				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET  		SET
#define GPIO_PIN_RESET		RESET


#include "stm32f303xx_gpio_driver.h"

#endif /* INC_STM32F303XX_H_ */
