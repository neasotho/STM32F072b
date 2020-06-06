/*
 * stm32f072rb.h
 *
 *  Created on: Jun 4, 2020
 *  Author: Nidhin Easo Thomas
 */



#ifndef INC_STM32F072RB_H_
#define INC_STM32F072RB_H_

#include<stdint.h>
#define __vo volatile


/*
 * ARM Cortex-M0 NVIC Register Address
 */
#define NVIC_ISER					((__vo uint32_t *)0xE000E100UL)	/* Base address for NVIC Interrupt Set Enable Register */
#define NVIC_ICER					((__vo uint32_t *)0xE000E180UL)	/* Base address for NVIC Interrupt Clear Enable Register */
#define NVIC_ISPR					((__vo uint32_t *)0xE000E200UL)	/* Base address for NVIC Interrupt Set Pending Register */
#define NVIC_ICPR					((__vo uint32_t *)0xE000E280UL) /* Base address for NVIC Interrupt Clear Pending Register */
#define NVIC_IPR_BASEADDR			((__vo uint32_t *)0xE000E400UL)	/* Base address for NVIC Interrupt Priority Register */
#define NVIC_IPR_BITS_ACCESS		2								/* 2 bits of Interrupt Priority is implemented in STM32F072RB micro-controller */


#define NVIC_IRQ_PRI0				0								/* NVIC IRQ Priority Level */
#define NVIC_IRQ_PRI1				1								/* NVIC IRQ Priority Level */
#define NVIC_IRQ_PRI2				2								/* NVIC IRQ Priority Level */
#define NVIC_IRQ_PRI3				3								/* NVIC IRQ Priority Level */



/* Base address for FLASH,SRAM and ROM */

#define FLASH_BASEADDR 		0x08000000UL 							/* Base address of Flash Memory */
#define SRAM1_BASEADDR 		0x20000000UL 							/* Base address of SRAM */
#define SRAM 				SRAM1_BASEADDR							/* Keeping SRAM1 as base SRAM */
#define ROM 				0x1FFFC800UL							/* Base address for ROM(System Memory) */


/*Base address for APB and AHBx Bus */

#define PERIPH_BASEADDR 	0x40000000UL							/* Peripheral Base address */
#define APBPERIPH_BASEADDR 	0x40000000UL							/* APB Bus base address */
#define AHB1PERIPH_BASEADDR 0x40020000UL							/* AHB1 Bus base address */
#define AHB2PERIPH_BASEADDR 0x48000000UL							/* AHB2 Bus base address */

/* Base address of peripherals on AHB1 bus */

#define RCC_BASEADDR 		(AHB1PERIPH_BASEADDR + 0x1000UL)		/* Base address for RCC */

/* Base address of peripherals on AHB2 bus */


#define GPIOA_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x0000UL )		/* Base address for GPIO port A */
#define GPIOB_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x0400UL )		/* Base address for GPIO port B */
#define GPIOC_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x0800UL )		/* Base address for GPIO port C */
#define GPIOD_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x0C00UL )		/* Base address for GPIO port D */
#define GPIOE_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x1000UL )		/* Base address for GPIO port E */
#define GPIOF_BASEADDR 		( AHB2PERIPH_BASEADDR + 0x1400UL )		/* Base address for GPIO port F */



/* Base address of peripherals on APB bus*/

#define I2C1_BASEADDR 		( APBPERIPH_BASEADDR + 0x5400UL )		/* Base address for I2C1 */
#define I2C2_BASEADDR 		( APBPERIPH_BASEADDR + 0x5800UL )		/* Base address for I2C2 */
#define USART1_BASEADDR 	( APBPERIPH_BASEADDR + 0x00013800UL )	/* Base address for USART1 */
#define USART2_BASEADDR 	( APBPERIPH_BASEADDR + 0x4400UL )		/* Base address for USART2 */
#define USART3_BASEADDR 	( APBPERIPH_BASEADDR + 0x4800UL )		/* Base address for USART3 */
#define USART4_BASEADDR 	( APBPERIPH_BASEADDR + 0x4C00UL )		/* Base address for USART4 */
#define USART5_BASEADDR 	( APBPERIPH_BASEADDR + 0x5000UL )		/* Base address for USART5 */
#define USART6_BASEADDR 	( APBPERIPH_BASEADDR + 0x00011400UL )	/* Base address for USART6 */
#define USART7_BASEADDR 	( APBPERIPH_BASEADDR + 0x00011800UL )	/* Base address for USART7 */
#define USART8_BASEADDR 	( APBPERIPH_BASEADDR + 0x00011C00UL )	/* Base address for USART8 */
#define SPI1_BASEADDR 		( APBPERIPH_BASEADDR + 0x00013000UL )	/* Base address for SPI1 */
#define SPI2_BASEADDR 		( APBPERIPH_BASEADDR + 0x3800UL )		/* Base address for SPI2 */
#define EXTI_BASEADDR 		( APBPERIPH_BASEADDR + 0x00010400UL )	/* Base address for EXTI */
#define SYSCFG_BASEADDR 	( APBPERIPH_BASEADDR + 0x00010000UL )	/* Base address for SYSCFG */



/* Peripheral Register Definitions */
/* Peripheral Register Definitions for GPIO */
typedef struct{
	__vo uint32_t MODER;								/* GPIO Mode Register; Offset: 0x00 */
	__vo uint32_t OTYPER;								/* GPIO Output Type Register; Offset: 0x04 */
	__vo uint32_t OSPEEDR;								/* GPIO Output Speed Register; Offset: 0x08 */
	__vo uint32_t PUPDR;								/* GPIO Pull-Up/Pull-Down Register; Offset: 0x0C */
	__vo uint32_t IDR;									/* GPIO Input Data Register; Offset: 0x10 */
	__vo uint32_t ODR;									/* GPIO Output Data Register; Offset: 0x14 */
	__vo uint32_t BSRR;									/* GPIO Bit Set/Reset Register; Offset: 0x18 */
	__vo uint32_t LCKR;									/* GPIO Configuration Lock Register; Offset: 0x1C */
	__vo uint32_t AFR[2];								/* GPIO Alternation Function Mode Register Low(AFR[0]) and High(AFR[1]); Offset: 0x20 */
	__vo uint32_t BRR;									/* GPIO Bit Reset Register; Offset: 0x28 */

}GPIO_RegDef_t;

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)			/* GPIO A Base Address type cast to (GPIO_RegDef_t*) */
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)			/* GPIO B Base Address type cast to (GPIO_RegDef_t*) */
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)			/* GPIO C Base Address type cast to (GPIO_RegDef_t*) */
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)			/* GPIO D Base Address type cast to (GPIO_RegDef_t*) */
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)			/* GPIO E Base Address type cast to (GPIO_RegDef_t*) */
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)			/* GPIO F Base Address type cast to (GPIO_RegDef_t*) */


/* Peripheral Register Definition for RCC */


typedef struct{
	__vo uint32_t CR;									/* Clock Control Register; Offset: 0x00 */
	__vo uint32_t CFGR;									/* Clock Configuration Register; Offset: 0x04 */
	__vo uint32_t CIR;									/* Clock Interrupt Register; Offset: 0x08 */
	__vo uint32_t APB2RSTR;								/* APB Peripheral Reset Register 2; Offset: 0x0C */
	__vo uint32_t APB1RSTR;								/* APB Peripheral Reset Register 1; Offset: 0x10 */
	__vo uint32_t AHBENR;								/* AHB Peripheral Clock Enable Register; Offset: 0x14 */
	__vo uint32_t APB2ENR;								/* APB Peripheral Clock Enable Register 2; Offset: 0x18 */
	__vo uint32_t APB1ENR;								/* APB Peripheral Clock Enable Register 1; Offset: 0x1C */
	__vo uint32_t BDCR;									/* Domain Control Register; Offset: 0x20 */
	__vo uint32_t CSR;									/* Control/Status Register; Offset: 0x24 */
	__vo uint32_t AHBRSTR;								/* AHB Peripheral Reset Register; Offset: 0x28 */
	__vo uint32_t CFGR2;								/* Clock Configuration Register 2; Offset: 0x2C */
	__vo uint32_t CFGR3;								/* Clock Configuration Register 3; Offset: 0x30 */
	__vo uint32_t CR2;									/* Clock Control Register 2; Offset: 0x34 */
}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)				/* RCC Base Address type cast to (RCC_RegDef_t*) */

/* EXTI peripheral register definition structure */

typedef struct{
	__vo uint32_t IMR;									/* EXTI Interrupt Mask Register; Offset: 0x00 */
	__vo uint32_t EMR;									/* EXTI Event Mask Register; Offset: 0x04 */
	__vo uint32_t RTSR;									/* EXTI Rising Trigger Selection Register; Offset: 0x08 */
	__vo uint32_t FTSR;									/* EXTI Falling Trigger Selection Register; Offset: 0x0C */
	__vo uint32_t SWIER;								/* EXTI Software Interrupt Event Register; Offset: 0x10 */
	__vo uint32_t PR;									/* EXTI Pending Register; Offset: 0x1C */


}EXTI_RegDef_t;

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)				/* EXTI Base Address type cast to (EXTI_RegDef_t*) */



/* SYSCFG peripheral register definition structure */

typedef struct
{
	__vo uint32_t CFGR1;
	__vo uint32_t RESERVED;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;
}SYSCFG_RegDef_t;

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)


/* GPIO Clock Enable Macros */


#define GPIOA_PCLK_EN() 	(RCC->AHBENR|=(1<<17))		/* Enable Clock for GPIO A */
#define GPIOB_PCLK_EN() 	(RCC->AHBENR|=(1<<18))		/* Enable Clock for GPIO B */
#define GPIOC_PCLK_EN() 	(RCC->AHBENR|=(1<<19))		/* Enable Clock for GPIO C */
#define GPIOD_PCLK_EN() 	(RCC->AHBENR|=(1<<20))		/* Enable Clock for GPIO D */
#define GPIOE_PCLK_EN() 	(RCC->AHBENR|=(1<<21))		/* Enable Clock for GPIO E */
#define GPIOF_PCLK_EN() 	(RCC->AHBENR|=(1<<22))		/* Enable Clock for GPIO F */


/* I2C Clock Enable Macros */

#define I2C1_PCLK_EN()		(RCC->APB1ENR|=(1<<21))		/* Enable Clock for I2C 1 */
#define I2C2_PCLK_EN()		(RCC->APB1ENR|=(1<<22))		/* Enable Clock for I2C 2 */

/* SPI Clock Enable Macros */

#define SPI1_PCLK_EN()		(RCC->APB2ENR|=(1<<12))		/* Enable Clock for SPI 1 */
#define SPI2_PCLK_EN()		(RCC->APB1ENR|=(1<<14))		/* Enable Clock for SPI 2 */


/* USART Clock Enable Macros */

#define USART1_PCLK_EN()	(RCC->APB2ENR|=(1<<14))		/* Enable clock for USART 1 */
#define USART2_PCLK_EN()	(RCC->APB1ENR|=(1<<17))		/* Enable clock for USART 2 */
#define USART3_PCLK_EN()	(RCC->APB1ENR|=(1<<18))		/* Enable clock for USART 3 */
#define USART4_PCLK_EN()	(RCC->APB1ENR|=(1<<19))		/* Enable clock for USART 4 */
#define USART5_PCLK_EN()	(RCC->APB1ENR|=(1<<20))		/* Enable clock for USART 5 */
#define USART6_PCLK_EN()	(RCC->APB2ENR|=(1<<5))		/* Enable clock for USART 6 */
#define USART7_PCLK_EN()	(RCC->APB2ENR|=(1<<6))		/* Enable clock for USART 7 */
#define USART8_PCLK_EN()	(RCC->APB2ENR|=(1<<7))		/* Enable clock for USART 8 */


/* System Configuration Control Clock Enable Macros */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR|=(1<<0))		/* Enable Clock for System Configuration Controller */


/* GPIO Clock Disable Macros */

#define GPIOA_PCLK_DI() 	(RCC->AHBENR&=~(1<<17))		/* Disable Clock for GPIO A */
#define GPIOB_PCLK_DI() 	(RCC->AHBENR&=~(1<<18))		/* Disable Clock for GPIO B */
#define GPIOC_PCLK_DI() 	(RCC->AHBENR&=~(1<<19))		/* Disable Clock for GPIO C */
#define GPIOD_PCLK_DI() 	(RCC->AHBENR&=~(1<<20))		/* Disable Clock for GPIO D */
#define GPIOE_PCLK_DI() 	(RCC->AHBENR&=~(1<<21))		/* Disable Clock for GPIO E */
#define GPIOF_PCLK_DI() 	(RCC->AHBENR&=~(1<<22))		/* Disable Clock for GPIO F */


/* I2C Clock Disable Macros */

#define I2C1_PCLK_DI()		(RCC->APB1ENR&=~(1<<21))	/* Disable Clock for I2C 1 */
#define I2C2_PCLK_DI()		(RCC->APB1ENR&=~(1<<22))	/* Disable Clock for I2C 2 */


/* SPI Clock Disable Macros */

#define SPI1_PCLK_DI()		(RCC->APB2ENR&=~(1<<12))	/* Disable Clock for SPI 1 */
#define SPI2_PCLK_DI()		(RCC->APB1ENR&=~(1<<14))	/* Disable Clock for SPI 2 */

/* USART Clock Disable Macros */

#define USART1_PCLK_DI()	(RCC->APB2ENR&=~(1<<14))	/* Disable clock for USART 1 */
#define USART2_PCLK_DI()	(RCC->APB1ENR&=~(1<<17))	/* Disable clock for USART 2 */
#define USART3_PCLK_DI()	(RCC->APB1ENR&=~(1<<18))	/* Disable clock for USART 3 */
#define USART4_PCLK_DI()	(RCC->APB1ENR&=~(1<<19))	/* Disable clock for USART 4 */
#define USART5_PCLK_DI()	(RCC->APB1ENR&=~(1<<20))	/* Disable clock for USART 5 */
#define USART6_PCLK_DI()	(RCC->APB2ENR&=~(1<<5))		/* Disable clock for USART 6 */
#define USART7_PCLK_DI()	(RCC->APB2ENR&=~(1<<6))		/* Disable clock for USART 7 */
#define USART8_PCLK_DI()	(RCC->APB2ENR&=~(1<<7))		/* Disable clock for USART 8 */


/* System Configuration Control Clock Disable Macros */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR&=~(1<<0))		/* Disable Clock for System Configuration Controller */

/* GPIO Register Reset */
#define GPIOA_REG_RESET()	do {(RCC->AHBRSTR|=(1<<17)); (RCC->AHBRSTR&=~(1<<17));}while(0)			/* Reset GPIO A Registers */
#define GPIOB_REG_RESET()	do {(RCC->AHBRSTR|=(1<<18)); (RCC->AHBRSTR&=~(1<<18));}while(0)			/* Reset GPIO B Registers */
#define GPIOC_REG_RESET()	do {(RCC->AHBRSTR|=(1<<19)); (RCC->AHBRSTR&=~(1<<19));}while(0)			/* Reset GPIO C Registers */
#define GPIOD_REG_RESET()	do {(RCC->AHBRSTR|=(1<<20)); (RCC->AHBRSTR&=~(1<<20));}while(0)			/* Reset GPIO D Registers */
#define GPIOE_REG_RESET()	do {(RCC->AHBRSTR|=(1<<21)); (RCC->AHBRSTR&=~(1<<21));}while(0)			/* Reset GPIO E Registers */
#define GPIOF_REG_RESET()	do {(RCC->AHBRSTR|=(1<<22)); (RCC->AHBRSTR&=~(1<<22));}while(0)			/* Reset GPIO F Registers */


#define GPIO_BASEADDR_TO_EXTICODE(x)	   ((x==GPIOA)?0:\
											(x==GPIOB)?1:\
											(x==GPIOC)?2:\
											(x==GPIOD)?3:\
											(x==GPIOE)?4:5)			/* EXTI Code for GPIO Ports */
#define IRQ_NUM_EXTI0_1		5										/* EXTI Line[1:0] interrupts NVIC Position */
#define IRQ_NUM_EXTI2_3		6										/* EXTI Line[3:2] interrupts NVIC Position */
#define IRQ_NUM_EXTI4_15	7										/* EXTI Line[15:4] interrupts NVIC Position */

/*Generic  Macros */
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET 	SET
#define GPIO_PIN_RESET  RESET



#include<stm32f072rb_gpio_driver.h>				/* Include GPIO Driver Header file */



#endif /* INC_STM32F072RB_H_ */

