/*
 * stm32f072rb_gpio_driver.h
 *
 *  Created on	: Jun. 5, 2020
 *  Author		: Nidhin Easo Thomas
 */

#ifndef INC_STM32F072RB_GPIO_DRIVER_H_
#define INC_STM32F072RB_GPIO_DRIVER_H_
#include<stm32f072rb.h>


/*
 * GPIO Pin Numbers(GPIO_PIN_NUM)
 */
#define GPIO_PIN_NUM_0 		0								/* GPIO Pin number 0 */
#define GPIO_PIN_NUM_1 		1								/* GPIO Pin number 1 */
#define GPIO_PIN_NUM_2 		2								/* GPIO Pin number 2 */
#define GPIO_PIN_NUM_3 		3								/* GPIO Pin number 3 */
#define GPIO_PIN_NUM_4 		4								/* GPIO Pin number 4 */
#define GPIO_PIN_NUM_5 		5								/* GPIO Pin number 5 */
#define GPIO_PIN_NUM_6 		6								/* GPIO Pin number 6 */
#define GPIO_PIN_NUM_7 		7								/* GPIO Pin number 7 */
#define GPIO_PIN_NUM_8 		8								/* GPIO Pin number 8 */
#define GPIO_PIN_NUM_9 		9								/* GPIO Pin number 9 */
#define GPIO_PIN_NUM_10 	10								/* GPIO Pin number 10 */
#define GPIO_PIN_NUM_11 	11								/* GPIO Pin number 11 */
#define GPIO_PIN_NUM_12 	12								/* GPIO Pin number 12 */
#define GPIO_PIN_NUM_13 	13								/* GPIO Pin number 13 */
#define GPIO_PIN_NUM_14 	14								/* GPIO Pin number 14 */
#define GPIO_PIN_NUM_15 	15								/* GPIO Pin number 15 */


/*
 * GPIO Possible operating modes(GPIO_PIN_MODES)
 */

#define GPIO_MODE_IN 				0				/* Input Mode for GPIO */
#define GPIO_MODE_OUT 				1				/* Output Mode for GPIO */
#define GPIO_MODE_ALTFN 			2				/* Alternate Function Mode for GPIO */
#define GPIO_MODE_ANALOG			3				/* Analog Mode for GPIO */
#define GPIO_MODE_IN_RE 			4				/* Input Rising Edge Detect for GPIO */
#define GPIO_MODE_IN_FE 			5				/* Input Falling Edge Detect for GPIO */
#define GPIO_MODE_IN_RFE 			6				/* Input Rising and Falling Edge Detect for GPIO */
/*
 * GPIO Possible Output Types(GPIO_OUTPUT_TYPES)
 */
#define GPIO_OUT_PP 				0				/* GPIO Output type Push-Pull Mode */
#define GPIO_OUT_OD 				1				/* GPIO Output Type Open-drain Mode */
/*
 * GPIO Possible Output Speed(GPIO_OUTPUT_SPEED)
 */
#define GPIO_OUT_SPEED_LOW 			0				/* GPIO Output Low Speed */
#define GPIO_OUT_SPEED_MED 			1				/* GPIO Output Medium Speed */
#define GPIO_OUT_SPEED_HIGH 		2				/* GPIO Output HIgh Speed */

/*
 * GPIO Pull-up/Pull-down configuration(GPIO_PUPD)
 */
#define GPIO_PIN_PUPD_DIS 				0			/* Disable GPIO Pull-up/Pull-down */
#define GPIO_PIN_PUPD_PU 				1			/* Enable GPIO Pull-up */
#define GPIO_PIN_PUPD_PD 				2			/* Enable GPIO Pull-down */

/*
 * GPIO Alternate Function Modes(GPIO_ALTFUN)
 */
#define GPIO_PIN_ALTFUN_0				0		/* GPIO Alternate Function Mode 0 */
#define GPIO_PIN_ALTFUN_1				1		/* GPIO Alternate Function Mode 1 */
#define GPIO_PIN_ALTFUN_2				2		/* GPIO Alternate Function Mode 2 */
#define GPIO_PIN_ALTFUN_3				3		/* GPIO Alternate Function Mode 3 */
#define GPIO_PIN_ALTFUN_4				4		/* GPIO Alternate Function Mode 4 */
#define GPIO_PIN_ALTFUN_5				5		/* GPIO Alternate Function Mode 5 */
#define GPIO_PIN_ALTFUN_6				6		/* GPIO Alternate Function Mode 6 */
#define GPIO_PIN_ALTFUN_7				7		/* GPIO Alternate Function Mode 7 */

/* Configuration structure for GPIO */
typedef struct{
	uint8_t GPIO_PinNum;						/* Possible values from @GPIO_PIN_NUM*/
	uint8_t GPIO_PinMode;						/* Possible values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;						/* Possible values from @GPIO_OUTPUT_SPEED */
	uint8_t GPIO_PinPuPdCtrl;					/* Possible values from @GPIO_PUPD */
	uint8_t GPIO_PinOpType;						/* Possible values from @GPIO_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;					/* Possible values from @GPIO_ALTFUN */
}GPIO_PinConfig_t;;






/* Handle structure for GPIO */
typedef struct
{
	GPIO_RegDef_t * pGPIOx;					/* Holds the base address of GPIO*/
	GPIO_PinConfig_t GPIO_PinConfig;		/* Holds the GPIO pin configuration settings */
}GPIO_Handle_t;


/* APIs Supported by the driver */

/* GPIO Clock setup*/
void GPIO_PeriphClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);

/* GPIO Initialize/Uninitialize */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/* GPIO Read/Write Data*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum,uint8_t Value);
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum);

/* GPIO IRQ handling*/
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNum,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);



#endif /* INC_STM32F072RB_GPIO_DRIVER_H_ */

