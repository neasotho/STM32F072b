/*
 * stm32f072rb_gpio_driver.c
 *
 *  Created on: Jun. 5, 2020
 *      Author: nidhi
 */


#include<stm32f072rb_gpio_driver.h>
/* APIs Supported by the driver */

/* GPIO Clock setup*/
/********************************************************************
 * @function			-	GPIO_PeriphClkCtrl
 *
 * @brief				-	Enable or Disable the GPIO Peripheral Clock
 *
 * @param[in]			-	Base address of GPIO peripheral
 * @param[in]			-	ENABLE or DISABLE
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void GPIO_PeriphClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
	{
		if(pGPIOx==GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx==GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx==GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx==GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx==GPIOE) GPIOE_PCLK_EN();
		else if (pGPIOx==GPIOF) GPIOF_PCLK_EN();
	}
	else
	{
		if(pGPIOx==GPIOA) GPIOA_PCLK_DI();
		else if (pGPIOx==GPIOB) GPIOB_PCLK_DI();
		else if (pGPIOx==GPIOC) GPIOC_PCLK_DI();
		else if (pGPIOx==GPIOD) GPIOD_PCLK_DI();
		else if (pGPIOx==GPIOE) GPIOE_PCLK_DI();
		else if (pGPIOx==GPIOF) GPIOF_PCLK_DI();
	}

}



/* GPIO Initialize/Uninitialize */
/********************************************************************
 * @function			-	GPIO_Init
 *
 * @brief				-	Initialize the GPIO Peripheral
 *
 * @param[in]			-	GPIO Handle
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0;

	/* Mode Initialiation */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));	/*Clear the corresponding bit position*/
		pGPIOHandle->GPIO_PinConfig.GPIO_PinMode&=~(0x11<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);		/*Set the corresponding bit position*/
		pGPIOHandle->pGPIOx->MODER|=temp;
		temp=0;

	}
	else
	{

		//Interrupt mode initialization
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IN_RE)
		{
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Enable Edge Rise Detect in RTSR */
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Disable Edge Fall Detect in FSTR */
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IN_FE)
		{
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Enable Edge Fall Detect in FTSR */
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Disable Edge Rise Detect in RTSR */
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IN_RFE)
		{
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Enable Edge Rise Detect in RTSR */
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);								/* Enable Edge Fall Detect in FTSR */

		}
		uint8_t Reg_Sel=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum/(uint8_t)4);
		uint8_t Pin_Sel=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum%(uint8_t)4);
		uint8_t Port_Code = GPIO_BASEADDR_TO_EXTICODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[Reg_Sel]|=(Port_Code<<Pin_Sel*4);




		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);									/* Enable EXTI in IMR */
	}
	/* Output Speed Initialization */
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDR&=~((0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;

	/* GPIO Pin Pull-up/Pull-down Setup */
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdCtrl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR&=~((0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	/* GPIO Pin Output Type configuration*/
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum);
	pGPIOHandle->pGPIOx->OTYPER&=~((0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
	pGPIOHandle->pGPIOx->OTYPER|=temp;

	/* GPIO Pin ALternate Function Mode configuration */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN)
	{	uint8_t Reg_Select=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum/(uint32_t)8);
		uint8_t Pin_Select=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNum%(uint32_t)8);
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*Pin_Select));
		pGPIOHandle->pGPIOx->AFR[Reg_Select]&=~((0xF<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNum));
		pGPIOHandle->pGPIOx->AFR[Reg_Select]|=temp;
	}








}



/********************************************************************
 * @function			-	GPIO_DeInit
 *
 * @brief				-	Deinitialize the GPIO Peripheral
 *
 * @param[in]			-	Base address of GPIO peripheral
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx==GPIOA) GPIOA_REG_RESET();
	else if (pGPIOx==GPIOB) GPIOB_REG_RESET();
	else if (pGPIOx==GPIOC) GPIOC_REG_RESET();
	else if (pGPIOx==GPIOD) GPIOD_REG_RESET();
	else if (pGPIOx==GPIOE) GPIOE_REG_RESET();
	else if (pGPIOx==GPIOF) GPIOF_REG_RESET();

}




/* GPIO Read/Write Data*/

/*********************************************************************
 * @function			-	GPIO_ReadFromInputPin
 *
 * @brief				-	Read Data from the specified pin of GPIO Port in Input Mode
 *
 * @param[in]			-	Base address of GPIO peripheral
 * @param[in]			-	GPIO Port Pin Number
 *
 * @return				-	uint8_t; 8 bit data
 *
 * @Note				-	None
 *********************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	uint8_t datVal;
	datVal=(uint8_t)((pGPIOx->IDR>>PinNum)&0x00000001);
	return datVal;
}




/***********************************************************************
 * @function			-	GPIO_ReadFromInputPort
 *
 * @brief				-	Read Data from GPIO Port in Input Mode
 *
 * @param[in]			-	Base address of GPIO peripheral
 *
 * @return				-	uint16_t; 16 bit data
 *
 * @Note				-	None
 ***********************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t datVal;
	datVal=pGPIOx->IDR;
	return datVal;

}




/***********************************************************************
 * @function			-	GPIO_WriteToInputPin
 *
 * @brief				-	Write Data to specified pin of GPIO Port in Output Mode
 *
 * @param[in]			-	Base address of GPIO peripheral
 * @param[in]			-	GPIO Port Pin Number
 * @param[in]			-	Value;uint8_t; 8 bit data
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum,uint8_t Value)
{
	if(Value==GPIO_PIN_SET)
	{
		pGPIOx->ODR|=(1<<PinNum);
	}
	else
	{
		pGPIOx->ODR&=~(1<<PinNum);
	}

}





/***********************************************************************
 * @function			-	GPIO_WriteToInputPort
 *
 * @brief				-	Write Data to GPIO Port in Output Mode
 *
 * @param[in]			-	Base address of GPIO peripheral
 * @param[in]			-	Value;uint16_t; 16 bit data
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value)
{
	pGPIOx->ODR=Value;

}





/***********************************************************************
 * @function			-	GPIO_ToggleOutputPin
 *
 * @brief				-	Toggle Data on the specified pin of GPIO Port in Output Mode
 *
 * @param[in]			-	Base address of GPIO peripheral
 * @param[in]			-	GPIO Port Pin Number
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNum)
{
	pGPIOx->ODR^=(1<<PinNum);

}





/* GPIO IRQ handling*/
/***********************************************************************
 * @function			-	GPIO_IRQInterruptConfig
 *
 * @brief				-	Configure Interrupt on GPIO Peripheral
 *
 * @param[in]			-	IRQ Number
 * @param[in]			-	ENABLE or DISABLE
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
		*NVIC_ISER|=(1<<IRQNum);
	else
		*NVIC_ICER|=(1<<IRQNum);
}



/***********************************************************************
 * @function			-	GPIO_IRQPriorityConfig
 *
 * @brief				-	Configure Interrupt Priority
 *
 * @param[in]			-	IRQ Number
 * @param[in]			-	Interrupt Priority
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNum,uint32_t IRQPriority)
{
	uint32_t iprx=IRQNum/4;
	uint32_t ipr_sec=IRQNum%4;
	*(NVIC_IPR_BASEADDR+iprx)|=(IRQPriority<<((ipr_sec*8)+(8-NVIC_IPR_BITS_ACCESS))); /* For Cortex M0 processor, only Bits 6 and 7 are implemented */

}



/***********************************************************************
 * @function			-	GPIO_IRQHandling
 *
 * @brief				-	Handle Interrupt on the specified pin of GPIO port
 *
 * @param[in]			-	GPIO Port Pin Number
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void GPIO_IRQHandling(uint8_t PinNum)
{
	if(EXTI->PR&(1<<PinNum))
	{
		EXTI->PR|=(1<<PinNum);
	}

}

