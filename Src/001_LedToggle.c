/*
 * 001_LedToggle.c
 *
 *  Created on	: Jun 5, 2020
 *  Author		: Nidhin Easo Thomas
 *  Details		: Functionality check for GPIO driver: Toggle OnBoard Led of STM32F072RB Nucleo64 board
 *  			  keeping GPIO pin in Push-pull configuration with Pull-up and Pull down disabled
 *
 */


#include<stm32f072rb.h>


void delay()
{
	for(uint32_t i=0;i<500000;i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNum=GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OUT_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl=GPIO_PIN_PUPD_DIS;
	GPIO_PeriphClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay();
	}
	return 0;

}
