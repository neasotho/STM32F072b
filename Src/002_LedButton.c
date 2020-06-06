/*
 * 002_LedButton.c
 *
 *  Created on	: Jun 5, 2020
 *  Author		: Nidhin Easo Thomas
 *  Details		: Functionality check for GPIO driver: Toggle OnBoard Led(Port A Pin 5) of STM32F072RB Nucleo64 board
 *  			  on On-board Button press(Port C Pin 13)
 */


#include<stm32f072rb.h>
#define LOW 0
#define BTN_PRESSED LOW									/* For STM32F072RB Nucleo64 board. When On-Board button is pressed, Port C Pin 13 is driven to GROUND */

void delay()
{
	for(uint32_t i=0;i<500000;i++);
}

int main()
{
	GPIO_Handle_t GpioLed,GpioBtn;
	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNum=GPIO_PIN_NUM_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OUT_SPEED_HIGH;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType=GPIO_OUT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdCtrl=GPIO_PIN_PUPD_DIS;
	GPIO_PeriphClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);


	GpioBtn.pGPIOx=GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNum=GPIO_PIN_NUM_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed=GPIO_OUT_SPEED_HIGH;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdCtrl=GPIO_PIN_PUPD_DIS;
	GPIO_PeriphClkCtrl(GPIOC, ENABLE);
	GPIO_Init(&GpioBtn);





	while(1)
	{
		if((GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13))==BTN_PRESSED)
		{
			delay();														/* Debounce time for button press */
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);

		}

	}
	return 0;

}
