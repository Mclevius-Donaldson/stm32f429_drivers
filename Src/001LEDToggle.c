/*
 * 001LEDToggle.c
 *
 *  Created on: Apr 21, 2021
 *      Author: Ben Mia
 */

#include "stm32f429.h"

void delay()
{
	for(uint32_t i; i < 5000; i++)
	{

	}
}

int main(void)
{
	GPIO_Handle_t LED_PIN;

	LED_PIN.pGPIOx = GPIOB;
	LED_PIN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	LED_PIN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED_PIN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED_PIN.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PP;
	LED_PIN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

	GPIO_PCLKControl(GPIOB, ENABLE);
	GPIO_Init(&LED_PIN);

	while(1)
	{
		GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		delay();
	}
}
