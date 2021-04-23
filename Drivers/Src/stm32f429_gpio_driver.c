/*
 * stm32f429_gpio_driver.c
 *
 *  Created on: Apr 9, 2021
 *      Author: Ben Mia
 */

#include "stm32f429_gpio_driver.h"




/********************************************************************************************************************************************
 *
 * 															Driver Functions
 *
 *
 ********************************************************************************************************************************************/

/*
 *  GPIO Init & DeInit
 */

/********************************************************************************************************************************************
 * @fn				- GPIO_Init
 * @brief			- This function initializes the given GPIO with the settings provided
 * @param[in]		- *pGPIOHandle: This value will hold the configuration bits for the given GPIO as well as the address
 * @return			- NA
 * @note			- NA
 *******************************************************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;	//temp register

	// Configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));		// Shift the data 2*PinNumber
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));					// Clear the register you want to change to make sure your settings are copied correctly
		pGPIOHandle->pGPIOx->MODER |= temp;																			// Setting the register |= (or equal) will change the register ONLY at the specified pin (look at 2 lines above)
	}else
	{

	}

	temp = 0;

	// Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	// Configure the pull-up/down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	// Configure the output alt. functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT)
	{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->AFR[temp1] |= temp;
	}

	temp = 0;
}

/********************************************************************************************************************************************
 * @fn				- GPIO_DeInit
 * @brief			- This function initializes the given GPIO with the settings
 * @param[in]		- *pGPIOHandle: This value will hold the configuration bits for the given GPIO as well as the address
 * @return			- NA
 * @note			- NA
 *******************************************************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	else if(pGPIOx == GPIOJ)
	{
		GPIOJ_REG_RESET();
	}
	else if(pGPIOx == GPIOK)
	{
		GPIOK_REG_RESET();
	}
}

/*
 *  GPIO Clock Control
*/

/********************************************************************************************************************************************
 * @fn				- GPIO_PCLKControl
 * @brief			- This Function enables or disables peripheral clock for the given GPIO Port
 * @param[in]		- *pGPIOx: base address of gpio peripheral
 * @param[in]		- EN: ENABLE or DISABLE
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EN)
{
	if(EN == ENABLE){
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
		else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}
		else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}else
	{

	}
}

/********************************************************************************************************************************************
 * @fn				- GPIO_ReadPin
 * @brief			- This function will read the state of the given GPIO pin
 * @param[in]		- *pGPIOx: base address of GPIO peripheral
 * @param[in]		- PinNumber: Pin number
 * @return			- State of pin (0 or 1)
 * @note			- NA
 ********************************************************************************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/********************************************************************************************************************************************
 * @fn				- GPIO_ReadPort
 * @brief			- This function will read the state of the given GPIO port
 * @param[in]		- *pGPIOx: base address of GPIO peripheral
 * @return			- 16-bit data from GPIOx port
 * @note			- NA
 ********************************************************************************************************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/********************************************************************************************************************************************
 * @fn				- GPIO_WritePin
 * @brief			- This function will write an output to the GPIO pin
 * @param[in]		- *pGPIOx: base address of GPIO peripheral
 * @param[in]		- PinNumber: Pin numbers
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}else
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
}

/********************************************************************************************************************************************
 * @fn				- GPIO_WritePort
 * @brief			- This function will write an output to the GPIO port
 * @param[in]		- *pGPIOx: base address of GPIO peripheral
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************************************************************************************************
 * @fn				- GPIO_TogglePin
 * @brief			- This function will toggle output to the GPIO pin
 * @param[in]		- *pGPIOx: base address of GPIO peripheral
 * @param[in]		- PinNumber: Pin numbers
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ config and handling
 */


/********************************************************************************************************************************************
 * @fn				- GPIO_IRQConfig
 * @brief			- This function will configure the IRQ handler
 * @param[in]		- IRQNumber: Number of IRQ
 * @param[in]		- IRQPriority: Priority of IRQ
 * @param[in]		- EN: ENABLE or DISABLE
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EN)
{

}

/********************************************************************************************************************************************
 * @fn				- GPIO_IRQConfig
 * @brief			- This function will ???????????????????????????????????????????????????????????????????????????
 * @param[in]		- PinNumber: Pin number
 * @return			- NA
 * @note			- NA
 ********************************************************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
