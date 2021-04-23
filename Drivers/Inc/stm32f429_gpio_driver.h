/*
 * stm32f429_gpio_driver.h
 *
 *  Created on: Apr 9, 2021
 *      Author: Ben Mia
 */

#ifndef INC_STM32F429_GPIO_DRIVER_H_
#define INC_STM32F429_GPIO_DRIVER_H_

#include "stm32f429.h"

/*
 * This is a handle structure for a GPIO pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;				/* !< Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/* !< Possible values from @GPIO_OTYPE >*/
	uint8_t GPIO_PinPuPdControl;		/* !< Possible values from @GPIO_OSPEED >*/
	uint8_t GPIO_PinOPType;					/* !< Possible values from @GPIO_PUPD >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

typedef struct
{
										// Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;				// THIS HOLDS THE BASE ADDRESS OF THE GPIO PORT TO WHICH THE PIN BELONGS
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES - GPIO Pin Modes
 */
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALT			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4		// GPIO input interrupt falling edge trigger
#define GPIO_MODE_IT_RT			5		// GPIO input interrupt rising edge trigger
#define GPIO_MODE_IT_RFT		6		// GPIO input interrupt rising/falling edge trigger

/*
 * @GPIO_OTYPE - GPIO Output Types
 */
#define GPIO_OTYPE_PP			0
#define GPIO_OTYPE_OD			1

/*
 * @GPIO_OSPEED - GPIO Output Speeds
 */
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MED			1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_ULTRA_FAST	3

/*
 * @GPIO_PUPD - GPIO Pull-Up/Pull-Down
 */
#define GPIO_PIN_NO_PUPD		0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * @GPIO_PIN - GPIO Pin No.
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN 4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/***********************************************************************************************************************
 * 												APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 *
 ***********************************************************************************************************************/

/*
 * Init & DeInit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *gPIOx);												//Deinit resets registers in selected peripheral

/*
 * Peripheral clock setup
 */
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint8_t EN);

/*
 * Data read/write
 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *gPIOx, uint8_t PinNumber);						// Data will return 0 or 1 (stored as uint8_t)
uint16_t GPIO_ReadPort(GPIO_RegDef_t *gPIOx);										// Data will return 16-bit input port
void GPIO_WritePin(GPIO_RegDef_t *gPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIO_RegDef_t *gPIOx, uint8_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *gPIOx, uint8_t PinNumber);

/*
 * IRQ config and handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EN);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F429_GPIO_DRIVER_H_ */
