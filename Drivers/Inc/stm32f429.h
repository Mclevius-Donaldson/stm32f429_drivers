/*
 * stm32f429.h
 *
 *  Created on: Apr 9, 2021
 *      Author: Ben Mia
 */

#ifndef INC_STM32F429_H_
#define INC_STM32F429_H_

#include <stdint.h>
/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U			// The U at the end indicates to the compiler that this is unsigned
#define SRAM 					SRAM1_BASEADDR		// Base address of SRAM
#define SRAM1_BASEADDR			0x20000000U 		// SRAM1 is 112KB
#define SRAM2_BASEADDR			0X2001C000U 		// SRAM2 is 112KB behind SRAM1 (KB = 1024b)
#define ROM						0X1FFFC000			// ROM Memory location (System Memory) used for constants

/*
 * RCC Base Address
 */

#define RCC_BASEADDR			0x40023800

/*
 * Base Addresses of Peripherals
 */

#define PERIPH_BASEADDR 		0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
/*
 * APB1 Peripheral Addresses
 */

#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define	SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)

#define	USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define	USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define	UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define	UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define	I2C2_BASEADDR			(AHB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(AHB1PERIPH_BASE + 0x5C00)

/*
 * APB2 Peripheral Addresses
 */

/*
 * AHB1 Peripheral Addresses
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2800)

/*
 * AHB2 Peripheral Addresses
 */

#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)

#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)





/************************************ Peripheral Register Definition Structures *******************************************/

/*
 * These Type Def Structures are used to flexibly modify registers within certain similar peripherals (i2c, spi, etc...)
 *
 * 	NOTE: THE VOLATILE KEYWORD IS ADDED BECAUSE THE DATA MAY CHANGE AT ANY TIME. (Really only necessary for data on input...)
 */

typedef struct
{
	volatile uint32_t MODER;			// Mode 													Address Offset: 0x00
	volatile uint32_t OTYPER;			// Output Type 												Address Offset: 0x04
	volatile uint32_t OSPEEDER;			// Output Speed												Address Offset: 0x08
	volatile uint32_t PUPDR;			// Pull-Up/Pull-Down										Address Offset: 0x0C
	volatile uint32_t IDR;				// Input Data												Address Offset: 0x10
	volatile uint32_t ODR;				// Output Data												Address Offset: 0x14
	volatile uint32_t BSRR;				// Bit set/reset											Address Offset: 0x18
	volatile uint32_t LCKR;				// Configuration Lock										Address Offset: 0x1C
	volatile uint32_t AFR[2];			// Alternate Function Low & High (AF[0] = L, AF[1] = H)		Address Offset: 0x20
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;				//  Clock control											Address Offset: 0x00
	volatile uint32_t PLLCFGR;			//  PLL configure											Address Offset: 0x04
	volatile uint32_t CFGR;				//  Clock configuration										Address Offset: 0x08
	volatile uint32_t CIR;				//  Clock interrupt											Address Offset: 0x0C
	volatile uint32_t AHB1RSTR;			//  AHB1 peripheral reset									Address Offset: 0x10
	volatile uint32_t AHB2RSTR;			//  AHB2 peripheral reset									Address Offset: 0x14
	volatile uint32_t AHB3RSTR;			//  AHB2 peripheral reset									Address Offset: 0x18
	uint32_t RESERVED0;					//  Reserved												Address Offset: 0x1C
	volatile uint32_t APB1RSTR;			//  APB1 peripheral reset									Address Offset: 0x20
	volatile uint32_t APB2RSTR;			//  APB2 peripheral reset									Address Offset: 0x24
	uint32_t RESERVED1[2];				//  Reserved												Address Offset: 0x28
	volatile uint32_t AHB1ENR;			//  AHB1 peripheral clock enable							Address Offset: 0x30
	volatile uint32_t AHB2ENR;			//  AHB2 peripheral clock enable							Address Offset: 0x34
	volatile uint32_t AHB3ENR;			//  AHB3 peripheral clock enable							Address Offset: 0x38
	uint32_t RESERVED2;					//  Reserved												Address Offset: 0x3C
	volatile uint32_t APB1ENR;			//  APB1 peripheral clock enable							Address Offset: 0x40
	volatile uint32_t APB2ENR;			//  APB2 peripheral clock enable							Address Offset: 0x44
	uint32_t RESERVED3[2];				//  Reserved												Address Offset: 0x48
	volatile uint32_t AHB1LPENR;		//  AHB1 peripheral clock enable in lower power				Address Offset: 0x50
	volatile uint32_t AHB2LPENR;		//  AHB2 peripheral clock enable in lower poweR				Address Offset: 0x54
	volatile uint32_t AHB3LPENR;		//  AHB13 peripheral clock enable in lower power			Address Offset: 0x58
	uint32_t RESERVED4[2];				//  Clock control											Address Offset: 0x5C
	volatile uint32_t APB1LPENR;		//  APB1 peripheral clock enable in lower power				Address Offset: 0x60
	volatile uint32_t APB2LPENR;		//  APB2 peripheral clock enable in lower power				Address Offset: 0x64
	uint32_t RESERVED5[2];				//  Reserved												Address Offset: 0x68
	volatile uint32_t BDCR;				//  Backup domain control									Address Offset: 0x70
	volatile uint32_t CSR;				//  Clock control & status									Address Offset: 0x74
	uint32_t RESERVED6[2];				//  Reserved												Address Offset: 0x78
	volatile uint32_t SSCGR;			//  Spread spectrum clock generation						Address Offset: 0x80
	volatile uint32_t PLLI2SCFGR;		//  PLLI2S Configuration									Address Offset: 0x84
}RCC_RegDef_t;

/*
 * Peripheral definitions (Perpipheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA					((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG 					((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t *) GPIOI_BASEADDR)
#define GPIOJ 					((GPIO_RegDef_t *) GPIOJ_BASEADDR)
#define GPIOK 					((GPIO_RegDef_t *) GPIOK_BASEADDR)

#define RCC						((RCC_RegDef_t *) RCC_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()	( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()	( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()	( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()	( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()	( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()	( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()	( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()	( RCC->AHB1ENR |= (1 << 8) )
#define GPIOJ_PCLK_EN()	( RCC->AHB1ENR |= (1 << 9) )
#define GPIOK_PCLK_EN()	( RCC->AHB1ENR |= (1 << 10) )

/*
 * Clock enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN() ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN() ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN() ( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN() ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() ( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN() ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN() ( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() ( RCC->APB1ENR |= (1 << 18) )
#define UART4_PCLK_EN() ( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN() ( RCC->APB1ENR |= (1 << 20) )
#define USART6_PCLK_EN() ( RCC->APB2ENR |= (1 << 5) )
/*
 * Clock enable macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_EN() ( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 8) )
#define GPIOJ_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 9) )
#define GPIOK_PCLK_DI()	( RCC->AHB1ENR &= ~(1 << 10) )

/*
 * Clock disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI() ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI() ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 18) )
#define UART4_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI() ( RCC->APB1ENR &= ~(1 << 20) )
#define USART6_PCLK_DI() ( RCC->APB2ENR &= ~(1 << 5) )
/*
 * Clock disable macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_DI() ( RCC->APB2ENR &= ~(1 << 14) )

/*
 * Macros to reset GPIOx peripherals
 * This function sets and resets the reset register bit
 * do...while...condition(0) loop is a 'c' technique to execute multiple 'c' statements using a single 'c' macro
 */
#define GPIOA_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOB_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOC_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOD_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOE_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOF_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOG_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOH_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOI_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOJ_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)
#define GPIOK_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); } while(0)

/* Some Generic Macros */
#define ENABLE			1
#define	DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	ENABLE
#define GPIO_PIN_RESET	DISABLE


#include "stm32f429_gpio_driver.h"


#endif /* INC_STM32F429_H_ */
