/*
 * stm32f030xx.h
 *
 *  Created on: Dec 31, 2023
 *      Author: Aditya
 */

#ifndef INC_STM32F030XX_H_
#define INC_STM32F030XX_H_

#include <stdint.h>

#define __vo volatile

/* defining flash and SRAM base address */

#define FLASH_BASEADDR 0x08000000U /* have to explicitly add 'U' otherwise compiler treats it as signed */
#define SRAM_BASEADDR  0x20000000U /* SRAM base address */
#define ROM_BASEADDR   0x1FFFECC0U /* system memory (or) ROM base address */

/* defining base address of AHP, APB and peripherals */

#define APB_BASEADDR   0x40000000U
#define AHB1_BASEADDR  0x40020000U
#define AHB2_BASEADDR  0x48000000U

/* defining all peripherals hanging on AHB2 bus */

#define GPIOA_BASEADDR  (AHB2_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR  (AHB2_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR  (AHB2_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR  (AHB2_BASEADDR + 0x0C00U)
#define GPIOF_BASEADDR  (AHB2_BASEADDR + 0x1400U)

/* defining all peripherals hanging on APB bus */

#define I2C1_BASEADDR   (APB_BASEADDR + 0x5400U)
#define I2C2_BASEADDR	(APB_BASEADDR + 0x5800U)

#define SPI1_BASEADDR	(APB_BASEADDR + 0x13000U)
#define SPI2_BASEADDR	(APB_BASEADDR + 0x3800U)

#define USART1_BASEADDR	(APB_BASEADDR + 0x13800U)
#define USART2_BASEADDR	(APB_BASEADDR + 0x4400U)
#define USART3_BASEADDR	(APB_BASEADDR + 0x4800U)
#define USART4_BASEADDR	(APB_BASEADDR + 0x4C00U)
#define USART5_BASEADDR	(APB_BASEADDR + 0x5000U)
#define USART6_BASEADDR	(APB_BASEADDR + 0x11400U)

#define EXTI_BASEADDR	(APB_BASEADDR + 0x10400U)
#define SYSCFG_BASEADDR	(APB_BASEADDR + 0x10000U)

#define RCC_BASEADDR 	(AHB1_BASEADDR + 0x1000U)

/* peripheral register definition structures */

typedef struct {
	__vo uint32_t MODER; 	/* address offset : 0x00 */
	__vo uint32_t OTYPER; 	/* address offset : 0x04 */
	__vo uint32_t OSPEEDR; 	/* address offset : 0x08 */
	__vo uint32_t PUPDR; 	/* address offset : 0x0C */
	__vo uint32_t IDR;	 	/* address offset : 0x10 */
	__vo uint32_t ODR;	 	/* address offset : 0x14 */
	__vo uint32_t BSRR;	 	/* address offset : 0x18 */
	__vo uint32_t LCKR;	 	/* address offset : 0x1C */
	__vo uint32_t AFRL;	 	/* address offset : 0x20 */
	__vo uint32_t AFRH;	 	/* address offset : 0x24 */
	__vo uint32_t BRR;	 	/* address offset : 0x28 */
} GPIO_RegDef_t;

typedef struct {
	__vo uint32_t RCC_CR; 			/* address offset : 0x00 */
	__vo uint32_t RCC_CFGR; 		/* address offset : 0x04 */
	__vo uint32_t RCC_CIR;	 		/* address offset : 0x08 */
	__vo uint32_t RCC_APB2RSTR; 	/* address offset : 0x0C */
	__vo uint32_t RCC_APB1RSTR; 	/* address offset : 0x10 */
	__vo uint32_t RCC_AHBENR;	 	/* address offset : 0x14 */
	__vo uint32_t RCC_APB2ENR;	 	/* address offset : 0x18 */
	__vo uint32_t RCC_APB1ENR;	 	/* address offset : 0x1C */
	__vo uint32_t RCC_BDCR;	 		/* address offset : 0x20 */
	__vo uint32_t RCC_CSR;	 		/* address offset : 0x24 */
	__vo uint32_t RCC_AHBRSTR;	 	/* address offset : 0x28 */
	__vo uint32_t RCC_CFGR2;	 	/* address offset : 0x2C */
	__vo uint32_t RCC_CFGR3;	 	/* address offset : 0x30 */
	__vo uint32_t RCC_CR2;	 		/* address offset : 0x34 */
} RCC_RegDef_t;

/* peripheral definitions */

#define GPIOA ((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) GPIOF_BASEADDR)

#define RCC ((RCC_RegDef_t*) RCC_BASEADDR)

/* Clock enable macros for GPIO peripherals */

#define GPIOA_PCLK_EN()	(RCC->RCC_AHBENR |= (1u << 17))
#define GPIOB_PCLK_EN()	(RCC->RCC_AHBENR |= (1u << 18))
#define GPIOC_PCLK_EN()	(RCC->RCC_AHBENR |= (1u << 19))
#define GPIOD_PCLK_EN()	(RCC->RCC_AHBENR |= (1u << 20))
#define GPIOF_PCLK_EN()	(RCC->RCC_AHBENR |= (1u << 22))

/* Clock enable macros for I2Cx peripherals */

#define I2C1_PCLK_EN()	(RCC->RCC_APB1ENR |= (1u << 21))
#define I2C2_PCLK_EN()	(RCC->RCC_APB1ENR |= (1u << 22))

/* Clock enable macros for SPIx peripherals */

#define SPI1_PCLK_EN()	(RCC->RCC_APB2ENR  |= (1u << 12))
#define SPI2_PCLK_EN()	(RCC->RCC_APB1ENR  |= (1u << 14))

/* Clock enable macros for USARTx peripherals */

#define USART1_PCLK_EN()	(RCC->RCC_APB2ENR  |= (1u << 14))
#define USART2_PCLK_EN()	(RCC->RCC_APB1ENR  |= (1u << 17))
#define USART3_PCLK_EN()	(RCC->RCC_APB1ENR  |= (1u << 18))
#define USART4_PCLK_EN()	(RCC->RCC_APB1ENR  |= (1u << 19))
#define USART5_PCLK_EN()	(RCC->RCC_APB1ENR  |= (1u << 20))
#define USART6_PCLK_EN()	(RCC->RCC_APB2ENR  |= (1u << 5))

/* Clock enable macros for SYSCFG peripherals */

#define SYSCFG_PCLK_EN()	(RCC->RCC_APB2ENR  |= (1u << 0))

/* Clock disable macros for GPIO peripherals */

#define GPIOA_PCLK_DI()	(RCC->RCC_AHBENR &= ~(1u << 17))
#define GPIOB_PCLK_DI()	(RCC->RCC_AHBENR &= ~(1u << 18))
#define GPIOC_PCLK_DI()	(RCC->RCC_AHBENR &= ~(1u << 19))
#define GPIOD_PCLK_DI()	(RCC->RCC_AHBENR &= ~(1u << 20))
#define GPIOF_PCLK_DI()	(RCC->RCC_AHBENR &= ~(1u << 22))

/* Clock disable macros for I2Cx peripherals */

#define I2C1_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1u << 21))
#define I2C2_PCLK_DI()	(RCC->RCC_APB1ENR &= ~(1u << 22))

/* Clock disable macros for SPIx peripherals */

#define SPI1_PCLK_DI()	(RCC->RCC_APB2ENR  &= ~(1u << 12))
#define SPI2_PCLK_DI()	(RCC->RCC_APB1ENR  &= ~(1u << 14))

/* Clock disable macros for USARTx peripherals */

#define USART1_PCLK_DI()	(RCC->RCC_APB2ENR  &= ~(1u << 14))
#define USART2_PCLK_DI()	(RCC->RCC_APB1ENR  &= ~(1u << 17))
#define USART3_PCLK_DI()	(RCC->RCC_APB1ENR  &= ~(1u << 18))
#define USART4_PCLK_DI()	(RCC->RCC_APB1ENR  &= ~(1u << 19))
#define USART5_PCLK_DI()	(RCC->RCC_APB1ENR  &= ~(1u << 20))
#define USART6_PCLK_DI()	(RCC->RCC_APB2ENR  &= ~(1u << 5))

/* Clock disable macros for SYSCFG peripherals */

#define SYSCFG_PCLK_DI()	(RCC->RCC_APB2ENR  &= ~(1u << 0))

/* Macros to reset GPIO peripherals */

#define GPIOA_REG_RESET()			(RCC->RCC_AHBRSTR |= (1u << 17))
#define GPIOA_REG_RESET_CLEAR()		(RCC->RCC_AHBRSTR &= ~(1u << 17))

#define GPIOB_REG_RESET()			(RCC->RCC_AHBRSTR |= (1u << 18))
#define GPIOB_REG_RESET_CLEAR()		(RCC->RCC_AHBRSTR &= ~(1u << 18))

#define GPIOC_REG_RESET()			(RCC->RCC_AHBRSTR |= (1u << 19))
#define GPIOC_REG_RESET_CLEAR()		(RCC->RCC_AHBRSTR &= ~(1u << 19))

#define GPIOD_REG_RESET()			(RCC->RCC_AHBRSTR |= (1u << 20))
#define GPIOD_REG_RESET_CLEAR()		(RCC->RCC_AHBRSTR &= ~(1u << 20))

#define GPIOF_REG_RESET()			(RCC->RCC_AHBRSTR |= (1u << 22))
#define GPIOF_REG_RESET_CLEAR()		(RCC->RCC_AHBRSTR &= ~(1u << 22))

/* defining enable and disable macros */

#define ENABLE  		1u
#define DISABLE 		0u
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#endif /* INC_STM32F030XX_H_ */
