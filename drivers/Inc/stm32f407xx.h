/*
 * stm32f407xx.h
 *
 *  Created on: May 13, 2025
 *      Author: zaman
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>


// Base address of memory region
#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0x20000000U
#define SRAM2_BASEADDR 0x2001C000U
#define SRAM SRAM1_BASEADDR
#define ROM_BASEADDR 0x1FFF0000U

// Base address of different bus domain
#define APB1_PERI_BASEADDR 0x40000000U
#define APB2_PERI_BASEADDR 0x40010000U
#define AHB1_PERI_BASEADDR 0x40020000U
#define AHB2_PERI_BASEADDR 0x50000000U
#define AHB3_PERI_BASEADDR 0xA0000000U

// Base address of AHB1 bus peripherals
#define GPIOA_BASEADDR (AHB1_PERI_BASEADDR + 0x0000)
#define GPIOB_BASEADDR (AHB1_PERI_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1_PERI_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1_PERI_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1_PERI_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1_PERI_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1_PERI_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1_PERI_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR (AHB1_PERI_BASEADDR + 0x2000)
#define RCC_BASEADDR   (AHB1_PERI_BASEADDR + 0x3800)

// Base address of APB1 bus peripherals
#define I2C1_BASEADDR (APB1_PERI_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1_PERI_BASEADDR + 0x5800)
#define I2C3_BASEADDR (APB1_PERI_BASEADDR + 0x5C00)

#define SPI2_BASEADDR (APB1_PERI_BASEADDR + 0x3800)
#define SPI3_BASEADDR (APB1_PERI_BASEADDR + 0x3C00)

#define USART2_BASEADDR (APB1_PERI_BASEADDR + 0x4400)
#define USART3_BASEADDR (APB1_PERI_BASEADDR + 0x4800)
#define UART4_BASEADDR  (APB1_PERI_BASEADDR + 0x4C00)
#define UART5_BASEADDR  (APB1_PERI_BASEADDR + 0x5000)

// Base address of APB2 bus peripherals
#define SPI1_BASEADDR   (APB2_PERI_BASEADDR + 0x3000)
#define USART1_BASEADDR (APB2_PERI_BASEADDR + 0x1000)
#define USART6_BASEADDR (APB2_PERI_BASEADDR + 0x1400)
#define EXTI_BASEADDR   (APB2_PERI_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR (APB2_PERI_BASEADDR + 0x3800)


// Peripheral register address

typedef struct {
	volatile uint32_t MODER; 		// GPIO port mode register; address offset 0x00
	volatile uint32_t OTYPER;		// GPIO port output type register; address offset 0x04
	volatile uint32_t OSPEEDR;		// GPIO port output speed register; address offset 0x08
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register; address offset 0x0C
	volatile uint32_t IDR;			// GPIO port input data register; address offset 0x10
	volatile uint32_t ODR;			// GPIO port output data register; address offset 0x14
	volatile uint32_t BSRR;			// GPIO port bit set/reset register; address offset 0x18
	volatile uint32_t LCKR;			// GPIO port configuration lock register; address offset 0x1C
	volatile uint32_t AFR[2];		// GPIO alternate function register; address offset 0x20 - 0x24

} gpio_reg_def_t;


#define GPIOA 	((gpio_reg_def_t*)(GPIOA_BASEADDR))
#define GPIOB 	((gpio_reg_def_t*)(GPIOB_BASEADDR))
#define GPIOC 	((gpio_reg_def_t*)(GPIOC_BASEADDR))
#define GPIOD 	((gpio_reg_def_t*)(GPIOD_BASEADDR))
#define GPIOE 	((gpio_reg_def_t*)(GPIOE_BASEADDR))
#define GPIOF 	((gpio_reg_def_t*)(GPIOF_BASEADDR))
#define GPIOG 	((gpio_reg_def_t*)(GPIOG_BASEADDR))
#define GPIOH 	((gpio_reg_def_t*)(GPIOH_BASEADDR))
#define GPIOI 	((gpio_reg_def_t*)(GPIOI_BASEADDR))


/*
 * peripheral register definition structure for RCC
 */
typedef struct
{                                 //
  volatile uint32_t CR;           // RCC clock control register; Address offset: 0x00 */
  volatile uint32_t PLLCFGR;      // RCC PLL configuration register; Address offset: 0x04 */
  volatile uint32_t CFGR;         // RCC clock configuration register; Address offset: 0x08 */
  volatile uint32_t CIR;          // RCC clock interrupt register; Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;     // RCC AHB1 peripheral reset register; Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;     // RCC AHB2 peripheral reset register; Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;     // RCC AHB3 peripheral reset register; Address offset: 0x18 */
  uint32_t      RESERVED0;        //                   */
  volatile uint32_t APB1RSTR;     // RCC APB1 peripheral reset register; Address offset: 0x20 */
  volatile uint32_t APB2RSTR;     // RCC APB2 peripheral reset register; Address offset: 0x24 */
  uint32_t      RESERVED1[2];     //                   */
  volatile uint32_t AHB1ENR;      // RCC AHB1 peripheral clock enable register; Address offset: 0x30 */
  volatile uint32_t AHB2ENR;      // RCC AHB2 peripheral clock enable register; Address offset: 0x34 */
  volatile uint32_t AHB3ENR;      // RCC AHB3 peripheral clock enable register; Address offset: 0x38 */
  uint32_t      RESERVED2;        //                   */
  volatile uint32_t APB1ENR;      // RCC APB1 peripheral clock enable register; Address offset: 0x40 */
  volatile uint32_t APB2ENR;      // RCC APB2 peripheral clock enable register; Address offset: 0x44 */
  uint32_t      RESERVED3[2];     //                   */
  volatile uint32_t AHB1LPENR;    // RCC AHB1 peripheral clock enable in low power mode register; Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;    // RCC AHB2 peripheral clock enable in low power mode register; Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;    // RCC AHB3 peripheral clock enable in low power mode register; Address offset: 0x58 */
  uint32_t      RESERVED4;        //                   */
  volatile uint32_t APB1LPENR;    // RCC APB1 peripheral clock enable in low power mode register; Address offset: 0x60 */
  volatile uint32_t APB2LPENR;    // RCC APB2 peripheral clock enabled in low power mode; Address offset: 0x64 */
  uint32_t      RESERVED5[2];     //                   */
  volatile uint32_t BDCR;         // RCC Backup domain control register; Address offset: 0x70 */
  volatile uint32_t CSR;          // RCC clock control & status register; Address offset: 0x74 */
  uint32_t      RESERVED6[2];     //                   */
  volatile uint32_t SSCGR;        // RCC spread spectrum clock generation register; Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;   // RCC PLLI2S configuration register; Address offset: 0x84 */

} rcc_reg_def_t;

#define RCC 	((rcc_reg_def_t*)RCC_BASEADDR)

/*
 * Clock enable macros for GPIOs peripherals
 */

#define GPIOA_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS() 		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */

/*
 * GPIOx registers reset Macros
 */

#define GPIOA_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET() 		do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)




// Generic Macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET




#endif /* INC_STM32F407XX_H_ */
