/*
 * stm32f3xx.h
 *
 *  Created on: May 8, 2025
 *      Author: v
 */

#ifndef INC_STM32F3XX_H_
#define INC_STM32F3XX_H_

#include <stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4


/*
 * Base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR   0x08000000U  // Base address of FLASH memory
#define SRAM_BASEADDR    0x20000000U  // Base address of SRAM
#define ROM_BASEADDR     0x1FFFD800U  // Base address of system memory (boot ROM)

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR       0x40000000U
#define APB1PERIPH_BASEADDR   PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR   0x40010000U
#define AHB1PERIPH_BASEADDR   0x40020000U
#define AHB2PERIPH_BASEADDR   0x48000000U
#define AHB3PERIPH_BASEADDR   0x50000000U

/* GPIO base addresses */
#define GPIOA_BASEADDR   (AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR   (AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR   (AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR   (AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR   (AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR   (AHB2PERIPH_BASEADDR + 0x1400)

#define RCC_BASEADDR     (AHB1PERIPH_BASEADDR + 0x1000)

/* Peripheral base addresses on APB1 and APB2 */
#define I2C1_BASEADDR    (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR    (APB1PERIPH_BASEADDR + 0x5800)

#define SPI1_BASEADDR    (APB2PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR    (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR    (APB1PERIPH_BASEADDR + 0x3C00)

#define USART1_BASEADDR  (APB2PERIPH_BASEADDR + 0x3800)
#define USART2_BASEADDR  (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR  (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR   (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR   (APB1PERIPH_BASEADDR + 0x5000)

#define EXTI_BASEADDR    (APB2PERIPH_BASEADDR + 0x0400)
#define SYSCFG_BASEADDR  (APB2PERIPH_BASEADDR + 0x0000)

/* GPIO register definition structure */
typedef struct {
    __vo uint32_t MODER;
    __vo uint32_t OTYPER;
    __vo uint32_t OSPEEDR;
    __vo uint32_t PUPDR;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t LCKR;
    __vo uint32_t AFR[2];
    __vo uint32_t BRR;
} GPIO_RegDef_t;

/* RCC register definition structure */
typedef struct {
    __vo uint32_t CR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    __vo uint32_t AHBENR;
    __vo uint32_t APB2ENR;
    __vo uint32_t APB1ENR;
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    __vo uint32_t AHBRSTR;
    __vo uint32_t CFGR2;
    __vo uint32_t CFGR3;
} RCC_RegDef_t;

/* EXTI register definition structure */
typedef struct {
    __vo uint32_t IMR1;
    __vo uint32_t EMR1;
    __vo uint32_t RTSR1;
    __vo uint32_t FTSR1;
    __vo uint32_t SWIER1;
    __vo uint32_t PR1;
    __vo uint32_t IMR2;
    __vo uint32_t EMR2;
    __vo uint32_t RTSR2;
    __vo uint32_t FTSR2;
    __vo uint32_t SWIER2;
    __vo uint32_t PR2;
} EXTI_RegDef_t;

/* SPI register definition structure */
typedef struct {
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRCR;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;
} SPI_RegDef_t;

/* SYSCFG register definition structure */
typedef struct {
    __vo uint32_t CFGR1;
    __vo uint32_t RCR;
    __vo uint32_t CFGR2;
    __vo uint32_t EXTICR[4];
    __vo uint32_t CFGR3;
    __vo uint32_t CFGR4;
} SYSCFG_RegDef_t;

/* USART register definition structure */
typedef struct {
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t BRR;
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t CR3;
    __vo uint32_t GTPR;
} USART_RegDef_t;

/* I2C register definition structure */
typedef struct {
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t OAR1;
    __vo uint32_t OAR2;
    __vo uint32_t DR;
    __vo uint32_t SR1;
    __vo uint32_t SR2;
    __vo uint32_t CCR;
    __vo uint32_t TRISE;
    __vo uint32_t FLTR;
} I2C_RegDef_t;

/* Peripheral definitions */
#define GPIOA   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC     ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI    ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG  ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1    ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2    ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3    ((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1    ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2    ((I2C_RegDef_t*)I2C2_BASEADDR)

#define USART1  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4   ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5   ((USART_RegDef_t*)UART5_BASEADDR)


/*
 * Clock Enable Macros for GPIOx Peripheral (STM32F3)
 */

#define GPIOA_PCLK_EN()         (RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN()         (RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN()         (RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN()         (RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN()         (RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN()         (RCC->AHBENR |= (1 << 22))

/*
 * Clock Enable Macros for I2Cx Peripheral (STM32F3)
 */

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))

/*
 * Clock Enable Macros for SPIx Peripheral (STM32F3)
 */

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx Peripheral (STM32F3)
 */

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()        (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()         (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()         (RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable Macro for SYSCFG Peripheral (STM32F3)
 */

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 0))

/*
 * Clock Disable Macros for GPIOx Peripheral (STM32F3)
 */

#define GPIOA_PCLK_DI()         (RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI()         (RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI()         (RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI()         (RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI()         (RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI()         (RCC->AHBENR &= ~(1 << 22))

/*
 * Clock Disable Macros for I2Cx Peripheral (STM32F3)
 */

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))

/*
 * Clock Disable Macros for SPIx Peripheral (STM32F3)
 */

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx Peripheral (STM32F3)
 */

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()         (RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock Disable Macro for SYSCFG Peripheral (STM32F3)
 */

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 0))




/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOE_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); }while(0)
#define GPIOF_REG_RESET()  do{ (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:0)

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()  do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()  do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15)); }while(0)



#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define NVIC_IRQ_PR0       0  // EXTI0_IRQn
#define NVIC_IRQ_PR1       1  // EXTI1_IRQn
#define NVIC_IRQ_PR2       2  // EXTI2_IRQn
#define NVIC_IRQ_PR3       3  // EXTI3_IRQn
#define NVIC_IRQ_PR4       4  // EXTI4_IRQn
#define NVIC_IRQ_PR5       5  // EXTI5_IRQn
#define NVIC_IRQ_PR6       6  // EXTI6_IRQn
#define NVIC_IRQ_PR7       7  // EXTI7_IRQn
#define NVIC_IRQ_PR8       8  // EXTI8_IRQn
#define NVIC_IRQ_PR9       9  // EXTI9_IRQn
#define NVIC_IRQ_PR10     10  // EXTI10_IRQn
#define NVIC_IRQ_PR11     11  // EXTI11_IRQn
#define NVIC_IRQ_PR12     12  // EXTI12_IRQn
#define NVIC_IRQ_PR13     13  // EXTI13_IRQn
#define NVIC_IRQ_PR14     14  // EXTI14_IRQn
#define NVIC_IRQ_PR15     15  // EXTI15_IRQn



#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51

#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39

#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53



/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15



//some generic macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET         RESET
#define FLAG_SET 			SET


/*
 * bit position definitions of SPI peripheral FOR CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15
/*
 * bit position definitions of SPI peripheral FOR CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE 		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14
/*
 * bit position definitions of SPI peripheral FOR SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11





#include "stm32f3xx_spi_driver.h"
#include "stm32f3xx_gpio_driver.h"


#endif /* INC_STM32F3XX_H_ */
