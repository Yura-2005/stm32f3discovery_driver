/*
 * stm32f3xx_gpio_driver.c
 *
 *  Created on: May 8, 2025
 *      Author: v
 */


#include "stm32f3xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
            GPIOA_PCLK_EN();
        else if (pGPIOx == GPIOB)
            GPIOB_PCLK_EN();
        else if (pGPIOx == GPIOC)
            GPIOC_PCLK_EN();
        else if (pGPIOx == GPIOD)
            GPIOD_PCLK_EN();
        else if (pGPIOx == GPIOF)
            GPIOF_PCLK_EN();
        else if (pGPIOx == GPIOE)
            GPIOE_PCLK_EN();
    }
    else
    {
        if (pGPIOx == GPIOA)
            GPIOA_PCLK_DI();
        else if (pGPIOx == GPIOB)
            GPIOB_PCLK_DI();
        else if (pGPIOx == GPIOC)
            GPIOC_PCLK_DI();
        else if (pGPIOx == GPIOD)
            GPIOD_PCLK_DI();
        else if (pGPIOx == GPIOF)
            GPIOF_PCLK_DI();
        else if (pGPIOx == GPIOE)
            GPIOE_PCLK_DI();
    }
}

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    uint8_t pinno = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    if (pinno > 15) return; // недопустимий пін

    // 1. Configure the mode
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // Non-interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pinno));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pinno)); // clear
        pGPIOHandle->pGPIOx->MODER |= temp; // set
    }
    else
    {
        // Interrupt mode
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        // 1. Enable SYSCFG clock
        SYSCFG_PCLK_EN();

        // 2. Configure SYSCFG_EXTICR
        SYSCFG->EXTICR[pinno / 4] &= ~(0xF << (4 * (pinno % 4)));
        SYSCFG->EXTICR[pinno / 4] |= (portcode << (4 * (pinno % 4)));

        // 3. Configure trigger
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            if (pinno < 32) {
                EXTI->FTSR1 |= (1 << pinno);
                EXTI->RTSR1 &= ~(1 << pinno);
            } else {
                EXTI->FTSR2 |= (1 << (pinno - 32));
                EXTI->RTSR2 &= ~(1 << (pinno - 32));
            }
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            if (pinno < 32) {
                EXTI->RTSR1 |= (1 << pinno);
                EXTI->FTSR1 &= ~(1 << pinno);
            } else {
                EXTI->RTSR2 |= (1 << (pinno - 32));
                EXTI->FTSR2 &= ~(1 << (pinno - 32));
            }
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            if (pinno < 32) {
                EXTI->RTSR1 |= (1 << pinno);
                EXTI->FTSR1 |= (1 << pinno);
            } else {
                EXTI->RTSR2 |= (1 << (pinno - 32));
                EXTI->FTSR2 |= (1 << (pinno - 32));
            }
        }

        // Enable interrupt mask
        if (pinno < 32)
            EXTI->IMR1 |= (1 << pinno);
        else
            EXTI->IMR2 |= (1 << (pinno - 32));
    }

    // 2. Configure the speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pinno));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pinno));
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    // 3. Configure the pull-up/pull-down resistor
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pinno));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pinno));
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // 4. Configure the output type
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pinno);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinno);
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    // 5. Configure alternate function (if needed)
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t afr_index = pinno / 8;
        uint8_t afr_pos = (pinno % 8) * 4;

        pGPIOHandle->pGPIOx->AFR[afr_index] &= ~(0xF << afr_pos);
        pGPIOHandle->pGPIOx->AFR[afr_index] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << afr_pos);
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
        GPIOA_REG_RESET();
    else if (pGPIOx == GPIOB)
        GPIOB_REG_RESET();
    else if (pGPIOx == GPIOC)
        GPIOC_REG_RESET();
    else if (pGPIOx == GPIOD)
        GPIOD_REG_RESET();
    else if (pGPIOx == GPIOF)
        GPIOF_REG_RESET();
    else if (pGPIOx == GPIOE)
        GPIOE_REG_RESET();
}

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)pGPIOx->IDR;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
        pGPIOx->ODR |= (1 << PinNumber);
    else
        pGPIOx->ODR &= ~(1 << PinNumber);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber < 32)
            *NVIC_ISER0 |= (1 << IRQNumber);
        else if (IRQNumber < 64)
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        else if (IRQNumber < 96)
            *NVIC_ISER2 |= (1 << (IRQNumber % 32));
        // Додай NVIC_ISER3 при потребі
    }
    else
    {
        if (IRQNumber < 32)
            *NVIC_ICER0 |= (1 << IRQNumber);
        else if (IRQNumber < 64)
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        else if (IRQNumber < 96)
            *NVIC_ICER2 |= (1 << (IRQNumber % 32));
    }
}


void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    volatile uint32_t *priority_reg = (uint32_t *)(NVIC_PR_BASE_ADDR + iprx);
    *priority_reg &= ~(0xFF << (8 * iprx_section));
    *priority_reg |= (IRQPriority << shift_amount);
}


void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (PinNumber < 32)
    {
        if (EXTI->PR1 & (1 << PinNumber))
        {
            EXTI->PR1 |= (1 << PinNumber); // Clear by writing 1
        }
    }
    else
    {
        if (EXTI->PR2 & (1 << (PinNumber - 32)))
        {
            EXTI->PR2 |= (1 << (PinNumber - 32)); // Clear
        }
    }
}

