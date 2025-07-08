/*
 * stm32f3xx_timer_driver.c
 *
 *  Created on: Jul 5, 2025
 *      Author: patru
 */

#include "stm32f3xx_timer_driver.h"

void TIM_PeriClockControl(Timer_RegDef_t *pTIMx, uint8_t EnOrDi) {
    if (EnOrDi == ENABLE) {
        if (pTIMx == TIM1) {
            TIM1_PCLK_EN();
        } else if (pTIMx == TIM2) {
            TIM2_PCLK_EN();
        } else if (pTIMx == TIM3) {
            TIM3_PCLK_EN();
        } else if (pTIMx == TIM4) {
            TIM4_PCLK_EN();
        } else if (pTIMx == TIM6) {
            TIM6_PCLK_EN();
        } else if (pTIMx == TIM7) {
            TIM7_PCLK_EN();
        } else if (pTIMx == TIM15) {
            TIM15_PCLK_EN();
        } else if (pTIMx == TIM16) {
            TIM16_PCLK_EN();
        } else if (pTIMx == TIM17) {
            TIM17_PCLK_EN();
        }
    } else {
        if (pTIMx == TIM1) {
            TIM1_PCLK_DI();
        } else if (pTIMx == TIM2) {
            TIM2_PCLK_DI();
        } else if (pTIMx == TIM3) {
            TIM3_PCLK_DI();
        } else if (pTIMx == TIM4) {
            TIM4_PCLK_DI();
        } else if (pTIMx == TIM6) {
            TIM6_PCLK_DI();
        } else if (pTIMx == TIM7) {
            TIM7_PCLK_DI();
        } else if (pTIMx == TIM15) {
            TIM15_PCLK_DI();
        } else if (pTIMx == TIM16) {
            TIM16_PCLK_DI();
        } else if (pTIMx == TIM17) {
            TIM17_PCLK_DI();
        }
    }
}

void TIM_Init(Timer_Handler_t *pTimer){
	TIM_PeriClockControl(pTimer->pTIMx, ENABLE);

	pTimer->pTIMx->PSC = pTimer->Timer_Config.Prescaler;
	pTimer->pTIMx->ARR = pTimer->Timer_Config.AutoReload;

	pTimer->pTIMx->CNT = 0;

	if (pTimer->Timer_Config.Mode == TIMER_MODE_ONE_PULSE) {
		pTimer->pTIMx->CR1 |= (1 << 3);
	} else {
		pTimer->pTIMx->CR1 &= ~(1 << 3);
	}

	if (pTimer->Timer_Config.InterruptEnable){
		pTimer->pTIMx->DIER |= (1 << 0);
	}

	pTimer->pTIMx->CR1 |= (1 << 0);
}

void TIM_DeInit(Timer_RegDef_t *pTIMx){
	if (pTIMx == TIM1) {
		TIM1_REG_RESET();
	} else if (pTIMx == TIM2) {
		TIM2_REG_RESET();
	} else if (pTIMx == TIM3) {
		TIM3_REG_RESET();
	} else if (pTIMx == TIM4) {
		TIM4_REG_RESET();
	} else if (pTIMx == TIM6) {
		TIM6_REG_RESET();
	} else if (pTIMx == TIM7) {
		TIM7_REG_RESET();
	} else if (pTIMx == TIM15) {
		TIM15_REG_RESET();
	} else if (pTIMx == TIM16) {
		TIM16_REG_RESET();
	} else if (pTIMx == TIM17) {
		TIM17_REG_RESET();
	}
}

void TIM_Start(Timer_RegDef_t *pTIMx){
	pTIMx->CR1 |= (1 << 0);
}

void TIM_Stop(Timer_RegDef_t *pTIMx){
	pTIMx->CR1 &= ~(1 << 0);
}

void TIM_IRQHandling(Timer_RegDef_t *pTIMx){
	pTIMx->SR &= ~(1 << 0);
}

void NVIC_EnableIRQ(uint8_t IRQNumber) {
    if (IRQNumber < 32) {
        *NVIC_ISER0 |= (1 << IRQNumber);
    } else if (IRQNumber < 64) {
        *NVIC_ISER1 |= (1 << (IRQNumber % 32));
    }
}
