/*
 * stm32f3xx_timer_driver.h
 *
 *  Created on: Jul 5, 2025
 *      Author: patru
 */

#ifndef INC_STM32F3XX_TIMER_DRIVER_H_
#define INC_STM32F3XX_TIMER_DRIVER_H_

#include "stm32f3xx.h"

typedef struct{
	uint32_t Prescaler;
	uint32_t AutoReload;
	uint8_t  Mode;
	uint8_t  InterruptEnable;
}Timer_Config_t;

typedef struct{
	Timer_RegDef_t *pTIMx;
	Timer_Config_t Timer_Config;
}Timer_Handler_t;

void TIM_PeriClockControl(Timer_RegDef_t *pTIMx, uint8_t EnOrDi);

void TIM_Init(Timer_Handler_t *pTimer);

void TIM_DeInit(Timer_RegDef_t *pTIMx);

void TIM_Start(Timer_RegDef_t *pTIMx);

void TIM_Stop(Timer_RegDef_t *pTIMx);

void TIM_IRQHandling(Timer_RegDef_t *pTIMx);

void NVIC_EnableIRQ(uint8_t IRQNumber);

#define TIMER_MODE_CONTINUOUS    0
#define TIMER_MODE_ONE_PULSE     1


#endif /* INC_STM32F3XX_TIMER_DRIVER_H_ */
