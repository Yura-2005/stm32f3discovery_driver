#include "stm32f3xx_gpio_driver.h"
#include "stm32f3xx_timer_driver.h"

volatile uint8_t direction = 0;
volatile uint8_t direction_last_state = 0;
volatile int8_t current_led_index = 0;

Timer_Handler_t htimer_run;
Timer_Handler_t htimer_debounce;

uint8_t led_pins[8] = {
		GPIO_PIN_NO_8,
		GPIO_PIN_NO_9,
		GPIO_PIN_NO_10,
		GPIO_PIN_NO_11,
		GPIO_PIN_NO_12,
		GPIO_PIN_NO_13,
		GPIO_PIN_NO_14,
		GPIO_PIN_NO_15
};

void delay(uint32_t ms){
	for(__vo int i = 0 ; i < (ms * 1000) ; i++);
}

void initLeds(uint8_t led_pins[]){
	GPIO_Handle_t Led_blink ;

		Led_blink.pGPIOx = GPIOE;

		Led_blink.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;

		Led_blink.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
		Led_blink.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		Led_blink.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

		for(__vo int i = 0 ; i < 8 ; i++){
			Led_blink.GPIO_PinConfig.GPIO_PinNumber = led_pins[i];
			GPIO_Init(&Led_blink);
		}
}

void RunLeds(uint8_t* led_pins, uint8_t direction) {
	GPIO_WriteToOutputPin(GPIOE, led_pins[current_led_index], DISABLE);

	if (direction == 1)
		current_led_index = (current_led_index + 1) % 8;
	else
		current_led_index = (current_led_index - 1 + 8) % 8;

	GPIO_WriteToOutputPin(GPIOE, led_pins[current_led_index], ENABLE);
}


int main() {

	initLeds(led_pins);

	GPIO_WriteToOutputPin(GPIOE, led_pins[current_led_index], ENABLE);

	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_Init(&Button);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR0);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	htimer_run.pTIMx = TIM2;
	htimer_run.Timer_Config.Prescaler = 35999;
	htimer_run.Timer_Config.AutoReload = 499;
	htimer_run.Timer_Config.Mode = TIMER_MODE_CONTINUOUS;
	htimer_run.Timer_Config.InterruptEnable = ENABLE;
	TIM_Init(&htimer_run);
	TIM_Start(htimer_run.pTIMx);
	NVIC_EnableIRQ(TIM2_IRQn);

	htimer_debounce.pTIMx = TIM7;
	htimer_debounce.Timer_Config.Prescaler = 35999;
	htimer_debounce.Timer_Config.AutoReload = 49;
	htimer_debounce.Timer_Config.Mode = TIMER_MODE_ONE_PULSE;
	htimer_debounce.Timer_Config.InterruptEnable = ENABLE;
	TIM_Init(&htimer_debounce);
	NVIC_EnableIRQ(TIM7_IRQn);

	while(1);
}


void EXTI0_IRQHandler(void){
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	EXTI->IMR1 &= ~(1 << 0);
	TIM_Start(htimer_debounce.pTIMx);
}

void TIM2_IRQHandler(void){
	TIM_IRQHandling(htimer_run.pTIMx);
	RunLeds(led_pins, direction);
}

void TIM7_IRQHandler(void) {
	TIM_IRQHandling(htimer_debounce.pTIMx);
	if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == 0) {
		direction ^= 1;
	}
	EXTI->IMR1 |= (1 << 0);
}



