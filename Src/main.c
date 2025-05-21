#include "stm32f3xx_gpio_driver.h"

void delay() {
  for(volatile int i = 0; i < 300000; i++);
}

int main() {
  // Налаштування зеленого світлодіода (PE15)
  GPIO_Handle_t GPIO_Led_GREEN;
  GPIO_Led_GREEN.pGPIOx = GPIOE;
  GPIO_Led_GREEN.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_15;
  GPIO_Led_GREEN.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
  GPIO_Led_GREEN.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
  GPIO_Led_GREEN.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
  GPIO_Led_GREEN.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;

//  // Налаштування зеленого світлодіода (PE14)
//  GPIO_Handle_t GPIO_Led_ORANGE;
//  GPIO_Led_ORANGE.pGPIOx = GPIOE;
//  GPIO_Led_ORANGE.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_14;
//  GPIO_Led_ORANGE.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
//  GPIO_Led_ORANGE.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
//  GPIO_Led_ORANGE.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
//  GPIO_Led_ORANGE.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;
//
//  // Налаштування синього світлодіода (PE12)
//  GPIO_Handle_t GPIO_Led_BLUE;
//  GPIO_Led_BLUE.pGPIOx = GPIOE;
//  GPIO_Led_BLUE.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_12;
//  GPIO_Led_BLUE.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_OUT;
//  GPIO_Led_BLUE.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
//  GPIO_Led_BLUE.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_NO_PUPD;
//  GPIO_Led_BLUE.GPIO_PinConfig.GPIO_PinOPType       = GPIO_OP_TYPE_PP;


  GPIO_Handle_t USER_BUTTON;
  USER_BUTTON.pGPIOx = GPIOA;
  USER_BUTTON.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;
  USER_BUTTON.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
  USER_BUTTON.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
  USER_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;


  // Увімкнення тактування порту E
  GPIO_PeriClockControl(GPIOE, ENABLE);

  // Увімкнення тактування порту A
  GPIO_PeriClockControl(GPIOA, ENABLE);

  // Ініціалізація світлодіодів
  GPIO_Init(&GPIO_Led_GREEN);
//  GPIO_Init(&GPIO_Led_ORANGE);
//  GPIO_Init(&GPIO_Led_BLUE);

  GPIO_Init(&USER_BUTTON);

  GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR0);
  GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

  while(1);
}

void EXTI0_IRQHandler(void){
	if(EXTI->PR1 & (1 << GPIO_PIN_NO_0)){
		GPIO_IRQHandling(GPIO_PIN_NO_0);
		GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_NO_15);
	}

}
