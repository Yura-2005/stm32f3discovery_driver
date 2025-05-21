#include "stm32f3xx_gpio_driver.h"
#define MODE_OFF     0
#define MODE_FAST    1
#define MODE_SLOW    2
#define MODE_ON      3

volatile uint32_t status = 0 ;

void delay(uint32_t amount){
	for(volatile int i = 0; i < (amount) ;i++);
}

int main() {

	GPIO_Handle_t Led_Blue1;
	Led_Blue1.pGPIOx = GPIOE;
	Led_Blue1.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	Led_Blue1.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	Led_Blue1.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Led_Blue1.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	Led_Blue1.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOE, ENABLE);
	GPIO_Init(&Led_Blue1);


	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOA;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Init(&Button);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PR0);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

  while(1){
	  switch(status){
	  	case MODE_FAST:
	  		GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_NO_8);
	  		delay(100000);
	  		break;
	  	case MODE_SLOW:
	  		GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_NO_8);
	  		delay(1000000);
	  		break;
	  	case MODE_ON:
	  		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NO_8, ENABLE);
	  		break;
	  	case MODE_OFF:
	  	default:
	  		GPIO_WriteToOutputPin(GPIOE, GPIO_PIN_NO_8, DISABLE);
	  		break;
	  }


  }

}

void EXTI0_IRQHandler(void){
	delay(30000);
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	delay(30000);
	status = (status + 1) % 4;
}





