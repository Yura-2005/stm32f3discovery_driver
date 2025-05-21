/*
 * spi_tx_testing.c
 *
 *  Created on: May 18, 2025
 *      Author: v
 */
#include "stm32f3xx.h"
#include <string.h>

//PB15 --> SPI2_MOSI
//PB14 --> SPI2_MISO
//PB13 --> SPI2_SCK
//PB12 --> SPI2_NSS
//ALT func. mode : 5

void SPI_GPIOIints(void){
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5 ;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);
//
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void){

	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;

	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;

	SPI_Init(&SPI2Handle);
}

void GPIO_ButtonInit(void){
	  GPIO_Handle_t USER_BUTTON;
	  USER_BUTTON.pGPIOx = GPIOA;
	  USER_BUTTON.GPIO_PinConfig.GPIO_PinNumber       = GPIO_PIN_NO_0;
	  USER_BUTTON.GPIO_PinConfig.GPIO_PinMode         = GPIO_MODE_IT_FT;
	  USER_BUTTON.GPIO_PinConfig.GPIO_PinSpeed        = GPIO_SPEED_LOW;
	  USER_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl  = GPIO_PIN_PU;
	  GPIO_Init(&USER_BUTTON);
}

void delay() {
  for(volatile int i = 0; i < 500000/2; i++);
}

int main(){
	char user_data[] = "Hello World";

	GPIO_ButtonInit();

	SPI_GPIOIints();
	SPI2_Inits();
	SPI_SSOEConfig(SPI2, ENABLE);
	while(1){
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		SPI_PeripheralControl(SPI2, ENABLE);
		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI2, &datalen, 1);
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));
		while((SPI2->SR & (1 << SPI_SR_BSY)));
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}
