/*
 * stm32f3xx_spi_driver.c
 *
 *  Created on: May 17, 2025
 *      Author: v
 */

#include "stm32f3xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){
	if (EnOrDi == ENABLE)
	    {
	        if (pSPIx == SPI1)
	        	SPI1_PCLK_EN();
	        else if (pSPIx == SPI2)
	        	SPI2_PCLK_EN();
	        else if (pSPIx == SPI3)
	        	SPI3_PCLK_EN();
	    }
	    else
	    {
	        if (pSPIx == SPI1)
	        	SPI1_PCLK_DI();
	        else if (pSPIx == SPI2)
	        	SPI2_PCLK_DI();
	        else if (pSPIx == SPI3)
	        	SPI3_PCLK_DI();
	    }
}


void SPI_Init(SPI_Handle_t *pSPIHandle){

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t temp = 0;
	//1 config the mode
	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR ;

	//2 config the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		temp |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |=  (1 << SPI_CR1_RXONLY);
	}

	//3 config Data Frame Format
	temp |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_CRCL ;

	//4 config Clock Phase (CPHA)
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA ;

	//5 config Clock Polarity (CPOL)
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL ;

	//6 config SPI Serial Clock Speed SPI_SclkSpeed
	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR ;

	pSPIHandle->pSPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
	 if (pSPIx == SPI1)
		 SPI1_REG_RESET();
	 else if (pSPIx == SPI2)
		 SPI2_REG_RESET();
	 else if (pSPIx == SPI3)
		 SPI3_REG_RESET();
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		while(!(pSPIx->SR & (1 << 1)));
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL)){
			//16
			pSPIx->DR = *((uint16_t*)(pTxBuffer));
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
