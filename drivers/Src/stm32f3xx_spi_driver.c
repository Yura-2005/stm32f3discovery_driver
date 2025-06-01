/*
 * stm32f3xx_spi_driver.c
 *
 *  Created on: May 17, 2025
 *      Author: v
 */

#include "stm32f3xx_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0){
		while((SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET));
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

void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		while((SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET));
		if(pSPIx->CR1 & (1 << SPI_CR1_CRCL)){
			//16
			*((uint16_t*)(pRxBuffer)) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
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

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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


void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

    volatile uint32_t *priority_reg = (uint32_t *)(NVIC_PR_BASE_ADDR + iprx);
    *priority_reg &= ~(0xFF << (8 * iprx_section));
    *priority_reg |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->TxState;
	if (state != SPI_BUSY_IN_TX){
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX){
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1 , temp2;

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2){
		spi_txe_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2){
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2){
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_CRCL)){
		//16
		pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else{
		//8
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen){
		SPI_CloseTransmision(pSPIHandle);
		SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_CRCL)){
		//16
		pSPIHandle->pSPIx->DR = *((uint16_t*)(pSPIHandle->pRxBuffer));
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else{
		//8
		pSPIHandle->pSPIx->DR = *pSPIHandle->pRxBuffer;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen){
		SPI_CloseReception(pSPIHandle);
		SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_AplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmision(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__weak void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){

}
