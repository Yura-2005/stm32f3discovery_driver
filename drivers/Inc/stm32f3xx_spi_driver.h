/*
 * stm32f3xx_spi_driver.h
 *
 *  Created on: May 17, 2025
 *      Author: v
 */

#include <stdint.h>
#include "stm32f3xx.h"

#ifndef INC_STM32F3XX_SPI_DRIVER_H_
#define INC_STM32F3XX_SPI_DRIVER_H_

// -------------------------------------
// Structure to configure SPI peripheral
// -------------------------------------
typedef struct {
	uint8_t SPI_DeviceMode;    // Device mode: Master or Slave
	uint8_t SPI_BusConfig;     // Bus configuration: Full-Duplex, Half-Duplex, or Simplex RX Only
	uint8_t SPI_DFF;           // Data Frame Format: 8-bit or 16-bit
	uint8_t SPI_CPHA;          // Clock Phase (CPHA): first edge or second edge
	uint8_t SPI_CPOL;          // Clock Polarity (CPOL): idle high or low
	uint8_t SPI_SSM;           // Software Slave Management (SSM): enable or disable
	uint8_t SPI_SclkSpeed;     // SPI Serial Clock Speed (Baud Rate Prescaler)
} SPI_Config_t;

// ----------------------------------------------------
// Handle structure for managing SPI peripheral instance
// ----------------------------------------------------
typedef struct {
	SPI_RegDef_t *pSPIx;       // Pointer to SPI peripheral's register definition (e.g., SPI1, SPI2, etc.)
	SPI_Config_t SPIConfig;    // SPI configuration settings
	uint8_t 	 *pTxBuffer;
	uint8_t 	 *pRxBuffer;
	uint32_t 	 TxLen;
	uint32_t 	 RxLen;
	uint8_t 	 RxState;
	uint8_t 	 TxState;
} SPI_Handle_t;



/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE  			0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 				1
#define SPI_BUS_CONFIG_HD 				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)
#define SPI_CHSIDE_FLAG					(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG					(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG					(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG					(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG					(1 << SPI_SR_OVR)
#define SPI_FRE_FLAG					(1 << SPI_SR_FRE)


#define SPI_READY 						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4

/******************************************************************************************
 *								APIs supported by this driver
 ******************************************************************************************/

/*
 * Clock control
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Initialization / De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ config
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmision(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

void SPI_AplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F3XX_SPI_DRIVER_H_ */
