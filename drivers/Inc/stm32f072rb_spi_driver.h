/*
 * stm32f072rb_spi_driver.h
 *
 *  Created on: Jun. 7, 2020
 *      Author: Nidhin Easo Thomas
 */

#ifndef INC_STM32F072RB_SPI_DRIVER_H_
#define INC_STM32F072RB_SPI_DRIVER_H_

#include<stm32f072rb.h>


/*
 * SPI Device Modes(SPI_DeviceMode)
 *
 */
#define SPI_DEVICE_MODE_MASTER 1				/* SPI Device Mode: Master */
#define SPI_DEVICE_MODE_SLAVE 0					/* SPI Device Mode: Slave */

/*
 * SPI Bus Configuration (SPI_BusConfig)
 *
 */

#define SPI_BUS_CONFIG_FD			1			/* SPI Bus in Full Duplex configuration */
#define SPI_BUS_CONFIG_HD			2			/* SPI Bus in Half Duplex configuration */
#define SPI_BUS_CONFIG_S_RXONLY		3			/* SPI Bus in Simplex configuration with Reception enabled */



/*
 * SPI Clock Speed(SPI_Speed)
 *
 */
#define SPI_CLK_SPEED_DIV2				0		/* SPI Clock prescaler set to divide by 2 */
#define SPI_CLK_SPEED_DIV4				1		/* SPI Clock prescaler set to divide by 4 */
#define SPI_CLK_SPEED_DIV8				2		/* SPI Clock prescaler set to divide by 8 */
#define SPI_CLK_SPEED_DIV16				3		/* SPI Clock prescaler set to divide by 16 */
#define SPI_CLK_SPEED_DIV32				4		/* SPI Clock prescaler set to divide by 32 */
#define SPI_CLK_SPEED_DIV64				5		/* SPI Clock prescaler set to divide by 64 */
#define SPI_CLK_SPEED_DIV128			6		/* SPI Clock prescaler set to divide by 128 */
#define SPI_CLK_SPEED_DIV256			7		/* SPI Clock prescaler set to divide by 256 */

/*
 *
 * SPI Data frame formats(SPI_DFF)
 */
#define SPI_DFF_8BIT		7					/* SPI Data Frame Format: 8 Bit */
#define SPI_DFF_16BIT		15					/* SPI Data Frame Format: 16 Bit */

/*
 * SPI Clock Polarity(SPI_CPOL)
 *
 *
 */
#define SPI_CPOL_HIGH 1							/* SPI Clock Polarity: High */
#define SPI_CPOL_LOW 0							/* SPI Clock Polarity: Low */



/*
 * SPI Clock Phase(SPI_CPHA)
 *
 *
 */

#define SPI_CPHA_HIGH 1							/* SPI Clock Phase: High */
#define SPI_CPHA_LOW 0							/* SPI Clock Phase: Low */



/*
 * SPI Software Slave Management(SPI_SSM)
 *
 *
 */
#define SPI_SSM_DIS	0							/* Slave Management done by Hardware */
#define SPI_SSM_EN	1							/* Slave Management done by Software */


#define SPI_TXE_FLAG	(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1<<SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG	(1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG	(1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG	(1<<SPI_SR_CRCERR)
#define SPI_MODEF_FLAG	(1<<SPI_SR_MODF)
#define SPI_OVR_FLAG	(1<<SPI_SR_OVR)
#define SPI_FRE_FLAG	(1<<SPI_SR_FRE)
#define SPI_BSY_FLAG	(1<<SPI_SR_BSY)

/* Possible SPI application state */
#define SPI_RDY			0
#define SPI_BSY_TX		1
#define SPI_BSY_RX		2


/* Possible SPI application events */
#define SPI_EVNT_TX_CMPLT 	1
#define SPI_EVNT_RX_CMPLT 	2
#define SPI_EVNT_OVR_ERR	3

/* Configuration Structure for SPI */


typedef struct
{
	uint8_t SPI_DeviceMode;						/* SPI Device Mode; */
	uint8_t SPI_BusConfig;						/* SPI Bus Configuration */
	uint8_t SPI_DFF;							/* SPI Data Frame Format */
	uint8_t SPI_CPHA;							/* SPI Clock Phase */
	uint8_t SPI_CPOL;							/* SPI Clock Polarity */
	uint8_t SPI_SSM;							/* SPI Software Slave Management; */
	uint8_t SPI_Speed;							/* SPI CLock Speed; */
}SPI_PinConfig_t;

typedef struct
{
	SPI_RegDef_t* pSPIx;						/* Holds the base address for SPI to be configured */
	SPI_PinConfig_t SPI_PinCOnfig;				/* Holds the required Pin configuration */
	uint8_t *pTxBuffer;							/* Stores the application Transmission Buffer Address */
	uint8_t *pRxBuffer;							/* Store the application Receive Buffer Address*/
	uint32_t TxLen;								/* Stores Transmission Data Length */
	uint32_t RxLen;								/* Stores Receive Data Length */
	uint8_t TxState;							/* Stores Tx State */
	uint8_t RxState;							/* Stores Rx State */
}SPI_Handle_t;

/*
 * *********************************************************************************
 * 										SPI APIs supported by this driver
 * 								For More information, check the API definition
 * *********************************************************************************
 */

/* SPI Clock setup*/
void SPI_PeriphClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);

/* SPI Initialize/Uninitialize */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/* Data transmission */


void SPI_SendData(SPI_RegDef_t* pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t* pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/* SPI IRQ handling*/
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDis);
void SPI_IRQPriorityConfig(uint8_t IRQNum,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/* Non Blocking APIs - Send and Receive Data */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDatIT(SPI_Handle_t* pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


/* Peripheral Control APIs */
void SPI_PerphCtrl(SPI_RegDef_t*pSPIx,uint8_t EnOrDis);

/* SSI configuration */
void SPI_SSIConfig(SPI_RegDef_t*pSPIx,uint8_t EnOrDis);

void SPI_SSOEConfig(SPI_RegDef_t*pSPIx,uint8_t EnOrDis);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTx(SPI_Handle_t *pSPIHandle);
void SPI_CloseRx(SPI_Handle_t *pSPIHandle);

/*Appliction callbacks */
void SPI_AppEvntCallbck(SPI_Handle_t *pSPIHandle,uint8_t AppEv);





#endif /* INC_STM32F072RB_SPI_DRIVER_H_ */
