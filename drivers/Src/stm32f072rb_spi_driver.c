/*
 * stm32f072rb_spi_driver.c
 *
 *  Created on: Jun. 7, 2020
 *      Author: Nidhin Easo Thomas
 */

#include<stm32f072rb_spi_driver.h>
static void spi_txe_intr_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_intr_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_intr_handle(SPI_Handle_t *pSPIHandle);


/* SPI Clock setup*/
/********************************************************************
 * @function			-	SPI_PeriphClkCtrl
 *
 * @brief				-	Enable or Disable the SPI Peripheral Clock
 *
 * @param[in]			-	Base address of SPI peripheral
 * @param[in]			-	ENABLE or DISABLE
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/


void SPI_PeriphClkCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
		{
			if(pSPIx==SPI1) SPI1_PCLK_EN();
			else if (pSPIx==SPI2) SPI2_PCLK_EN();

		}
		else
		{
			if(pSPIx==SPI1) SPI1_PCLK_DI();
			else if (pSPIx==SPI2) SPI2_PCLK_DI();

		}

}



/* SPI Initialize/Uninitialize */
/********************************************************************
 * @function			-	SPI_Init
 *
 * @brief				-	Initialize the SPI Peripheral
 *
 * @param[in]			-	SPI Handle
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{


	uint32_t tempCR1=0;										/* To store the bit fields for CR1 register */
	uint32_t tempCR2=0;										/* To store the bit fields for CR2 register */

	SPI_PeriphClkCtrl(pSPIHandle->pSPIx, ENABLE);
	/* SPI Mode setup(Master/Slave configuration) */
	tempCR1|=(pSPIHandle->SPI_PinCOnfig.SPI_DeviceMode<<SPI_CR1_MSTR);

	/* SPI Bus Configuration */
	if(pSPIHandle->SPI_PinCOnfig.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		/* BIDIMODE bit(Bit 15) must be cleared */
		tempCR1&=~(1<<SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_PinCOnfig.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		/* BIDIMODE bit(Bit 15) must be set */
		tempCR1|=(1<<SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPI_PinCOnfig.SPI_BusConfig==SPI_BUS_CONFIG_S_RXONLY)
	{
		/* BIDIMODE bit(Bit 15) must be cleared */
		tempCR1&=~(1<<SPI_CR1_BIDIMODE);
		/* RXONLY bit(Bit 10) must be set */
		tempCR1|=(1<<SPI_CR1_RXONLY);

	}
	/* Setup Data Frame Format */

	tempCR2|=(pSPIHandle->SPI_PinCOnfig.SPI_DFF<<SPI_CR2_DFF);
	pSPIHandle->pSPIx->CR2|=tempCR2;

	/* Setup clock phase  */
	tempCR1|=(pSPIHandle->SPI_PinCOnfig.SPI_CPHA<<SPI_CR1_CPHA);

	/* Setup clock polarity  */
	tempCR1|=(pSPIHandle->SPI_PinCOnfig.SPI_CPOL<<SPI_CR1_CPOL);

	/* Setup clock speed  */
	tempCR1|=(pSPIHandle->SPI_PinCOnfig.SPI_Speed<<SPI_CR1_BR);;



	/* Software Slave Management configuration */
	tempCR1|=(pSPIHandle->SPI_PinCOnfig.SPI_SSM<<SPI_CR1_SSM);
	pSPIHandle->pSPIx->CR1|=tempCR1;



}


/********************************************************************
 * @function			-	SPI_DeInit
 *
 * @brief				-	De-Initialize the SPI Peripheral
 *
 * @param[in]			-	SPI Base address Macro
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1) SPI1_REG_RESET();
	else if(pSPIx==SPI2)SPI2_REG_RESET();

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR& FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


void SPI_PerphCtrl(SPI_RegDef_t*pSPIx,uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
	{
		pSPIx->CR1|=(1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1&=~(1<<SPI_CR1_SPE);
	}
}

void SPI_SSIConfig(SPI_RegDef_t*pSPIx,uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
	{
		pSPIx->CR1|=(1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1&=~(1<<SPI_CR1_SSI);
	}
}


void SPI_SSOEConfig(SPI_RegDef_t*pSPIx,uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
		{
			pSPIx->CR2|=(1<<SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->CR2&=~(1<<SPI_CR2_SSOE);
		}

}





/* SPI Data transmission */
/********************************************************************
 * @function			-	SPI_SendData
 *
 * @brief				-	Send Data using SPI
 *
 * @param[in]			-	SPI Base address macro
 * @param[in]			-	Pointer to transmission data buffer
 * @param[in]			-	Length of transmitting data
 *
 * @return				-	None
 *
 * @Note				-	This is a blocking call(Polling type API)
 ********************************************************************/

void SPI_SendData(SPI_RegDef_t* pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t dffData=0;
	while(Len>0)
	{
		/* Wait until the transmit buffer is empty */
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET);
		dffData=(pSPIx->CR2>>(uint8_t)8);
		if(dffData==0xF)
		{
			pSPIx->DR=*((uint16_t *)pTxBuffer);
			/* 2 bytes of data is transmitted */
			Len--;
			Len--;
			/* Increment the Tx Buffer pointer */
			(uint16_t *)pTxBuffer++ ;
		}
		else if(dffData==0x7)
		{
			pSPIx->DR=*(pTxBuffer);
			/*1 byte of data is transmitted */
			Len--;
			pTxBuffer++;

		}

	}

}




/********************************************************************
 * @function			-	SPI_ReceiveData
 *
 * @brief				-	Receive Data using SPI
 *
 * @param[in]			-	SPI Base address macro
 * @param[in]			-	Pointer to receive data buffer
 * @param[in]			-	Length of transmitting data
 *
 * @return				-	None
 *
 * @Note				-	None
 ********************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t dffData=0;
		while(Len>0)
		{
			/* Wait until the RCXNE buffer is set */
			while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)==FLAG_RESET);
			dffData=(pSPIx->CR2>>(uint8_t)8);
			if(dffData==0xF)
			{
				/* 2 bytes of data is loaded from DR to RX Buffer */
				*((uint16_t *)pRxBuffer)=pSPIx->DR;

				Len--;
				Len--;
				/* Increment the Rx Buffer pointer */
				(uint16_t *)pRxBuffer++ ;
			}
			else if(dffData==0x7)
			{
				*(pRxBuffer)=pSPIx->DR;
				/*1 byte of data is loaded from DR to RX Buffer */
				Len--;
				pRxBuffer++;

			}

		}


}



/* SPI IRQ handling*/
/***********************************************************************
 * @function			-	SPI_IRQInterruptConfig
 *
 * @brief				-	Configure Interrupt on SPI Peripheral
 *
 * @param[in]			-	IRQ Number
 * @param[in]			-	ENABLE or DISABLE
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/

void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDis)
{
	if(EnOrDis==ENABLE)
		*NVIC_ISER|=(1<<IRQNum);
	else
		*NVIC_ICER|=(1<<IRQNum);

}

/***********************************************************************
 * @function			-	SPI_IRQPriorityConfig
 *
 * @brief				-	Configure Interrupt Priority
 *
 * @param[in]			-	IRQ Number
 * @param[in]			-	Interrupt Priority
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNum,uint32_t IRQPriority)
{
	uint32_t iprx=IRQNum/4;
	uint32_t ipr_sec=IRQNum%4;
	*(NVIC_IPR_BASEADDR+iprx)|=(IRQPriority<<((ipr_sec*8)+(8-NVIC_IPR_BITS_ACCESS))); /* For Cortex M0 processor, only Bits 6 and 7 are implemented */

}




/* Non Blocking APIs - Send and Receive Data */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
		uint8_t state=pSPIHandle->TxState;
		if(state!=SPI_BSY_TX)
		{
		pSPIHandle->pTxBuffer=pTxBuffer;
		pSPIHandle->TxLen=Len;
		pSPIHandle->TxState=SPI_BSY_TX;
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t SPI_ReceiveDatIT(SPI_Handle_t* pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state=pSPIHandle->RxState;
	if(state!=SPI_BSY_RX)
	{
		pSPIHandle->pRxBuffer=pRxBuffer;
		pSPIHandle->RxLen=Len;
		pSPIHandle->RxState=SPI_BSY_TX;
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_RXNEIE);
	}
	return state;


}

/***********************************************************************
 * @function			-	SPI_IRQHandling
 *
 * @brief				-	Handle Interrupt on the specified pin of GPIO port
 *
 * @param[in]			-	GPIO Port Pin Number
 *
 * @return				-	None
 *
 * @Note				-	None
 ***********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t flagChk,IntrBitChk;
	/* Check for TXE */
	flagChk=pSPIHandle->pSPIx->SR&(1<<SPI_SR_TXE);
	IntrBitChk=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);
	if(flagChk&& IntrBitChk)
	{
		/* Handle TXE */
		spi_txe_intr_handle(pSPIHandle);
	}
	/* Check for RXNE */
	flagChk=pSPIHandle->pSPIx->SR&(1<<SPI_SR_RXNE);
	IntrBitChk=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);
	if(flagChk&& IntrBitChk)
	{
		/* Handle RXNE */
		spi_rxne_intr_handle(pSPIHandle);
	}
	/* Check for OVR ERROR  */
	flagChk=pSPIHandle->pSPIx->SR&(1<<SPI_SR_OVR);
	IntrBitChk=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);
	if(flagChk&& IntrBitChk)
	{
		/* Handle RXNE */
		spi_ovr_intr_handle(pSPIHandle);
	}
}


/* Helper function implementation */
static void spi_txe_intr_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t dffData;
	dffData=(pSPIHandle->pSPIx->CR2>>(uint8_t)8);
	if(dffData==0xF)
	{
		pSPIHandle->pSPIx->DR=*((uint16_t *)pSPIHandle->pTxBuffer);
		/* 2 bytes of data is transmitted */
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		/* Increment the Tx Buffer pointer */
		(uint16_t *)pSPIHandle->pTxBuffer++ ;
	}
	else if(dffData==0x7)
	{
		pSPIHandle->pSPIx->DR=*(pSPIHandle->pTxBuffer);
		/*1 byte of data is transmitted */
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++ ;
	}
	if(!pSPIHandle->TxLen)
	{
		/* Close SPI Transmission */

		SPI_CloseTx(pSPIHandle);
		SPI_AppEvntCallbck(pSPIHandle,SPI_EVNT_TX_CMPLT);

	}

}
static void spi_rxne_intr_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t dffData;
	dffData=(pSPIHandle->pSPIx->CR2>>(uint8_t)8);
	if(dffData==0xF)
	{
		*((uint16_t *)pSPIHandle->pRxBuffer)=pSPIHandle->pSPIx->DR;
		/* 2 bytes of data is transmitted */
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		/* Increment the Tx Buffer pointer */
		(uint16_t *)pSPIHandle->pRxBuffer++ ;
	}
	else if(dffData==0x7)
	{
		*(pSPIHandle->pTxBuffer)=pSPIHandle->pSPIx->DR;
		/*1 byte of data is transmitted */
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++ ;
	}
	if(!pSPIHandle->RxLen)
	{
		/* Close SPI Transmission */

		SPI_CloseRx(pSPIHandle);
		SPI_AppEvntCallbck(pSPIHandle,SPI_EVNT_RX_CMPLT);

	}
}
static void spi_ovr_intr_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t dummyRead;
	/* Clear OVR Flag */
	if(pSPIHandle->TxState!=SPI_BSY_TX)
	{
		dummyRead=pSPIHandle->pSPIx->DR;
		dummyRead=pSPIHandle->pSPIx->SR;
	}
	(void)dummyRead;
	SPI_AppEvntCallbck(pSPIHandle,SPI_EVNT_OVR_ERR);

}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t dummyRead;
	dummyRead=pSPIx->DR;
	dummyRead=pSPIx->SR;
	(void)dummyRead;

}
void SPI_CloseTx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_TXEIE);	/* Prevents further transmission interrupts */
	pSPIHandle->pTxBuffer=NULL;
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState=SPI_RDY;

}
void SPI_CloseRx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_RXNEIE);	/* Prevents further Rx interrupts */
	pSPIHandle->pRxBuffer=NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState=SPI_RDY;

}

__weak void SPI_AppEvntCallbck(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}



