/**
 * @file    spi.c
 * @brief   SPI driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */

//** Includes **//
#include "spi.h"

//** Global variables **//
SPI_HandleTypeDef hspi2;

//------------------- Function Defination -------------------//

SPI_HandleTypeDef spi2;
H7_SPIHandler_s h7spi2;


/**
 * @brief 	Initialize the structure of SPI for H7 library system
 * @param	spi This is a pointer the library structre and considered as the handler for library use,
 * 				it is already declared in the library
 * @param	hspi This is the pointer to the specific spi HAL handler, already declared in the library
 * @note	H7.1.0 board version only contains one SPI port which is SPI2
 * @retval	H7 State
 */
H7_state_e H7_struct_init(H7_SPIHandler_s *spi, SPI_HandleTypeDef *hspi){
	if(spi == NULL){
		spi->status = H7_STATE_NULL;
		return H7_STATE_NULL;
	}
	spi->hspi = hspi;
//	spi->GPIOx_NSS = GPIOx_NSS;
//	spi->GPIO_Pin_NSS = GPIO_Pin_NSS;
	memset(spi->rxData, 0, SPI_RX_BUF_SIZE);
	memset(spi->txData, 0, SPI_TX_BUF_SIZE);

	spi->status = H7_OK;
	return H7_OK;
}

/**
 * @brief 	Initialize the SPI peripheral
 * @param	spi This is a pointer the library structre and considered as the handler for library use,
 * 				it is already declared in the library
 * @param	spiMode 	To indicate whether to be a Master or a slave, can find Macro in spi.h\
 * @param	spiSpeed 	To indicate spped of the communication, argument should be
 * 						a member of @H7_SPI_speed_e
 * @param	dataSize	To indicate the size of the data in the communication,
 * 						argument should be a member of @H7_SPI_dataSize_e
 *
 * @note	H7_struct_init function  must be called before calling this function
 * @retval	H7 State
 */
H7_state_e H7_SPIx_Init(H7_SPIHandler_s *spi, u32 spiMode, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize)
{
//	if(spiMode == SPI_MODE_MASTER){
//		H7_gpio_PinInit(spi->GPIOx_NSS, spi->GPIO_Pin_NSS, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL);
//	}else if(spiMode == SPI_MODE_SLAVE){
//		H7_gpio_PinInit(spi->GPIOx_NSS, spi->GPIO_Pin_NSS, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_NOPULL);
//	}else{
//		spi->status = H7_INV_MODE;
//		return H7_INV_MODE;
//	}

	if(spi == NULL){
		spi->status = H7_STATE_NULL;
		return H7_STATE_NULL;
	}

	if(spi->hspi == &spi2){
		spi->hspi->Instance = SPI2;
	}else{
		spi->status = H7_INV_PORT;
		return H7_INV_PORT;
	}

	spi->hspi->Init.Mode 						= spiMode;
	spi->hspi->Init.DataSize 					= dataSize;
	spi->hspi->Init.BaudRatePrescaler 			= spiSpeed;
	spi->hspi->Init.Direction 					= SPI_DIRECTION_2LINES;	// Full Duplex
	spi->hspi->Init.CLKPolarity 				= SPI_POLARITY_LOW;
	spi->hspi->Init.CLKPhase 					= SPI_PHASE_1EDGE;
	spi->hspi->Init.NSS 						= SPI_NSS_SOFT; 				// NSS selection is controlled by user
	spi->hspi->Init.FirstBit 					= SPI_FIRSTBIT_MSB;
	spi->hspi->Init.TIMode 						= SPI_TIMODE_DISABLE;
	spi->hspi->Init.CRCCalculation 				= SPI_CRCCALCULATION_DISABLE;
	spi->hspi->Init.CRCPolynomial 				= 0x0;
//	spi->hspi->Init.NSSPMode 					= SPI_NSS_PULSE_DISABLE;
//	spi->hspi->Init.NSSPolarity 				= SPI_NSS_POLARITY_LOW;
	spi->hspi->Init.FifoThreshold 				= SPI_FIFO_THRESHOLD_01DATA;
	spi->hspi->Init.TxCRCInitializationPattern 	= SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	spi->hspi->Init.RxCRCInitializationPattern 	= SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	spi->hspi->Init.MasterSSIdleness 			= SPI_MASTER_SS_IDLENESS_00CYCLE;
	spi->hspi->Init.MasterInterDataIdleness 	= SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	spi->hspi->Init.MasterReceiverAutoSusp 		= SPI_MASTER_RX_AUTOSUSP_DISABLE;
	spi->hspi->Init.MasterKeepIOState 			= SPI_MASTER_KEEP_IO_STATE_DISABLE;
	spi->hspi->Init.IOSwap 						= SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(spi->hspi) != HAL_OK)
	{
		Error_Handler();
	}
	return H7_OK;

}

/**
 * @brief 	Initialize the SPI peripheral with DMA
 * @param	spi This is a pointer the library structre and considered as the handler for library use,
 * 				it is already declared in the library
 * @param	spiMode 	To indicate whether to be a Master or a slave, can find Macro in spi.h
 * @param	DMA_mode	To indicate how the DMA stores the data, for continuous receiving @DMA_CIRCULAR
 * @param	priority	To indicate the priority of the stream inside DMA controller,
 * 						macros can be found in spi.h
 * @param	spiSpeed 	To indicate spped of the communication, argument should be
 * 						a member of @H7_SPI_speed_e
 * @param	dataSize	To indicate the size of the data in the communication,
 * 						argument should be a member of @H7_SPI_dataSize_e
 *
 * @note	H7_struct_init function must be called before calling this function
 * @retval	H7 State
 */
H7_state_e H7_SPIx_rx_DMA_init(H7_SPIHandler_s *spi, u32 spiMode, u32 DMA_mode, u32 priority, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize){
	H7_state_e rslt;

	rslt = H7_SPIx_Init(spi, spiMode, spiSpeed, dataSize);

    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, H7_SPI2_RX_DMA_PRIO, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	if(rslt == H7_OK){
	    spi->hdma_spi_rx.Instance 					= DMA2_Stream4;
	    spi->hdma_spi_rx.Init.Request 				= DMA_REQUEST_SPI2_RX;
	    spi->hdma_spi_rx.Init.Direction 			= DMA_PERIPH_TO_MEMORY;
	    spi->hdma_spi_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	    spi->hdma_spi_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	    spi->hdma_spi_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	    spi->hdma_spi_rx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	    spi->hdma_spi_rx.Init.Mode 					= DMA_mode;
	    spi->hdma_spi_rx.Init.Priority 				= priority;				// Stream Priority
	    spi->hdma_spi_rx.Init.FIFOMode 				= DMA_FIFOMODE_DISABLE;
	    if (HAL_DMA_Init(&spi->hdma_spi_rx) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    __HAL_LINKDMA(spi->hspi,hdmarx, spi->hdma_spi_rx);

	    return rslt;
	}else{
		return rslt;
	}

}

/**
 * @brief 	Initialize the SPI peripheral with DMA
 * @param	spi This is a pointer the library structre and considered as the handler for library use,
 * 				it is already declared in the library
 * @param	spiMode 	To indicate whether to be a Master or a slave, can find Macro in spi.h
 * @param	DMA_mode	To indicate how the DMA stores the data, for continuous receiving @DMA_NORMAL
 * @param	priority	To indicate the priority of the stream inside DMA controller,
 * 						macros can be found in spi.h
 * @param	spiSpeed 	To indicate spped of the communication, argument should be
 * 						a member of @H7_SPI_speed_e
 * @param	dataSize	To indicate the size of the data in the communication,
 * 						argument should be a member of @H7_SPI_dataSize_e
 *
 * @note	H7_struct_init function must be called before calling this function
 * @retval	H7 State
 */
H7_state_e H7_SPIx_tx_DMA_init(H7_SPIHandler_s *spi, u32 spiMode, u32 DMA_mode, u32 priority, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize){
	H7_state_e rslt;

	rslt = H7_SPIx_Init(spi, spiMode, spiSpeed, dataSize);

	if(rslt == H7_OK){
	    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, H7_SPI2_TX_DMA_PRIO, 0);
	    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

	    spi->hdma_spi_tx.Instance 					= DMA2_Stream5;
	    spi->hdma_spi_tx.Init.Request 				= DMA_REQUEST_SPI2_TX;
	    spi->hdma_spi_tx.Init.Direction 			= DMA_MEMORY_TO_PERIPH;
	    spi->hdma_spi_tx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	    spi->hdma_spi_tx.Init.MemInc 				= DMA_MINC_ENABLE;
	    spi->hdma_spi_tx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	    spi->hdma_spi_tx.Init.MemDataAlignment 		= DMA_MDATAALIGN_BYTE;
	    spi->hdma_spi_tx.Init.Mode 					= DMA_mode;
	    spi->hdma_spi_tx.Init.Priority 				= priority;					// Stream Priority
	    spi->hdma_spi_tx.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	    spi->hdma_spi_tx.Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_FULL;
	    spi->hdma_spi_tx.Init.MemBurst				= DMA_MBURST_INC4;
	    spi->hdma_spi_tx.Init.PeriphBurst			= DMA_MBURST_SINGLE;
	    if (HAL_DMA_Init(&spi->hdma_spi_tx) != HAL_OK)
	    {
	      Error_Handler();
	    }
	    __HAL_LINKDMA(spi->hspi,hdmatx, spi->hdma_spi_tx);
	    return H7_OK;
	}else{
		return rslt;
	}


}

