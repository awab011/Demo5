/**
 * @file    uart.c
 * @brief   UART driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */



//** Includes **//
#include "uart.h"


//** Global variables **//
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart9;


H7_UARTHandler_s h7uart4;
H7_UARTHandler_s h7uart5;
H7_UARTHandler_s h7uart7;
H7_UARTHandler_s h7uart9;


//------------------- Function Defination -------------------//



H7_state_e H7_UARTx_init_struct(H7_UARTHandler_s *h7uart, UART_HandleTypeDef* huart){
	if(h7uart == NULL || huart == NULL){
		return H7_UART_NULL_PTR;
	}

	h7uart->huart = huart;
	h7uart->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}

H7_state_e H7_UARTx_init(H7_UARTHandler_s *h7uart, uint32_t BaudRate){
	if(h7uart->huart == &huart4){
		h7uart->huart->Instance = UART4;
	}else if(h7uart->huart == &huart5){
		h7uart->huart->Instance = UART5;
	}else if(h7uart->huart == &huart7){
		h7uart->huart->Instance = UART7;
	}else if(h7uart->huart == &huart9){
		h7uart->huart->Instance = UART9;
	}else{
		return H7_UART_PORT_INV;
	}


	h7uart->huart->Init.BaudRate 				= BaudRate;
	h7uart->huart->Init.WordLength 				= UART_WORDLENGTH_8B;
	h7uart->huart->Init.StopBits 				= UART_STOPBITS_1;
	h7uart->huart->Init.Parity 					= UART_PARITY_NONE;
	h7uart->huart->Init.Mode 					= UART_MODE_TX_RX;
	h7uart->huart->Init.HwFlowCtl 				= UART_HWCONTROL_NONE;
	h7uart->huart->Init.OverSampling 			= UART_OVERSAMPLING_16;
	h7uart->huart->Init.OneBitSampling 			= UART_ONE_BIT_SAMPLE_DISABLE;
	h7uart->huart->Init.ClockPrescaler 			= UART_PRESCALER_DIV1;
	h7uart->huart->AdvancedInit.AdvFeatureInit 	= UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_SetTxFifoThreshold(h7uart->huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(h7uart->huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_DisableFifoMode(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	return H7_PERIPH_OK;
}

H7_state_e H7_UARTx_DMA_RX_Init(H7_UARTHandler_s *h7uart, u32 DMA_mode, u32 BaudRate){

	if(h7uart->huart == &huart4){
		h7uart->huart->Instance 			= UART4;
		h7uart->uart_dma_rx.Instance 		= DMA1_Stream0;
		h7uart->uart_dma_rx.Init.Request	= DMA_REQUEST_UART4_RX;
		HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	}else if(h7uart->huart == &huart5){
		h7uart->huart->Instance 			= UART5;
		h7uart->uart_dma_rx.Instance 		= DMA1_Stream2;
		h7uart->uart_dma_rx.Init.Request	= DMA_REQUEST_UART5_RX;
		HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	}else if(h7uart->huart == &huart7){
		h7uart->huart->Instance 			= UART7;
		h7uart->uart_dma_rx.Instance 		= DMA1_Stream4;
		h7uart->uart_dma_rx.Init.Request	= DMA_REQUEST_UART7_RX;
		HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	}else if(h7uart->huart == &huart9){
		h7uart->huart->Instance 			= UART9;
		h7uart->uart_dma_rx.Instance 		= DMA1_Stream6;
		h7uart->uart_dma_rx.Init.Request	= DMA_REQUEST_UART9_RX;
		HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	}else{
		return H7_UART_PORT_INV;
	}


	h7uart->huart->Init.BaudRate = BaudRate;
	h7uart->huart->Init.WordLength = UART_WORDLENGTH_8B;
	h7uart->huart->Init.StopBits = UART_STOPBITS_1;
	h7uart->huart->Init.Parity = UART_PARITY_NONE;
	h7uart->huart->Init.Mode = UART_MODE_TX_RX;
	h7uart->huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	h7uart->huart->Init.OverSampling = UART_OVERSAMPLING_16;
	h7uart->huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	h7uart->huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
	h7uart->huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}

	h7uart->uart_dma_rx.Init.Direction				= DMA_PERIPH_TO_MEMORY;
	h7uart->uart_dma_rx.Init.PeriphInc 			= DMA_PINC_DISABLE;
	h7uart->uart_dma_rx.Init.MemInc 				= DMA_MINC_ENABLE;
	h7uart->uart_dma_rx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	h7uart->uart_dma_rx.Init.MemDataAlignment		= DMA_MDATAALIGN_BYTE;
	h7uart->uart_dma_rx.Init.Mode 					= DMA_mode;
	h7uart->uart_dma_rx.Init.Priority 				= DMA_PRIORITY_LOW;
	h7uart->uart_dma_rx.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	h7uart->uart_dma_rx.Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_1QUARTERFULL;
	h7uart->uart_dma_rx.Init.MemBurst 				= DMA_MBURST_SINGLE;
	h7uart->uart_dma_rx.Init.PeriphBurst 			= DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&h7uart->uart_dma_rx) != HAL_OK)
    {
    	return H7_UART_DMA_INIT_ERR;
//      Error_Handler();
    }

    __HAL_LINKDMA(h7uart->huart, hdmarx, h7uart->uart_dma_rx);

	if (HAL_UARTEx_SetTxFifoThreshold(h7uart->huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(h7uart->huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_DisableFifoMode(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	return H7_PERIPH_OK;
}

H7_state_e H7_UARTx_DMA_TX_Init(H7_UARTHandler_s *h7uart, u32 DMA_mode, u32 BaudRate){

	__HAL_RCC_DMA1_CLK_ENABLE();

	if(h7uart->huart == &huart4){
		h7uart->huart->Instance 			= UART4;
		h7uart->uart_dma_tx.Instance 		= DMA1_Stream1;
		h7uart->uart_dma_tx.Init.Request	= DMA_REQUEST_UART4_TX;
		HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	}else if(h7uart->huart == &huart5){
		h7uart->huart->Instance 			= UART5;
		h7uart->uart_dma_tx.Instance 		= DMA1_Stream3;
		h7uart->uart_dma_tx.Init.Request	= DMA_REQUEST_UART5_TX;
		HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	}else if(h7uart->huart == &huart7){
		h7uart->huart->Instance 			= UART7;
		h7uart->uart_dma_tx.Instance 		= DMA1_Stream5;
		h7uart->uart_dma_tx.Init.Request	= DMA_REQUEST_UART7_TX;
		HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	}else if(h7uart->huart == &huart9){
		h7uart->huart->Instance 			= UART9;
		h7uart->uart_dma_tx.Instance 		= DMA1_Stream7;
		h7uart->uart_dma_tx.Init.Request	= DMA_REQUEST_UART9_TX;
		HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	}else{
		return H7_UART_PORT_INV;
	}


	h7uart->huart->Init.BaudRate 				= BaudRate;
	h7uart->huart->Init.WordLength 				= UART_WORDLENGTH_8B;
	h7uart->huart->Init.StopBits 				= UART_STOPBITS_1;
	h7uart->huart->Init.Parity 					= UART_PARITY_NONE;
	h7uart->huart->Init.Mode 					= UART_MODE_TX_RX;
	h7uart->huart->Init.HwFlowCtl 				= UART_HWCONTROL_NONE;
	h7uart->huart->Init.OverSampling 			= UART_OVERSAMPLING_16;
	h7uart->huart->Init.OneBitSampling 			= UART_ONE_BIT_SAMPLE_DISABLE;
	h7uart->huart->Init.ClockPrescaler 			= UART_PRESCALER_DIV1;
	h7uart->huart->AdvancedInit.AdvFeatureInit 	= UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}

	h7uart->uart_dma_tx.Init.Direction				= DMA_PERIPH_TO_MEMORY;
	h7uart->uart_dma_tx.Init.PeriphInc 				= DMA_PINC_DISABLE;
	h7uart->uart_dma_tx.Init.MemInc 				= DMA_MINC_ENABLE;
	h7uart->uart_dma_tx.Init.PeriphDataAlignment 	= DMA_PDATAALIGN_BYTE;
	h7uart->uart_dma_tx.Init.MemDataAlignment		= DMA_MDATAALIGN_BYTE;
	h7uart->uart_dma_tx.Init.Mode 					= DMA_mode;
	h7uart->uart_dma_tx.Init.Priority 				= DMA_PRIORITY_LOW;
	h7uart->uart_dma_tx.Init.FIFOMode 				= DMA_FIFOMODE_ENABLE;
	h7uart->uart_dma_tx.Init.FIFOThreshold			= DMA_FIFO_THRESHOLD_1QUARTERFULL;
	h7uart->uart_dma_tx.Init.MemBurst 				= DMA_MBURST_SINGLE;
	h7uart->uart_dma_tx.Init.PeriphBurst 			= DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&h7uart->uart_dma_tx) != HAL_OK)
    {
    	return H7_UART_DMA_INIT_ERR;
//      Error_Handler();
    }

    __HAL_LINKDMA(h7uart->huart, hdmatx, h7uart->uart_dma_tx);

	if (HAL_UARTEx_SetTxFifoThreshold(h7uart->huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_SetRxFifoThreshold(h7uart->huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	if (HAL_UARTEx_DisableFifoMode(h7uart->huart) != HAL_OK)
	{
		return H7_UART_INIT_ERR;
	}
	return H7_PERIPH_OK;
}


















