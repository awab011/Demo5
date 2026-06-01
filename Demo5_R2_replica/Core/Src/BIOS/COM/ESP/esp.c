/*
 * esp.c
 *
 *  Created on: Dec 14, 2025
 *      Author: moon
 *      modified by Mohammed to comply with H7Lib1.0
 *      Changed:
 *      - Structure handler for uart, and how to deal with it
 */

#include "esp.h"

void ESP_Init(ESP_Handler_t *esp, H7_UARTHandler_s *h7uart, ESP_UART_Enable_t en)
{
	esp->huartx = h7uart->huart;

	uint8_t rx = 0;
	if (en == TX_N_RX) { en = TX_ONLY; rx = 1; }

	if (h7uart->huart->gState != HAL_UART_STATE_READY)
	{
		H7_UARTx_init(h7uart, 115200); // this line no use go check again
	}

	switch (en)
	{
		case TX_ONLY:
//			UARTx_DMA_Tx_Init(huartx, &hdma_usart2_tx, 115200);
			memset(esp->tx_msg.data_arr, 0, TX_MSG_SIZE);
			esp->tx_msg.header1 = TX_HEADER1_BYTE;
			esp->tx_msg.header2 = TX_HEADER2_BYTE;
			esp->tx_msg.endbyte = TX_END_BYTE;
			esp->rx_state = RX_DISABLE;
			if (rx == 0) break;

		case RX_ONLY:
			H7_UARTx_DMA_RX_Init(h7uart, DMA_CIRCULAR, 115200);
			esp->rx_idx = 0;
			esp->rx_state = PENDING_HEADER1;
//			HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
			HAL_UART_Receive_DMA(esp->huartx, esp->rx_buf, RX_MSG_SIZE);
			__HAL_UART_ENABLE_IT(esp->huartx, UART_IT_IDLE);
			break;

		default:
			break;
	}
}

void ESP_Tx_Handler(ESP_Handler_t *esp)
{
//	uint8_t tx_buf[TX_MSG_SIZE];
//	uint8_t cs = 0;
//
//	memcpy(tx_buf, esp->tx_msg.data_arr, TX_MSG_SIZE);
//
//	for (int i = 0; i < TX_MSG_SIZE - 1; i++)
//		cs ^= tx_buf[i];
//
//	tx_buf[TX_MSG_SIZE - 1] = cs;
//
//	HAL_UART_Transmit_IT(esp->huartx, tx_buf, TX_MSG_SIZE);

	HAL_UART_Transmit_IT(esp->huartx, esp->tx_msg.data_arr, TX_MSG_SIZE);

//	HAL_UART_Transmit_DMA(esp->huartx, esp->tx_msg.data_arr, TX_MSG_SIZE);
}

/*
void ESP_Rx_Handler(ESP_Handler_t *esp)
{
	switch (esp->rx_state)
	{
		case PENDING_HEADER1:
			if (esp->rx_buf[0] == RX_HEADER1_BYTE)
			{
				esp->rx_idx = 1;
				esp->rx_state = PENDING_HEADER2;
			}
			HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
//			HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[1], 1);
			break;

		case PENDING_HEADER2:
			if (esp->rx_buf[1] == RX_HEADER2_BYTE)
			{
				esp->rx_idx = 2;
				esp->rx_state = PAYLOAD1;
				HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], RX_CUTOFF_BYTE - 2);
//				HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[2], RX_CUTOFF_BYTE - 2);
			}
			else if (esp->rx_buf[1] == RX_HEADER1_BYTE)
			{
				HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
//				HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[1], 1);
			}
			else
			{
				esp->rx_idx = 0;
				esp->rx_state = PENDING_HEADER1;
				HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
//				HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[0], 1);
			}
			break;

		case PAYLOAD1:
			esp->rx_idx = RX_CUTOFF_BYTE;
			esp->rx_state = PAYLOAD2;
			HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], RX_MSG_SIZE - RX_CUTOFF_BYTE - 1);
//			HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[RX_CUTOFF_BYTE], RX_MSG_SIZE - RX_CUTOFF_BYTE - 1);
			break;

		case PAYLOAD2:
			esp->rx_idx = RX_MSG_SIZE - 1;
			esp->rx_state = PENDING_ENDBYTE;
			HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
//			HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[RX_MSG_SIZE - 1], 1);
			break;

		case PENDING_ENDBYTE:
			uint8_t cs = 0;

			for (int i = 0; i < RX_MSG_SIZE - 1; i++)
				cs ^= esp->rx_buf[i];

			if (esp->rx_buf[RX_MSG_SIZE - 1] == cs)
			{
				memcpy(esp->rx_msg.data_arr, esp->rx_buf, RX_MSG_SIZE);
				memcpy(ros2_msg.data_arr, esp->rx_buf, RX_MSG_SIZE);
			}

//			if (esp->rx_buf[RX_MSG_SIZE - 1] == RX_END_BYTE)
//				memcpy(esp->rx_msg.data_arr, esp->rx_buf, RX_MSG_SIZE);

			memset(esp->rx_buf, 0, RX_MSG_SIZE);
			esp->rx_idx = 0;
			esp->rx_state = PENDING_HEADER1;
			HAL_UART_Receive_IT(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
//			HAL_UART_Receive_DMA(esp->huartx, &esp->rx_buf[esp->rx_idx], 1);
			break;

		default:
			break;
	}
}
*/

//void ESP_Rx_Handler(ESP_Handler_t *esp)
//{
//	if (esp->rx_dma_cplt)
//	{
//		esp->rx_dma_cplt = false;
//
//		uint8_t rx_temp[RX_MSG_SIZE];
//
//		memcpy(rx_temp, esp->rx_buf, RX_MSG_SIZE);
//
//		uint8_t cs = 0;
//		for (int i = 0; i < RX_MSG_SIZE - 1; i++)
//			cs ^= rx_temp[i];
//
//		if (cs == rx_temp[RX_MSG_SIZE - 1])
//		{
//			memcpy(esp->rx_msg.data_arr, rx_temp, RX_MSG_SIZE);
//		}
//
//		HAL_UART_Receive_DMA(esp->huartx, esp->rx_buf, RX_MSG_SIZE);
//	}
//}

void ESP_Rx_Handler(ESP_Handler_t *esp)
{
	if (esp->rx_dma_cplt)
	{
		esp->rx_dma_cplt = false;

		if (esp->rx_buf[0] == RX_HEADER1_BYTE && esp->rx_buf[1] == RX_HEADER2_BYTE)
		{
			uint8_t cs = 0;

			for (int i = 0; i < RX_MSG_SIZE - 1; i++)
			{
				cs ^= esp->rx_buf[i];
			}

			if (cs == esp->rx_buf[RX_MSG_SIZE - 1])
			{
				memcpy(esp->rx_msg.data_arr, esp->rx_buf, RX_MSG_SIZE);
				memcpy(ros2_msg.data_arr, esp->rx_buf, RX_MSG_SIZE);
			}
		}

		HAL_UART_DMAStop(esp->huartx);
		memset(esp->rx_buf, 0, RX_MSG_SIZE);
		HAL_UART_Receive_DMA(esp->huartx, esp->rx_buf, RX_MSG_SIZE);
	}
}
