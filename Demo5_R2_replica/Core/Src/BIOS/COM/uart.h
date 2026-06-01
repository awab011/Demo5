/**
 * @file    uart.c
 * @brief   UART header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_


//** Includes **//
#include "../../Platform/H7_system.h"


//------------------- Enumeration -------------------//


//------------------- Structures -------------------//
typedef struct{
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef uart_dma_rx;
	DMA_HandleTypeDef uart_dma_tx;
	H7_state_e status;
} H7_UARTHandler_s;


//------------------- Macros and Global Variables -------------------//


//** Macros **//


//** Global variables **//
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart9;

extern H7_UARTHandler_s h7uart4;
extern H7_UARTHandler_s h7uart5;
extern H7_UARTHandler_s h7uart7;
extern H7_UARTHandler_s h7uart9;


//------------------- Function Declaration -------------------//

H7_state_e H7_UARTx_init_struct(H7_UARTHandler_s *h7uart, UART_HandleTypeDef *huart);
H7_state_e H7_UARTx_init(H7_UARTHandler_s *h7uart, uint32_t BaudRate);
H7_state_e H7_UARTx_DMA_RX_Init(H7_UARTHandler_s *h7uart, u32 DMA_mode, u32 BaudRate);
H7_state_e H7_UARTx_DMA_TX_Init(H7_UARTHandler_s *h7uart, u32 DMA_mode, u32 BaudRate);
H7_state_e H7_UART_Transmit();
H7_state_e H7_UART_Receive();
H7_state_e H7_UART_Receive_IT();
H7_state_e H7_UART_Transmit_IT();
H7_state_e H7_UART_Receive_DMA();
H7_state_e H7_UART_Transmit_DMA();


#endif /* SRC_UART_H_ */
