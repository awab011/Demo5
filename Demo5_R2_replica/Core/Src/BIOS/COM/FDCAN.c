/**
 * @file    FDCAN.c
 * @brief   FDCAN driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note	You have to define the FDCAN you are using before initialization in FDCAN.h
 * @ingroup Communication
 */


//** Includes **//
#include "FDCAN.h"


//** Global variables **//
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

H7_FDCANHandler_s h7fdcan1;
H7_FDCANHandler_s h7fdcan2;
H7_FDCANHandler_s h7fdcan3;

//------------------- Function Defination -------------------//

H7_state_e H7_FDCANHandler_init(H7_FDCANHandler_s *fdcan, FDCAN_HandleTypeDef* hfdcan){
	if(fdcan == NULL){
		H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_NULL_PTR);
		return H7_FDCAN_NULL_PTR;
	}
	fdcan->hfdcan 		= hfdcan;
	fdcan->filter_count = 0;

	fdcan->status = H7_PERIPH_OK;
	return H7_PERIPH_OK;
}


H7_state_e H7_FDCAN_init(H7_FDCANHandler_s *fdcan, u32 STD_Filters_Bank, u32 EXD_Filters_Bank, u32 rxFIFO0Elmnts, u32 rxFIFO1Elmnts, u32 txFIFOElmnts, u32 txFIFO_operation_mode, u32 FIFOBuffer, H7_fdcan_baudrate_e data_rate, H7_fdcan_data_size size){



	if(fdcan->hfdcan == &hfdcan1){
		fdcan->hfdcan->Instance 				= FDCAN1;
#ifdef H7FDCAN1
		fdcan->hfdcan->Init.MessageRAMOffset 	= FDCAN1_MESSAGE_RAM_OFFSET;
#endif
	}

	else if(fdcan->hfdcan == &hfdcan2){
		fdcan->hfdcan->Instance 				= FDCAN2;
#ifdef H7FDCAN2
		fdcan->hfdcan->Init.MessageRAMOffset 	= FDCAN2_MESSAGE_RAM_OFFSET;
#endif
	}

	else if(fdcan->hfdcan == &hfdcan3){
		fdcan->hfdcan->Instance 				= FDCAN3;
#ifdef H7FDCAN3
		fdcan->hfdcan->Init.MessageRAMOffset 	= FDCAN3_MESSAGE_RAM_OFFSET;
#endif
	}

	else{
		fdcan->status = H7_FDCAN_INVALID_PORT;
		H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_INVALID_PORT);
		return H7_FDCAN_INVALID_PORT;
	}
	// Initialize FDCAN peripheral
	fdcan->hfdcan->Init.FrameFormat 		= FDCAN_FRAME_CLASSIC;
	fdcan->hfdcan->Init.Mode 				= FDCAN_MODE_NORMAL;
	fdcan->hfdcan->Init.AutoRetransmission	= ENABLE;
	fdcan->hfdcan->Init.TransmitPause 		= DISABLE;
	fdcan->hfdcan->Init.ProtocolException 	= DISABLE; // Not needded as we are not usung CAN FD frame

	// Set timing parameters based on desired data rate
	if(data_rate == FDCAN_500kbps){
		fdcan->hfdcan->Init.NominalPrescaler 		= 1;
		fdcan->hfdcan->Init.NominalSyncJumpWidth 	= 30;
		fdcan->hfdcan->Init.NominalTimeSeg1 		= 169;
		fdcan->hfdcan->Init.NominalTimeSeg2 		= 30;
		fdcan->hfdcan->Init.DataPrescaler 			= 8;
		fdcan->hfdcan->Init.DataSyncJumpWidth 		= 10;
		fdcan->hfdcan->Init.DataTimeSeg1 			= 14;
		fdcan->hfdcan->Init.DataTimeSeg2 			= 10;
	}else if(data_rate == FDCAN_1Mbps){
		fdcan->hfdcan->Init.NominalPrescaler 		= 5;
		fdcan->hfdcan->Init.NominalSyncJumpWidth 	= 16;
		fdcan->hfdcan->Init.NominalTimeSeg1 		= 16;
		fdcan->hfdcan->Init.NominalTimeSeg2 		= 3;
		fdcan->hfdcan->Init.DataPrescaler 			= 5;
		fdcan->hfdcan->Init.DataSyncJumpWidth 		= 9;
	  	fdcan->hfdcan->Init.DataTimeSeg1 			= 10;
	  	fdcan->hfdcan->Init.DataTimeSeg2 			= 9;
	}

	// Configure message RAM and buffers

	fdcan->hfdcan->Init.StdFiltersNbr 			= STD_Filters_Bank;
	fdcan->hfdcan->Init.ExtFiltersNbr 			= EXD_Filters_Bank;
	fdcan->hfdcan->Init.RxFifo0ElmtsNbr 		= rxFIFO0Elmnts;
	fdcan->hfdcan->Init.RxFifo0ElmtSize 		= convert_to_data_bytes(size);
	fdcan->hfdcan->Init.RxFifo1ElmtsNbr 		= rxFIFO1Elmnts;
	fdcan->hfdcan->Init.RxFifo1ElmtSize 		= convert_to_data_bytes(size);
	fdcan->hfdcan->Init.RxBuffersNbr 			= 0;
	fdcan->hfdcan->Init.RxBufferSize 			= convert_to_data_bytes(size);
	fdcan->hfdcan->Init.TxEventsNbr 			= 0;	// Not needed for normal operations
	fdcan->hfdcan->Init.TxBuffersNbr 			= 0;
	fdcan->hfdcan->Init.TxFifoQueueElmtsNbr 	= txFIFOElmnts;
	fdcan->hfdcan->Init.TxFifoQueueMode 		= txFIFO_operation_mode;
	fdcan->hfdcan->Init.TxElmtSize 				= convert_to_data_bytes(size);

	fdcan->memRAMSize = H7_FDCAN_calcMemSize(fdcan);

	if(fdcan->memRAMSize >= RAM_SIZE_ALLOW){
		fdcan->status = H7_FDCAN_MEM_RAM_OVERSIZE;
		H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_MEM_RAM_OVERSIZE);
		return H7_FDCAN_MEM_RAM_OVERSIZE;

	}
	if (HAL_FDCAN_Init(fdcan->hfdcan) != HAL_OK)
	{
		fdcan->status = H7_FDCAN_INIT_ERR;
		H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_INIT_ERR);
		return H7_FDCAN_INIT_ERR;
	}

    /* Start FDCAN and enable FIFO new message notification */
    if(HAL_FDCAN_Start(fdcan->hfdcan) != HAL_OK){
    	fdcan->status = H7_FDCAN_START_ERR;
    	H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_START_ERR);
    	return H7_FDCAN_START_ERR;
    }

    if(HAL_FDCAN_ActivateNotification(fdcan->hfdcan, FIFOBuffer, 0) != HAL_OK){
    	fdcan->status = H7_FDCAN_FIFO_ACTICATE_ERR;
    	H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_FIFO_ACTICATE_ERR);
    	return H7_FDCAN_FIFO_ACTICATE_ERR;
    }
    fdcan->status = H7_PERIPH_OK;
    return H7_PERIPH_OK;

}

H7_state_e H7_FDCAN_addFilterRange(H7_FDCANHandler_s *fdcan, uint32_t idType, uint32_t ID1, uint32_t ID2, uint32_t storeTo){
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType 		= idType;
	sFilterConfig.FilterIndex 	= fdcan->filter_count++;
	sFilterConfig.FilterType 	= FDCAN_FILTER_RANGE;
	sFilterConfig.FilterID1 	= ID1;
	sFilterConfig.FilterID2 	= ID2;
	sFilterConfig.FilterConfig 	= storeTo;
	sFilterConfig.RxBufferIndex	= 0;

    if(HAL_FDCAN_ConfigFilter(fdcan->hfdcan, &sFilterConfig) != HAL_OK){
    	fdcan->status = H7_FDCAN_FILTER_INIT_ERR;
    	H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_FILTER_INIT_ERR);
    	return H7_FDCAN_FILTER_INIT_ERR;
    }
    return H7_FDCAN_FILTER_OK;
}
H7_state_e FDCAN_TxMsg(FDCAN_HandleTypeDef *hfdcan, u32 ID, u8 *msg, u8 len){
	FDCAN_TxHeaderTypeDef txHeader;
//	u16 i = 0;
	u8 txMsg[8] = {0};

	if(hfdcan == NULL){
//    	hfdcan->status = H7_FDCAN_NULL_PTR;
    	H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_NULL_PTR);
		return H7_FDCAN_NULL_PTR;
	}
	u32 timeout = HAL_GetTick();

	while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0){
		if((HAL_GetTick() - timeout) >= 50){  // 50ms timeout
//	    	hfdcan->status = H7_FDCAN_NULL_PTR;
			return H7_FDCAN_TIMEOUT_ERR;
		}
	}
	txHeader.Identifier				= ID;
	txHeader.IdType					= FDCAN_STANDARD_ID;
	txHeader.TxFrameType			= FDCAN_DATA_FRAME;
	txHeader.DataLength				= convert_to_DLC(len);
	txHeader.ErrorStateIndicator	= FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch			= FDCAN_BRS_OFF;
	txHeader.FDFormat				= FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl		= FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker			= 0;

	for(u8 i = 0; i < len; i++){
		txMsg[i] = msg[i];
	}
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txMsg) != HAL_OK){
//    	hfdcan->status = H7_FDCAN_MSGSEND_ERR;
//    	H7_ERR_HANDLE(H7SYSTEM, H7_FDCAN_MSGSEND_ERR);
//		Error_Handler();
		return H7_FDCAN_MSGSEND_ERR;
	}
	return H7_PERIPH_OK;
}

H7_state_e FDCAN_TxMsgEID(FDCAN_HandleTypeDef* hfdcan,uint32_t EID,uint8_t *msg,uint8_t len){

	FDCAN_TxHeaderTypeDef txHeader;
	u8 txMsg[8] = {0};

	// Safety check
	if(hfdcan == NULL){
		return H7_FDCAN_NULL_PTR;
	}
	if(len > 8){
		len = 8;
	}
	// Wait for free TX mailbox (FIFO) with timeout to not block
//	u32 timeout = HAL_GetTick();
	if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0){
//		if((HAL_GetTick() - timeout) >= 100){  // 100ms timeout
//			return H7_FDCAN_TIMEOUT_ERR;
//		}
		return H7_FDCAN_TX_FIFO_FULL;
	}

    // Clear header structure
    memset(&txHeader, 0, sizeof(FDCAN_TxHeaderTypeDef));

    // Configure TX header for Extended ID
	txHeader.Identifier 			= EID & 0x1FFFFFFF;  // Ensure 29-bit ID
	txHeader.IdType 				= FDCAN_EXTENDED_ID;      // Extended identifier type
	txHeader.FDFormat				= FDCAN_CLASSIC_CAN;
	txHeader.TxFrameType			= FDCAN_DATA_FRAME;  // Data frame (not remote)
//	txHeader.DataLength = len << 16; // for classic CAN with 8 bytes size
	txHeader.DataLength				= convert_to_DLC(len);
	txHeader.ErrorStateIndicator	= FDCAN_ESI_PASSIVE;
	txHeader.BitRateSwitch			= FDCAN_BRS_OFF; // Is off for classic FDCAN
	txHeader.TxEventFifoControl		= FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker			= 0;

	// Sending the msg
	memcpy(txMsg, msg, len);
//	for(u8 i = 0; i < len; i++){
//		txMsg[i] = msg[i];
//	}
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txMsg) != HAL_OK){

//		Error_Handler();
		return H7_FDCAN_MSGSEND_ERR;
	}
	return H7_PERIPH_OK;
}


u32 convert_to_DLC(u8 len){
    switch(len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return FDCAN_DLC_BYTES_8;
    }
}

u32 convert_to_data_bytes(H7_fdcan_data_size size){
	switch(size){
	case H7_FDCAN_8_BYTES: return FDCAN_DATA_BYTES_8;
	case H7_FDCAN_12_BYTES: return FDCAN_DATA_BYTES_12;
	case H7_FDCAN_16_BYTES: return FDCAN_DATA_BYTES_16;
	case H7_FDCAN_20_BYTES: return FDCAN_DATA_BYTES_20;
	case H7_FDCAN_24_BYTES: return FDCAN_DATA_BYTES_24;
	case H7_FDCAN_32_BYTES: return FDCAN_DATA_BYTES_32;
	case H7_FDCAN_48_BYTES: return FDCAN_DATA_BYTES_48;
	case H7_FDCAN_64_BYTES: return FDCAN_DATA_BYTES_64;
	default: return FDCAN_DATA_BYTES_8;
	}
}

u16 H7_FDCAN_calcMemSize(H7_FDCANHandler_s *fdcan){
	u16 memSize = (fdcan->hfdcan->Init.ExtFiltersNbr * 2) 	+
			(fdcan->hfdcan->Init.StdFiltersNbr * 1) 		+

			(((fdcan->hfdcan->Init.RxFifo0ElmtsNbr 		* fdcan->hfdcan->Init.RxFifo0ElmtSize) 	+
			  (fdcan->hfdcan->Init.RxFifo1ElmtsNbr 		* fdcan->hfdcan->Init.RxFifo1ElmtSize) 	+
			  (fdcan->hfdcan->Init.TxFifoQueueElmtsNbr 	* fdcan->hfdcan->Init.TxElmtSize) 		+
			  (fdcan->hfdcan->Init.TxBuffersNbr 		* fdcan->hfdcan->Init.TxElmtSize)		+
			  (fdcan->hfdcan->Init.RxBuffersNbr 		* fdcan->hfdcan->Init.RxBufferSize))) 	+
			  (fdcan->hfdcan->Init.TxEventsNbr * 2);

	return memSize;
}

