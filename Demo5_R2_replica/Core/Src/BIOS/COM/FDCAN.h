/**
 * @file    FDCAN.h
 * @brief   FDCAN header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note	You have to define the FDCAN you are using before initialization
 * @ingroup Communication
 */

#ifndef SRC_FDCAN_H_
#define SRC_FDCAN_H_



//** Includes **//
#include "../../Platform/H7_system.h"
#include "main.h"


//** Macros **//
#define TOTAL_MESSAGE_RAM_WORDS 2560

#define H7FDCAN1 // User enable this if using
#define H7FDCAN2 // User enable this if using
#define H7FDCAN3 // User enable this if using

#if defined(H7FDCAN1) && defined(H7FDCAN2) && defined(H7FDCAN3)

#define FDCAN1_MESSAGE_RAM_OFFSET 	0		// Allowable size is 853 words
#define FDCAN2_MESSAGE_RAM_OFFSET 	853		// Allowable size is 853 words
#define FDCAN3_MESSAGE_RAM_OFFSET 	1706	// Allowable size is 854 words
#define RAM_SIZE_ALLOW				853

#elif defined(H7FDCAN1) && defined(H7FDCAN2) && !defined(H7FDCAN3)

#define FDCAN1_MESSAGE_RAM_OFFSET 	0		// Allowable size is 1280 words
#define FDCAN2_MESSAGE_RAM_OFFSET 	1280	// Allowable size is 1280 words
#define RAM_SIZE_ALLOW				1280

#elif defined(H7FDCAN1) && !defined(H7FDCAN2) && defined(H7FDCAN3)

#define FDCAN1_MESSAGE_RAM_OFFSET 	0		// Allowable size is 1280 words
#define FDCAN3_MESSAGE_RAM_OFFSET 	1280	// Allowable size is 1280 words
#define RAM_SIZE_ALLOW				1280

#elif !defined(H7FDCAN1) && defined(H7FDCAN2) && defined(H7FDCAN3)

#define FDCAN2_MESSAGE_RAM_OFFSET 	0		// Allowable size is 1280 words
#define FDCAN3_MESSAGE_RAM_OFFSET 	1280	// Allowable size is 1280 words
#define RAM_SIZE_ALLOW				1280

#elif defined(H7FDCAN1) && !defined(H7FDCAN2) && !defined(H7FDCAN3)
#define FDCAN1_MESSAGE_RAM_OFFSET 	0		// Allowable size is 2560 words
#define RAM_SIZE_ALLOW				2560
#elif !defined(H7FDCAN1) && defined(H7FDCAN2) && !defined(H7FDCAN3)
#define FDCAN2_MESSAGE_RAM_OFFSET 	0		// Allowable size is 2560 words
#define RAM_SIZE_ALLOW				2560
#elif !defined(H7FDCAN1) && !defined(H7FDCAN2) && defined(H7FDCAN3)
#define FDCAN3_MESSAGE_RAM_OFFSET 	0		// Allowable size is 2560 words
#define RAM_SIZE_ALLOW				2560
#else
#define RAM_SIZE_ALLOW				0
#endif

//------------------- Enumeration -------------------//

typedef enum{
	FDCAN_500kbps,
	FDCAN_1Mbps
} H7_fdcan_baudrate_e;

typedef enum{
	H7_FDCAN_8_BYTES,
	H7_FDCAN_12_BYTES,
	H7_FDCAN_16_BYTES,
	H7_FDCAN_20_BYTES,
	H7_FDCAN_24_BYTES,
	H7_FDCAN_32_BYTES,
	H7_FDCAN_48_BYTES,
	H7_FDCAN_64_BYTES
} H7_fdcan_data_size;



//------------------- Structures -------------------//
typedef struct{
	FDCAN_HandleTypeDef *hfdcan;
	FDCAN_RxHeaderTypeDef rxHeader;
	FDCAN_TxHeaderTypeDef txHeader;
	u8 rxData[8];
	u8 txData[8];
	uint8_t filter_count;
	u16 memRAMSize;
	H7_state_e status;
} H7_FDCANHandler_s;

//------------------- Global Variables -------------------//





//** Global variables **//
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern H7_FDCANHandler_s h7fdcan1;
extern H7_FDCANHandler_s h7fdcan2;
extern H7_FDCANHandler_s h7fdcan3;


//------------------- Function Declaration -------------------//
H7_state_e H7_FDCANHandler_init(H7_FDCANHandler_s *fdcan, FDCAN_HandleTypeDef* h7fdcan);
H7_state_e H7_FDCAN_init(H7_FDCANHandler_s *fdcan, u32 STD_Filters_Bank, u32 EXD_Filters_Bank, u32 rxFIFO0Elmnts, u32 rxFIFO1Elmnts, u32 txFIFOElmnts, u32 txFIFO_operation_mode, u32 FIFOBuffer, H7_fdcan_baudrate_e data_rate, H7_fdcan_data_size size);
H7_state_e H7_FDCAN_addFilterRange(H7_FDCANHandler_s *fdcan, uint32_t idType, uint32_t ID1, uint32_t ID2, uint32_t storeTo);
H7_state_e FDCAN_TxMsg(FDCAN_HandleTypeDef *hfdcan, u32 ID, u8 *msg, u8 len);
H7_state_e FDCAN_TxMsgEID(FDCAN_HandleTypeDef* hfdcan,uint32_t EID,uint8_t *Msg,uint8_t len);
u32 convert_to_DLC(u8 len);
u32 convert_to_data_bytes(H7_fdcan_data_size size);
u16 H7_FDCAN_calcMemSize(H7_FDCANHandler_s *fdcan);


#endif /* SRC_FDCAN_H_ */
