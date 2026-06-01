/**
 * @file    spi.h
 * @brief   SPI header driver
 * @author  Mohammed Abdulalem
 * @date    2026-1
 *
 * @details
 *
 * @note
 * @ingroup Communication
 */

#ifndef SRC_BIOS_COM_SPI_H_
#define SRC_BIOS_COM_SPI_H_



//** Includes **//
#include "../../Platform/H7_system.h"
#include "../gpio.h"
#include "../../Platform/priorities.h"

//** Macros **//

// SPI Modes
#ifndef SPI_MODE_SLAVE
#define SPI_MODE_SLAVE                                (0x00000000UL)
#endif

#ifndef SPI_MODE_MASTER
#define SPI_MODE_MASTER                               SPI_CFG2_MASTER
#endif

// DMA Modes
#ifndef DMA_NORMAL
#define DMA_NORMAL              ((uint32_t)0x00000000U)                  /*!< Normal mode                                    */
#endif
#ifndef DMA_CIRCULAR
#define DMA_CIRCULAR            ((uint32_t)DMA_SxCR_CIRC)                /*!< Circular mode                                  */
#endif
#ifndef DMA_PFCTRL
#define DMA_PFCTRL              ((uint32_t)DMA_SxCR_PFCTRL)              /*!< Peripheral flow control mode                   */
#endif
#ifndef DMA_DOUBLE_BUFFER_M0
#define DMA_DOUBLE_BUFFER_M0    ((uint32_t)DMA_SxCR_DBM)                 /*!< Double buffer mode with first target memory M0 */
#endif
#ifndef DMA_DOUBLE_BUFFER_M1
#define DMA_DOUBLE_BUFFER_M1    ((uint32_t)(DMA_SxCR_DBM | DMA_SxCR_CT)) /*!< Double buffer mode with first target memory M1 */
#endif

// DMA Priorities
#ifndef DMA_PRIORITY_LOW
#define DMA_PRIORITY_LOW             ((uint32_t)0x00000000U)    /*!< Priority level: Low       */
#endif
#ifndef DMA_PRIORITY_MEDIUM
#define DMA_PRIORITY_MEDIUM          ((uint32_t)DMA_SxCR_PL_0)  /*!< Priority level: Medium    */
#endif
#ifndef DMA_PRIORITY_HIGH
#define DMA_PRIORITY_HIGH            ((uint32_t)DMA_SxCR_PL_1)  /*!< Priority level: High      */
#endif
#ifndef DMA_PRIORITY_VERY_HIGH
#define DMA_PRIORITY_VERY_HIGH       ((uint32_t)DMA_SxCR_PL)    /*!< Priority level: Very High */
#endif

#define SPI_RX_BUF_SIZE 8
#define SPI_TX_BUF_SIZE 8

//------------------- Enumeration -------------------//
typedef enum{
	SPI_DATA_SIZE_4B = (0x00000003UL),
	SPI_DATA_SIZE_5B,
	SPI_DATA_SIZE_6B,
	SPI_DATA_SIZE_7B,
	SPI_DATA_SIZE_8B,
	SPI_DATA_SIZE_9B,
	SPI_DATA_SIZE_10B,
	SPI_DATA_SIZE_11B,
	SPI_DATA_SIZE_12B,
	SPI_DATA_SIZE_13B,
	SPI_DATA_SIZE_14B,
	SPI_DATA_SIZE_15B,
	SPI_DATA_SIZE_16B,
	SPI_DATA_SIZE_17B,
	SPI_DATA_SIZE_18B,
	SPI_DATA_SIZE_19B,
	SPI_DATA_SIZE_20B,
	SPI_DATA_SIZE_21B,
	SPI_DATA_SIZE_22B,
	SPI_DATA_SIZE_23B,
	SPI_DATA_SIZE_24B,
	SPI_DATA_SIZE_25B,
	SPI_DATA_SIZE_26B,
	SPI_DATA_SIZE_27B,
	SPI_DATA_SIZE_28B,
	SPI_DATA_SIZE_29B,
	SPI_DATA_SIZE_30B,
	SPI_DATA_SIZE_31B,
	SPI_DATA_SIZE_32B
} H7_SPI_dataSize_e;

typedef enum{
	SPI_SLAVE			= SPI_BAUDRATEPRESCALER_2,	// Speed is ignored in slave mode
	SPI_SPEED_100MHZ 	= SPI_BAUDRATEPRESCALER_2,
	SPI_SPEED_50MHZ		= SPI_BAUDRATEPRESCALER_4,
	SPI_SPEED_25MHZ		= SPI_BAUDRATEPRESCALER_8,
	SPI_SPEED_12_5MHZ	= SPI_BAUDRATEPRESCALER_16,	// 12.5 MHz
	SPI_SPEED_6_25MHZ	= SPI_BAUDRATEPRESCALER_32,	// 6.25 MHz
	SPI_SPEED_3_125MHZ	= SPI_BAUDRATEPRESCALER_64,	// 3.125 MHz
	SPI_SPEED_1_5625MHZ	= SPI_BAUDRATEPRESCALER_128,// 1.1625 MHz
	SPI_SPEED_781_25KHZ	= SPI_BAUDRATEPRESCALER_256	// 781.25 KHz
} H7_SPI_speed_e;

//------------------- Structures -------------------//
typedef struct{
	SPI_HandleTypeDef *hspi;

	// NSS Pin	in H7.1.0 Board it is PB9
//	GPIO_TypeDef* GPIOx_NSS;
//	u16 GPIO_Pin_NSS;

	DMA_HandleTypeDef hdma_spi_rx;
	DMA_HandleTypeDef hdma_spi_tx;

	u8 rxData[SPI_RX_BUF_SIZE];
	u8 txData[SPI_TX_BUF_SIZE];

	H7_state_e status;
} H7_SPIHandler_s;

//------------------- Global Variables -------------------//
extern SPI_HandleTypeDef spi2;
extern H7_SPIHandler_s h7spi2;

//------------------- Function Declaration -------------------//

H7_state_e H7_struct_init(H7_SPIHandler_s *spi, SPI_HandleTypeDef *hspi);
H7_state_e H7_SPIx_Init(H7_SPIHandler_s *spi, u32 spiMode, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize);
H7_state_e H7_SPIx_rx_DMA_init(H7_SPIHandler_s *spi, u32 spiMode, u32 DMA_mode, u32 priority, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize);
H7_state_e H7_SPIx_tx_DMA_init(H7_SPIHandler_s *spi, u32 spiMode, u32 DMA_mode, u32 priority, H7_SPI_speed_e spiSpeed, H7_SPI_dataSize_e dataSize);


#endif /* SRC_BIOS_COM_SPI_H_ */
