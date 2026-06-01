/*
 * esp.h
 *
 *  Created on: Dec 14, 2025
 *      Author: moon
 */

#ifndef SRC_ESP_ESP_H_
#define SRC_ESP_ESP_H_

#include "../../BIOS.h"

/* MAINBOARD TO ESP MSG FRAME */
#define TX_HEADER1_BYTE	0xAA
#define TX_HEADER2_BYTE	0xCC
#define TX_END_BYTE		0xEE
#define TX_MSG_SIZE		81

typedef union
{
    struct __attribute__((packed))
	{
		uint8_t header1;
		uint8_t header2;
		float actual_x;
		float actual_y;
		int32_t qei_x; // raw qei values, can cast int16_t inside if needed
		int32_t qei_y;
		float current_yaw;
		float target_yaw;
		uint8_t navistate;
		uint8_t ppstate;
		uint8_t byte1;
		uint8_t byte2;
		uint8_t byte3;
		uint8_t byte4;
		int32_t int1;
		int32_t int2;
		int32_t int3;
		int32_t int4;
		float float1;
		float float2;
		float float3;
		float float4;
		double double1;
		double double2;
		uint8_t endbyte; // XOR all bytes including header except checksum
	};

    uint8_t data_arr[TX_MSG_SIZE];

} MAINBOARD_Tx_t;


/* ESP TO MAINBOARD MSG FRAME */
#define RX_HEADER1_BYTE	0xBB
#define RX_HEADER2_BYTE	0xDD
#define RX_CUTOFF_BYTE	34
#define RX_END_BYTE		0xFF
#define RX_MSG_SIZE		92

typedef union
{
    struct __attribute__((packed))
	{
    	uint8_t header1;
    	uint8_t header2;

        struct __attribute__((packed))
        {
            uint8_t cam_flag1;
            uint8_t cam_flag2;
            float pos_x;
            float pos_y;
            int32_t err_x;
            int32_t err_y;
        } cam;

        struct __attribute__((packed))
        {
            uint8_t rgbd_flag1;
            uint8_t rgbd_flag2;
            float trans_x;
            float trans_y;
            float trans_z; // last byte of PAYLOAD1 : rx_buf[33] -> byte 34
        } rgbd;

        struct __attribute__((packed))
        {
            uint8_t lsr_flag1;
            uint8_t lsr_flag2;
            uint8_t connection;
            float laser_xl;
            float laser_xr;
            float laser_yf;
            float laser_yb;
        } lsr;

        struct __attribute__((packed))
        {
            uint8_t ldr_flag1;
            uint8_t ldr_flag2;
            float angle;
            float lidar_xl;
            float lidar_xr;
            float lidar_yf;
            float lidar_yb;
        } ldr;

        struct __attribute__((packed))
		{
			uint8_t length;
			uint8_t pinpoint[15];
		} web;

        uint8_t endbyte;
    };

    uint8_t data_arr[RX_MSG_SIZE];

} MAINBOARD_Rx_t;

typedef enum
{
	TX_N_RX = 0,
	TX_ONLY,
	RX_ONLY
} ESP_UART_Enable_t;

typedef enum
{
	PENDING_HEADER1 = 0,
	PENDING_HEADER2,
	PAYLOAD1,
	PAYLOAD2,
	PENDING_ENDBYTE,
	RX_DISABLE
} ESP_UART_State_t;

typedef struct
{
	UART_HandleTypeDef* huartx;
	MAINBOARD_Tx_t tx_msg;
	MAINBOARD_Rx_t rx_msg;
	uint8_t rx_buf[RX_MSG_SIZE];
	volatile bool rx_dma_cplt;
	volatile uint8_t rx_idx;
	volatile ESP_UART_State_t rx_state;
} ESP_Handler_t;

ESP_Handler_t esp, esp_mbtx, esp_mbrx;
MAINBOARD_Rx_t ros2_msg;

void ESP_Init(ESP_Handler_t *esp, H7_UARTHandler_s *h7uart, ESP_UART_Enable_t en);
void ESP_Tx_Handler(ESP_Handler_t *esp);
void ESP_Rx_Handler(ESP_Handler_t *esp);

#endif /* SRC_ESP_ESP_H_ */
