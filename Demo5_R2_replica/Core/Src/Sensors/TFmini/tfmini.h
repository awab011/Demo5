/*
 * tfmini.h
 *
 *  Created on: 2 May 2022
 *      Author: wai
 */

#ifndef SRC_TFMINI_TFMINI_H_
#define SRC_TFMINI_TFMINI_H_

#include "../../BIOS/BIOS.h"
#include "../../Algorithms/Moving_Average/mov_ave.h"
//#define Window_L 10
#define TF_OFFSET 0.17
enum {
	FIRMWARE_ver,
	TRIGGER_DETECTION,
	FRAME_RATE,
	DATA_FRAME,
	BAUD_RATE,
	OUTPUT_FORMAT,
	OUTPUT_EN,
	SYSTEM_RESET,
	FACTORY_SET,
	SAVE
};

//enum {PENDING_SYNC = 0, CONFIRMING_SYNC, IN_SYNC};

typedef struct {

	UART_HandleTypeDef* huartx;
	volatile uint8_t checksum;
	uint8_t state;
	unsigned char command;
	uint8_t buff[10];

	volatile float dist;	//in m
	volatile float f_dist;
	volatile float str;
	volatile float temp;
	float dist_target;
	float dist_err;
	union{
		uint32_t setbuf;
		struct{
			char buf1;
			char buf2;
			char buf3;
			char buf4;
		};
	};
	Mov_Ave_t Mov_Ave;

//	float History[Window_L]; /*Array to store values of filter window*/
//	float Sum;	/* Sum of filter window's elements*/
//	uint32_t WindowPointer; /* Pointer to the first element of window*/
}tfmini_t;

void TFmini_Init(tfmini_t* tfmini, UART_HandleTypeDef* huartx);
void TFmini_set(tfmini_t* tfmini, unsigned char setting, ...);
void TFmini_Handler(tfmini_t* tfmini);
//void TFmini_Moving_Average_Filter(tfmini_t *Mov_Ave);

#endif /* SRC_TFMINI_TFMINI_H_ */
