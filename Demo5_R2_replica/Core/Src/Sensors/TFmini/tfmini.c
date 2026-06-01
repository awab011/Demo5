/*
 * tfmini.c
 *
 *  Created on: 2 May 2022
 *      Author: wai
 */
#include "tfmini.h"

void TFmini_Init(tfmini_t* tfmini, UART_HandleTypeDef* huartx){
	tfmini->huartx = huartx;
	tfmini->checksum = 0;
//	tfmini->Sum = 0;
//	tfmini->WindowPointer = 0;
//
//	for(uint32_t i=0; i<Window_L; i++)
//	{
//		tfmini->History[i] = 0;
//	}
	Moving_Average_Init(&tfmini->Mov_Ave,&tfmini->dist,&tfmini->f_dist);
	HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 1);
}
//
//void TFmini_set(tfmini_t* tfmini, unsigned char setting, ...){
//	va_list value;
//	va_start(value, setting);
//	tfmini->command = setting;
//	switch(tfmini->command){
//	case FIRMWARE_ver:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x01, 0x5F);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		//5A 07 01 V1 V2 V3 SU Version V3.V2.V1
//		break;
//	case TRIGGER_DETECTION:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x04, 0x62);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		After setting the frame rate to 0，detection can be triggered with this 	command
//		break;
//	case FRAME_RATE:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		sprintf(tfmini->buff, "%x%x%x%x%x%x", 0x5A, 0x04, 0x03, tfmini->buf1, tfmini->buf2, tfmini->checksum);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		//Frame rate 5A 06 03 LL HH SU
//		//5A 06 03 LL HH SU 1-1000Hz①
//		break;
//	case DATA_FRAME:
//		break;
//	case BAUD_RATE:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		sprintf(tfmini->buff, "%x%x%x%x%x%x%x%x", 0x5A, 0x04, 0x03, tfmini->buf1, tfmini->buf2, tfmini->buf3, tfmini->buf4, tfmini->checksum);
////		5A 08 06 H1 H2 H3 H4 SU Set baud rate③
//		break;
//	case OUTPUT_FORMAT:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		if(tfmini->setbuf == 1)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x01, 0x65);
//		if(tfmini->setbuf == 2)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x02, 0x66);
//		if(tfmini->setbuf == 3)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x05, 0x06, 0x6A);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
//		UARTPrintString(&huart5, tfmini->buff);
////		5A 05 05 01 65 Standard 9 bytes(cm), default
////		5A 05 05 02 66 Pixhaw
////		5A 05 05 06 6A Standard 9 bytes(mm)
//		break;
//	case OUTPUT_EN:
//		tfmini->setbuf = va_arg(value, uint32_t);
//		if(tfmini->setbuf == 1)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x07, 0x00, 0x66);
//		if(tfmini->setbuf == 2)
//			sprintf(tfmini->buff, "%x%x%x%x%x", 0x5A, 0x05, 0x07, 0x01, 0x67);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 07 00 66 Disable tfmini->buff output
////		5A 05 07 01 67 Enable tfmini->buff output, default
//		break;
//	case SYSTEM_RESET:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x02, 0x60);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 02 00 60 Succeeded
////		5A 05 02 01 61 Failed
//		break;
//	case FACTORY_SET:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x10, 0x6E);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 10 00 6E Succeeded
////		5A 05 10 01 6F Failed
//		break;
//	case SAVE:
//		sprintf(tfmini->buff, "%x%x%x%x", 0x5A, 0x04, 0x11, 0x6F);
//		UARTPrintString(tfmini->huartx, tfmini->buff);
////		5A 05 11 00 6F Succeeded
////		5A 05 11 01 70 Failed
//		break;
//	}
//	va_end(value);
//}

void TFmini_Handler(tfmini_t* tfmini){
//	static uint32_t tfledtick = 0;
	switch(tfmini->state){
	case PENDING_SYNC:
		if(tfmini->buff[0] == 0x59){
			tfmini->state = CONFIRMING_SYNC;
			HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 1);
		}else{
			tfmini->state = PENDING_SYNC;
			HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 1);
		}
		break;
	case CONFIRMING_SYNC:
		if(tfmini->buff[0] == 0x59){
			tfmini->state = IN_SYNC;
			HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 7);
		}else{
			tfmini->state = PENDING_SYNC;
			HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 1);
		}
		break;
	case IN_SYNC:
		tfmini->checksum = 0;
		tfmini->checksum = tfmini->buff[0] + tfmini->buff[1] + tfmini->buff[2] + tfmini->buff[3] + tfmini->buff[4] + tfmini->buff[5];
		//		if(tfmini->checksum == tfmini->buff[8]){
		tfmini->dist = *((int16_t*)&tfmini->buff[0]) / 100.0;
		tfmini->str  = *((int16_t*)&tfmini->buff[2]);
		tfmini->temp = *((int16_t*)&tfmini->buff[4]) / 8 - 256;
		if(tfmini->dist < 0.02)
			tfmini->dist = 13.0;
		tfmini->dist -= TF_OFFSET;
		Moving_Average_Filter(&tfmini->Mov_Ave);
//		tfledtick ++;
//		if(tfledtick >= 20){
//			GPIOC_OUT->bit14 = !GPIOC_OUT->bit14;	//led2
//			tfledtick = 0;
//		}
		tfmini->state = PENDING_SYNC;
		HAL_UART_Receive_IT(tfmini->huartx, tfmini->buff, 1);
		break;
	}
}

