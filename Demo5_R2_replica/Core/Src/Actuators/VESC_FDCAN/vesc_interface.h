#ifndef VESC_CAN_VESC_INTERFACE_H_
#define VESC_CAN_VESC_INTERFACE_H_

#include "../../Actuators/VESC_FDCAN/vesc_fdcan.h"
#include "../../Actuators/VESC_FDCAN/vesc_uart.h"
//#include "../BIOS/bios.h"

void VESCInit(uint8_t id1, uint8_t id2, uint8_t id3, uint8_t id4, VESC_t* vesc);
void VESCUARTInit(UART_HandleTypeDef* huart1, UART_HandleTypeDef* huart2, UART_HandleTypeDef* huart3, UART_HandleTypeDef* huart4, VESC_t* vesc);
void VESCVelocity(float FLeftVel, float FRightVel, float BLeftVel, float BRightVel, VESC_t* vesc);
void VESCRPM(float FLeftRPM, float FRightRPM, float BLeftRPM, float BRightRPM, VESC_t* vesc);
void VESCPDC(float FLeftRDC, float FRightRDC, float BLeftRDC, float BRightRDC, VESC_t* vesc);
void VESCPOS(float FLeftPOS, float FRightPOS, float BLeftPOS, float BRightPOS, VESC_t* vesc);
void VESCCurr(float FLeftCurr, float BLeftCurr, float FRightCurr, float BRightCurr, VESC_t* vesc);
void VESCStop(VESC_t* vesc);

void VESCEnquire(uint32_t status, VESC_t* vesc);

void VESCGetSpeed(VESC_t* vesc);
void VESCGetDist(VESC_t* vesc);
void VESCSetPosOffset(float arg, unsigned char flash, uint16_t id);
void VESCSetDist(float dist, uint16_t id, VESC_t* vesc);
void VESCReleaseMotor(VESC_t* vesc);
void VESCHandBrake(float HBrakeCurr, VESC_t* vesc);
void VESCCurrBrake(float BrakeCurr, VESC_t* vesc);
void VESCFullBrake(VESC_t* vesc);

#endif /* VESC_CAN_VESC_INTERFACE_H_ */
