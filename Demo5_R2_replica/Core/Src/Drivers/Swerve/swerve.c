/*
 * swerve->c
 *
 *  Created on:
 *      Author:
 */

#include "swerve.h"
#include <math.h>


Swerve_t swerve;
//void SwerveRun(Swerve_t *swerve)
//{
//	if (swerve->aligned)
//	{
//      if((fabs(swerve->vel[0])+fabs(swerve->vel[1])+fabs(swerve->vel[2])+fabs(swerve->vel[3]))>0){
//				for(int i = 0; i < 4; i++)
//				{
//					if(swerve->ang[i] - swerve->angprev[i] < -180.0)
//					{
//						swerve->addvalue[i] += 360.0;
//					}
//					else if(swerve->ang[i] - swerve->angprev[i] > 180.0)
//					{
//						swerve->addvalue[i] -= 360.0;
//					}
//
//					swerve->fang[i] = swerve->ang[i] + swerve->addvalue[i];
//
//					swerve->ang1[i] = swerve->fang[i] - swerve->fangprev[i];
//					swerve->ang2[i] = (swerve->ang1[i] >= 0)? swerve->ang1[i] - 180.0 : swerve->ang1[i] + 180.0;
//					swerve->angprev[i] = swerve->ang[i];
//					swerve->fangprev[i] = swerve->fang[i];
//
//					if(swerve->state[i] == 0)
//					{
//						swerve->finalang[i] = swerve->finalangprev[i] + swerve->ang1[i];
//						swerve->sign[i] = 1;
//						if(fabs(swerve->ang1[i]) > fabs(swerve->ang2[i]))
//						{
//							swerve->state[i] = 1;
//							swerve->finalang[i] = swerve->finalangprev[i] + swerve->ang2[i];
//							swerve->sign[i] = -1;
//						}
//					}
//					else if(swerve->state[i] == 1)
//					{
//						swerve->finalang[i] = swerve->finalangprev[i] + swerve->ang1[i];
//						swerve->sign[i] = -1;
//						if(fabs(swerve->ang1[i]) > fabs(swerve->ang2[i]))
//						{
//							swerve->state[i] = 0;
//							swerve->finalang[i] = swerve->finalangprev[i] + swerve->ang2[i];
//							swerve->sign[i] = 1;
//						}
//					}
//
//					swerve->finalangdif[i] = swerve->finalang[i]-swerve->finalangprev[i];
//					swerve->finalangprev[i] = swerve->finalang[i];
//				}
//
//				swerve->fvel[0] = swerve->vel[0] * swerve->sign[0]  ;
//				swerve->fvel[1] = swerve->vel[1] * swerve->sign[1]  ;
//				swerve->fvel[2] = swerve->vel[2] * swerve->sign[2]  ;
//				swerve->fvel[3] = swerve->vel[3] * swerve->sign[3]  ;
//
//				VESCVelocity(swerve->fvel[0], swerve->fvel[1], swerve->fvel[2], swerve->fvel[3],swerve->vesc);
//
//				RBMS_Set_Target_Position(swerve->rbms, RBMS1, swerve->finalang[0] / 360.0 );
//				RBMS_Set_Target_Position(swerve->rbms, RBMS2, swerve->finalang[1] / 360.0 );
//				RBMS_Set_Target_Position(swerve->rbms, RBMS3, swerve->finalang[2] / 360.0 );
//				RBMS_Set_Target_Position(swerve->rbms, RBMS4, swerve->finalang[3] / 360.0 );
//				swerve->timer=0;
//
//
//      }
//      else{
//			VESCVelocity(0,0,0,0,swerve->vesc);
//
//            if(swerve->timer>3){
////            }
//			RBMS_Set_Target_Position(&rbms1, RBMS1, 0);
//			RBMS_Set_Target_Position(&rbms1, RBMS2, 0);
//			RBMS_Set_Target_Position(&rbms1, RBMS3, 0);
//			RBMS_Set_Target_Position(&rbms1, RBMS4, 0);
//
//			for(int i = 0; i < 4; i++)
//			{
//				swerve->ang[i] = 0.0;
//				swerve->angprev[i] = 0.0;
//				swerve->addvalue[i] = 0.0;
//				swerve->fang[i] = 0.0;
//				swerve->fangprev[i] = 0.0;
//				swerve->finalang[i] = 0.0;
//				swerve->finalangprev[i] = 0.0;
//				swerve->closestzero[i] = 0.0;
//				swerve->state[i] = 0;
//			}
//          }
//      }
//
//	}
//	else{
//		if(swerve->aligndone[0] && swerve->aligndone[1] && swerve->aligndone[2] && swerve->aligndone[3]){
//
//			swerve->aligned=1;
//		    swerve->rbms->motor[RBMS1].offset_pos = -0.125;
//		    swerve->rbms->motor[RBMS2].offset_pos =  0.125;
//			swerve->rbms->motor[RBMS3].offset_pos =  0.125;
//			swerve->rbms->motor[RBMS4].offset_pos = -0.125;
//		}
//	}
//
//}


void swerve_enc_init(GPIO_TypeDef * encGPIOx, uint16_t encGPIO_Pin){
	H7_gpio_PinInit(encGPIOx, encGPIO_Pin, GPIO_MODE_IT_RISING_FALLING, GPIO_SPEED_FREQ_HIGH,GPIO_NOPULL);
//	uint32_t prior = encGPIO_Pin - 1;
	H7_EXTI_init(&EXTI_Handler, encGPIO_Pin, 9);
}


void swerve_init(swerve_allign_t* swerve, float swerve_gear_ratio, uint8_t rbmsid, float * enc){
//	swerve_v2.turnmode = unlimitedturn;
	swerve->RBMSID = rbmsid - 1;

	RBMS_Config(&rbms1, swerve->RBMSID, C610, swerve_gear_ratio); //FL
	RBMS_PID_Init(&rbms1);

	swerve->enc = enc;

	PIDSourceInit(&swerve->enc_target_err, &swerve->enc_output, &swerve->enc_align_pid);
	PIDGainInit(0.005, 1.0, 1.0/180.0, 1.0, 5.0, 0.0, 0.001, 60, &swerve->enc_align_pid);

	swerve->swerve_init = 1;
//	PIDGainInit(ts, sat, ke, ku, kp, ki, kd, kn, pid);
}


//credit to Mr. VinCent
void swerve_allign(swerve_allign_t* swerve, GPIO_TypeDef * hallGPIOx, uint16_t hallGPIO_Pin, float offset, float enc_target){


	switch(swerve->alligning_status){
	case SWERVE_CLOCKWISE:
		swerve->alligning_angle1 = 0.0;
		swerve->alligning_angle2 = 0.0;
		swerve->cur_enc_angle = 0.0;
		swerve->enc_output = 0.0;
		swerve->enc_target_err = 0.0;
		swerve->prev_enc_angle = 0.0;
		swerve->turns = 0;
		swerve->enc_target = enc_target;

		rbms1.motor[swerve->RBMSID].config.vel_limit = 500; //high speed will caz detector didnt detect
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);

		if (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, -500.0);
			while (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
				RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, -500.0);
				swerve->alligning_angle2 = rbms1.motor[swerve->RBMSID].pos;

			}
		}
		RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0.0);
		swerve->alligning_status = SWERVE_ANTICLOCKWISE;
		break;

	case SWERVE_ANTICLOCKWISE:
		osDelay(100);
		if (fabs(swerve->alligning_angle2) < 0.45) { //Mr V use 0.25 caz he got 2 hall XD
			swerve->alligning_angle1 = (swerve->alligning_angle2 - rbms1.motor[swerve->RBMSID].pos);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, swerve->alligning_angle1);
			osDelay(250);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			swerve->alligning_status = SWERVE_ALLIGN_ENC;
		}

		else {
//			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 500.0);
			osDelay(100); //uncomment if ur rbms speed limit is slow

			while (HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) {
				RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 500.0);
				swerve->alligning_angle2 = rbms1.motor[swerve->RBMSID].pos;
			}

			swerve->alligning_angle1 = (swerve->alligning_angle2 - rbms1.motor[swerve->RBMSID].pos);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, swerve->alligning_angle1);
			osDelay(250);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			swerve->alligning_status = SWERVE_ALLIGN_ENC;

			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0.0);
			osDelay(100);

		}
		break;



	case SWERVE_ALLIGN_ENC:
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, VELOCITY);

		uint32_t swervecount = 0;
		while((swervecount<500000 || HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1) || swerve->enc_target_err > 0.5){
			RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, swerve->enc_output*100.0);
			swervecount++;
		}
		RBMS_Set_Target_Velocity(&rbms1, swerve->RBMSID, 0);
		osDelay(100);
		RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
		osDelay(100);

		//if alignment fail, it will keep restart.
		if(HAL_GPIO_ReadPin(hallGPIOx, hallGPIO_Pin) == 1){
			swerve->alligning_status = SWERVE_CLOCKWISE;
		}else{
			rbms1.motor[swerve->RBMSID].config.vel_limit = 500;
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			RBMS_Set_Target_Position(&rbms1, swerve->RBMSID, offset);
			osDelay(1000);
			RBMS_Set_Control_Mode(&rbms1, swerve->RBMSID, POSITION);
			osDelay(500);
			swerve->alligning_status = SWERVE_ALLIGNED;

		}

//		swerve->alligning_status = SWERVE_ALLIGNED;


		break;

	case SWERVE_ALLIGNED:


		break;
	}
}

void SwerveCalcAngle(swerve_allign_t* swerve){
//	if(swerve == &swerveA){PWMEncoder_Angle_Update(&enc1); swerve->raw_enc_angle = enc1.Angle;}
//	if(swerve == &swerveB){PWMEncoder_Angle_Update(&enc2); swerve->raw_enc_angle = enc2.Angle;}
//	if(swerve == &swerveC){PWMEncoder_Angle_Update(&enc3); swerve->raw_enc_angle = enc3.Angle;}
//	if(swerve == &swerveD){PWMEncoder_Angle_Update(&enc4); swerve->raw_enc_angle = enc4.Angle;}

//	PWMEncoder_Angle_Update(swerve->enc);
	swerve->raw_enc_angle = *(swerve->enc);


	if (swerve->swerve_init == 1) {

		/*
		 * The value after correction too weird, sus the magnetic enc jumping, need KF
		 *
		 if (swerve->raw_enc_angle - swerve->prev_enc_angle < -350) {
		 swerve->turns++;
		 } else if (swerve->raw_enc_angle - swerve->prev_enc_angle > 350) {
		 swerve->turns--;
		 }
		 swerve->cur_enc_angle = (float)swerve->turns * 360.0 + swerve->raw_enc_angle;
		 swerve->prev_enc_angle = swerve->raw_enc_angle;


		 */

		swerve->cur_enc_angle = swerve->raw_enc_angle;
		swerve->enc_target_err = swerve->enc_target - swerve->cur_enc_angle;
		PID(&swerve->enc_align_pid);
	}
}




