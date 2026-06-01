/********************************* ﺐﺴﻣ ﺎﻠﻠﻫ ﺎﻟﺮﺤﻤﻧ ﺎﻟﺮﺤﻴﻣ *********************************
 * BMM350.h
 *
 *  Created on: Apr 27, 2025
 *  recreated on : May 3, 2025

 *      Author: MOAHMMED ABDULALEM | ﻢﺤﻣﺩ ﻊﺑﺩ ﺎﻠﻋﺎﻠﻣ
 *		version: 1.00
 */

#ifndef SRC_BMM350_IMU_BMM350_H_
#define SRC_BMM350_IMU_BMM350_H_


/*************************************************************
 * ********************* Includes ****************************
 ************************************************************/
#include "../../../adapter.h"



/*************************************************************
 * ********************* Macros*******************************
 ************************************************************/

/* Macro to for debugging*/
#define DEB_SEND
/* Macro to SET and GET BITS of a register*/
#define BMM350_SET_BITS(reg_data, bitname, data)  		((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))

#define BMM350_GET_BITS(reg_data, bitname)          	((reg_data & (bitname##_MSK)) >> (bitname##_POS))

#define BMM350_GET_BITS_POS_0(reg_data, bitname)    	(reg_data & (bitname##_MSK))

#define BMM350_SET_BITS_POS_0(reg_data, bitname, data)  ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))


/*! Chip id of BMM350 */
#define BMM350_CHIP_ID                              UINT8_C(0x33)

/*! Variant ID of BMM350 */
#define BMM350_MIN_VAR                              UINT8_C(0x10)

//#define ADSL_HIGH  // Uncomment if ADSL pin is HIGH

/*! I2C Address depending on ADSEL PIN STATE */
#ifdef ADSL_HIGH
	#define BMM350_I2C_ADDRESS                   UINT8_C(0x15)
#else
	#define BMM350_I2C_ADDRESS                   UINT8_C(0x14)
#endif

#define BMM350_DUMMY_BYTES                          UINT8_C(2)

/************************* Sensor delay time settings in microseconds **************************/
#define BMM350_SOFT_RESET_DELAY                     UINT32_C(24000)
#define BMM350_MAGNETIC_RESET_DELAY                 UINT32_C(40000)
#define BMM350_START_UP_TIME_FROM_POR               UINT32_C(3000)

#define BMM350_GOTO_SUSPEND_DELAY                   UINT32_C(6000)
#define BMM350_SUSPEND_TO_NORMAL_DELAY              UINT32_C(38000)

#define BMM350_SUS_TO_FORCEDMODE_NO_AVG_DELAY       UINT32_C(15000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_2_DELAY        UINT32_C(17000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_4_DELAY        UINT32_C(20000)
#define BMM350_SUS_TO_FORCEDMODE_AVG_8_DELAY        UINT32_C(28000)

#define BMM350_SUS_TO_FORCEDMODE_FAST_NO_AVG_DELAY  UINT32_C(4000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_2_DELAY   UINT32_C(5000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_4_DELAY   UINT32_C(9000)
#define BMM350_SUS_TO_FORCEDMODE_FAST_AVG_8_DELAY   UINT32_C(16000)

#define BMM350_UPD_OAE_DELAY                        UINT16_C(1000)

#define BMM350_BR_DELAY                             UINT16_C(14000)
#define BMM350_FGR_DELAY                            UINT16_C(18000)

/************************ Length macros ************************/
#define BMM350_OTP_DATA_LENGTH                      UINT8_C(32)
#define BMM350_READ_BUFFER_LENGTH                   UINT8_C(127)
#define BMM350_MAG_TEMP_DATA_LEN                    UINT8_C(12)

/******************************* AVG **************************/
#define BMM350_AVG_NO_AVG                           UINT8_C(0x0)
#define BMM350_AVG_2                                UINT8_C(0x1)
#define BMM350_AVG_4                                UINT8_C(0x2)
#define BMM350_AVG_8                                UINT8_C(0x3)

/******************************* ODR **************************/
#define BMM350_ODR_400HZ                            UINT8_C(0x2)
#define BMM350_ODR_200HZ                            UINT8_C(0x3)
#define BMM350_ODR_100HZ                            UINT8_C(0x4)
#define BMM350_ODR_50HZ                             UINT8_C(0x5)
#define BMM350_ODR_25HZ                             UINT8_C(0x6)
#define BMM350_ODR_12_5HZ                           UINT8_C(0x7)
#define BMM350_ODR_6_25HZ                           UINT8_C(0x8)
#define BMM350_ODR_3_125HZ                          UINT8_C(0x9)
#define BMM350_ODR_1_5625HZ                         UINT8_C(0xA)

/********************* Power modes *************************/
#define BMM350_PMU_CMD_SUS                          UINT8_C(0x00)
#define BMM350_PMU_CMD_NM                           UINT8_C(0x01)
#define BMM350_PMU_CMD_UPD_OAE                      UINT8_C(0x02)
#define BMM350_PMU_CMD_FM                           UINT8_C(0x03)
#define BMM350_PMU_CMD_FM_FAST                      UINT8_C(0x04)
#define BMM350_PMU_CMD_FGR                          UINT8_C(0x05)
#define BMM350_PMU_CMD_FGR_FAST                     UINT8_C(0x06)
#define BMM350_PMU_CMD_BR                           UINT8_C(0x07)
#define BMM350_PMU_CMD_BR_FAST                      UINT8_C(0x08)

#define BMM350_PMU_STATUS_0                         UINT8_C(0x0)

#define BMM350_CMD_NOP                              UINT8_C(0x0)
#define BMM350_CMD_SOFTRESET                        UINT8_C(0xB6)

#define BMM350_TARGET_PAGE_PAGE0                    UINT8_C(0x0)
#define BMM350_TARGET_PAGE_PAGE1                    UINT8_C(0x1)

#define BMM350_INT_MODE_LATCHED                     UINT8_C(0x1)
#define BMM350_INT_MODE_PULSED                      UINT8_C(0x0)

#define BMM350_INT_POL_ACTIVE_HIGH                  UINT8_C(0x1)
#define BMM350_INT_POL_ACTIVE_LOW                   UINT8_C(0x0)

#define BMM350_INT_OD_PUSHPULL                      UINT8_C(0x1)
#define BMM350_INT_OD_OPENDRAIN                     UINT8_C(0x0)

#define BMM350_INT_OUTPUT_EN_OFF                    UINT8_C(0x0)
#define BMM350_INT_OUTPUT_EN_ON                     UINT8_C(0x1)

#define BMM350_INT_DRDY_EN                          UINT8_C(0x1)
#define BMM350_INT_DRDY_DIS                         UINT8_C(0x0)

#define BMM350_MR_MR1K8                             UINT8_C(0x0)
#define BMM350_MR_MR2K1                             UINT8_C(0x1)
#define BMM350_MR_MR1K5                             UINT8_C(0x2)
#define BMM350_MR_MR0K6                             UINT8_C(0x3)

#define BMM350_SEL_DTB1X_PAD_PAD_INT                UINT8_C(0x0)
#define BMM350_SEL_DTB1X_PAD_PAD_BYP                UINT8_C(0x1)

#define BMM350_TMR_TST_HIZ_VTMR_VTMR_ON             UINT8_C(0x0)
#define BMM350_TMR_TST_HIZ_VTMR_VTMR_HIZ            UINT8_C(0x1)

#define BMM350_LSB_MASK                             UINT16_C(0x00FF)
#define BMM350_MSB_MASK                             UINT16_C(0xFF00)



/********************** Register Addresses ************************/

#define BMM350_REG_CHIP_ID                          UINT8_C(0x00)
#define BMM350_REG_REV_ID                           UINT8_C(0x01)
#define BMM350_REG_ERR_REG                          UINT8_C(0x02)
#define BMM350_REG_PAD_CTRL                         UINT8_C(0x03)
#define BMM350_REG_PMU_CMD_AGGR_SET                 UINT8_C(0x04)
#define BMM350_REG_PMU_CMD_AXIS_EN                  UINT8_C(0x05)
#define BMM350_REG_PMU_CMD                          UINT8_C(0x06)
#define BMM350_REG_PMU_CMD_STATUS_0                 UINT8_C(0x07)
#define BMM350_REG_PMU_CMD_STATUS_1                 UINT8_C(0x08)
#define BMM350_REG_I3C_ERR                          UINT8_C(0x09)
#define BMM350_REG_I2C_WDT_SET                      UINT8_C(0x0A)
#define BMM350_REG_TRSDCR_REV_ID                    UINT8_C(0x0D)
#define BMM350_REG_TC_SYNC_TU                       UINT8_C(0x21)
#define BMM350_REG_TC_SYNC_ODR                      UINT8_C(0x22)
#define BMM350_REG_TC_SYNC_TPH_1                    UINT8_C(0x23)
#define BMM350_REG_TC_SYNC_TPH_2                    UINT8_C(0x24)
#define BMM350_REG_TC_SYNC_DT                       UINT8_C(0x25)
#define BMM350_REG_TC_SYNC_ST_0                     UINT8_C(0x26)
#define BMM350_REG_TC_SYNC_ST_1                     UINT8_C(0x27)
#define BMM350_REG_TC_SYNC_ST_2                     UINT8_C(0x28)
#define BMM350_REG_TC_SYNC_STATUS                   UINT8_C(0x29)
#define BMM350_REG_INT_CTRL                         UINT8_C(0x2E)
#define BMM350_REG_INT_CTRL_IBI                     UINT8_C(0x2F)
#define BMM350_REG_INT_STATUS                       UINT8_C(0x30)
#define BMM350_REG_MAG_X_XLSB                       UINT8_C(0x31)
#define BMM350_REG_MAG_X_LSB                        UINT8_C(0x32)
#define BMM350_REG_MAG_X_MSB                        UINT8_C(0x33)
#define BMM350_REG_MAG_Y_XLSB                       UINT8_C(0x34)
#define BMM350_REG_MAG_Y_LSB                        UINT8_C(0x35)
#define BMM350_REG_MAG_Y_MSB                        UINT8_C(0x36)
#define BMM350_REG_MAG_Z_XLSB                       UINT8_C(0x37)
#define BMM350_REG_MAG_Z_LSB                        UINT8_C(0x38)
#define BMM350_REG_MAG_Z_MSB                        UINT8_C(0x39)
#define BMM350_REG_TEMP_XLSB                        UINT8_C(0x3A)
#define BMM350_REG_TEMP_LSB                         UINT8_C(0x3B)
#define BMM350_REG_TEMP_MSB                         UINT8_C(0x3C)
#define BMM350_REG_SENSORTIME_XLSB                  UINT8_C(0x3D)
#define BMM350_REG_SENSORTIME_LSB                   UINT8_C(0x3E)
#define BMM350_REG_SENSORTIME_MSB                   UINT8_C(0x3F)
#define BMM350_REG_OTP_CMD_REG                      UINT8_C(0x50)
#define BMM350_REG_OTP_DATA_MSB_REG                 UINT8_C(0x52)
#define BMM350_REG_OTP_DATA_LSB_REG                 UINT8_C(0x53)
#define BMM350_REG_OTP_STATUS_REG                   UINT8_C(0x55)
#define BMM350_REG_TMR_SELFTEST_USER                UINT8_C(0x60)
#define BMM350_REG_CTRL_USER                        UINT8_C(0x61)
#define BMM350_REG_CMD                              UINT8_C(0x7E)


/*********************** Macros for bit masking ***************************/

#define BMM350_CHIP_ID_OTP_MSK                      UINT8_C(0xf)
#define BMM350_CHIP_ID_OTP_POS                      UINT8_C(0x0)
#define BMM350_CHIP_ID_FIXED_MSK                    UINT8_C(0xf0)
#define BMM350_CHIP_ID_FIXED_POS                    UINT8_C(0x4)
#define BMM350_REV_ID_MAJOR_MSK                     UINT8_C(0xf0)
#define BMM350_REV_ID_MAJOR_POS                     UINT8_C(0x4)
#define BMM350_REV_ID_MINOR_MSK                     UINT8_C(0xf)
#define BMM350_REV_ID_MINOR_POS                     UINT8_C(0x0)
#define BMM350_PMU_CMD_ERROR_MSK                    UINT8_C(0x1)
#define BMM350_PMU_CMD_ERROR_POS                    UINT8_C(0x0)
#define BMM350_BOOT_UP_ERROR_MSK                    UINT8_C(0x2)
#define BMM350_BOOT_UP_ERROR_POS                    UINT8_C(0x1)
#define BMM350_DRV_MSK                              UINT8_C(0x7)
#define BMM350_DRV_POS                              UINT8_C(0x0)
#define BMM350_AVG_MSK                              UINT8_C(0x30)
#define BMM350_AVG_POS                              UINT8_C(0x4)
#define BMM350_ODR_MSK                              UINT8_C(0xf)
#define BMM350_ODR_POS                              UINT8_C(0x0)
#define BMM350_PMU_CMD_MSK                          UINT8_C(0xf)
#define BMM350_PMU_CMD_POS                          UINT8_C(0x0)
#define BMM350_EN_X_MSK                             UINT8_C(0x01)
#define BMM350_EN_X_POS                             UINT8_C(0x0)
#define BMM350_EN_Y_MSK                             UINT8_C(0x02)
#define BMM350_EN_Y_POS                             UINT8_C(0x1)
#define BMM350_EN_Z_MSK                             UINT8_C(0x04)
#define BMM350_EN_Z_POS                             UINT8_C(0x2)
#define BMM350_EN_XYZ_MSK                           UINT8_C(0x7)
#define BMM350_EN_XYZ_POS                           UINT8_C(0x0)
#define BMM350_PMU_CMD_BUSY_MSK                     UINT8_C(0x1)
#define BMM350_PMU_CMD_BUSY_POS                     UINT8_C(0x0)
#define BMM350_ODR_OVWR_MSK                         UINT8_C(0x2)
#define BMM350_ODR_OVWR_POS                         UINT8_C(0x1)
#define BMM350_AVG_OVWR_MSK                         UINT8_C(0x4)
#define BMM350_AVG_OVWR_POS                         UINT8_C(0x2)
#define BMM350_PWR_MODE_IS_NORMAL_MSK               UINT8_C(0x8)
#define BMM350_PWR_MODE_IS_NORMAL_POS               UINT8_C(0x3)
#define BMM350_CMD_IS_ILLEGAL_MSK                   UINT8_C(0x10)
#define BMM350_CMD_IS_ILLEGAL_POS                   UINT8_C(0x4)
#define BMM350_PMU_CMD_VALUE_MSK                    UINT8_C(0xE0)
#define BMM350_PMU_CMD_VALUE_POS                    UINT8_C(0x5)
#define BMM350_PMU_ODR_S_MSK                        UINT8_C(0xf)
#define BMM350_PMU_ODR_S_POS                        UINT8_C(0x0)
#define BMM350_PMU_AVG_S_MSK                        UINT8_C(0x30)
#define BMM350_PMU_AVG_S_POS                        UINT8_C(0x4)
#define BMM350_I3C_ERROR_0_MSK                      UINT8_C(0x1)
#define BMM350_I3C_ERROR_0_POS                      UINT8_C(0x0)
#define BMM350_I3C_ERROR_3_MSK                      UINT8_C(0x8)
#define BMM350_I3C_ERROR_3_POS                      UINT8_C(0x3)
#define BMM350_I2C_WDT_EN_MSK                       UINT8_C(0x1)
#define BMM350_I2C_WDT_EN_POS                       UINT8_C(0x0)
#define BMM350_I2C_WDT_SEL_MSK                      UINT8_C(0x2)
#define BMM350_I2C_WDT_SEL_POS                      UINT8_C(0x1)
#define BMM350_TRSDCR_REV_ID_OTP_MSK                UINT8_C(0x3)
#define BMM350_TRSDCR_REV_ID_OTP_POS                UINT8_C(0x0)
#define BMM350_TRSDCR_REV_ID_FIXED_MSK              UINT8_C(0xfc)
#define BMM350_TRSDCR_REV_ID_FIXED_POS              UINT8_C(0x2)
#define BMM350_PAGING_EN_MSK                        UINT8_C(0x80)
#define BMM350_PAGING_EN_POS                        UINT8_C(0x7)
#define BMM350_DRDY_DATA_REG_MSK                    UINT8_C(0x4)
#define BMM350_DRDY_DATA_REG_POS                    UINT8_C(0x2)
#define BMM350_INT_MODE_MSK                         UINT8_C(0x1)
#define BMM350_INT_MODE_POS                         UINT8_C(0x0)
#define BMM350_INT_POL_MSK                          UINT8_C(0x2)
#define BMM350_INT_POL_POS                          UINT8_C(0x1)
#define BMM350_INT_OD_MSK                           UINT8_C(0x4)
#define BMM350_INT_OD_POS                           UINT8_C(0x2)
#define BMM350_INT_OUTPUT_EN_MSK                    UINT8_C(0x8)
#define BMM350_INT_OUTPUT_EN_POS                    UINT8_C(0x3)
#define BMM350_DRDY_DATA_REG_EN_MSK                 UINT8_C(0x80)
#define BMM350_DRDY_DATA_REG_EN_POS                 UINT8_C(0x7)
#define BMM350_DRDY_INT_MAP_TO_IBI_MSK              UINT8_C(0x1)
#define BMM350_DRDY_INT_MAP_TO_IBI_POS              UINT8_C(0x0)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_MSK   UINT8_C(0x10)
#define BMM350_CLEAR_DRDY_INT_STATUS_UPON_IBI_POS   UINT8_C(0x4)
#define BMM350_TC_SYNC_TU_MSK                       UINT8_C(0xff)
#define BMM350_TC_SYNC_ODR_MSK                      UINT8_C(0xff)
#define BMM350_TC_SYNC_TPH_1_MSK                    UINT8_C(0xff)
#define BMM350_TC_SYNC_TPH_2_MSK                    UINT8_C(0xff)
#define BMM350_TC_SYNC_DT_MSK                       UINT8_C(0xff)
#define BMM350_TC_SYNC_ST_0_MSK                     UINT8_C(0xff)
#define BMM350_TC_SYNC_ST_1_MSK                     UINT8_C(0xff)
#define BMM350_TC_SYNC_ST_2_MSK                     UINT8_C(0xff)
#define BMM350_CFG_FORCE_SOSC_EN_MSK                UINT8_C(0x4)
#define BMM350_CFG_FORCE_SOSC_EN_POS                UINT8_C(0x2)
#define BMM350_ST_IGEN_EN_MSK                       UINT8_C(0x1)
#define BMM350_ST_IGEN_EN_POS                       UINT8_C(0x0)
#define BMM350_ST_N_MSK                             UINT8_C(0x2)
#define BMM350_ST_N_POS                             UINT8_C(0x1)
#define BMM350_ST_P_MSK                             UINT8_C(0x4)
#define BMM350_ST_P_POS                             UINT8_C(0x2)
#define BMM350_IST_EN_X_MSK                         UINT8_C(0x8)
#define BMM350_IST_EN_X_POS                         UINT8_C(0x3)
#define BMM350_IST_EN_Y_MSK                         UINT8_C(0x10)
#define BMM350_IST_EN_Y_POS                         UINT8_C(0x4)
#define BMM350_CFG_SENS_TIM_AON_MSK                 UINT8_C(0x1)
#define BMM350_CFG_SENS_TIM_AON_POS                 UINT8_C(0x0)
#define BMM350_DATA_X_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_X_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_X_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_X_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_X_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_X_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Y_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Y_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Y_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Y_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Y_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Y_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_Z_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_Z_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_Z_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_Z_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_Z_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_Z_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_T_7_0_MSK                       UINT8_C(0xff)
#define BMM350_DATA_T_7_0_POS                       UINT8_C(0x0)
#define BMM350_DATA_T_15_8_MSK                      UINT8_C(0xff)
#define BMM350_DATA_T_15_8_POS                      UINT8_C(0x0)
#define BMM350_DATA_T_23_16_MSK                     UINT8_C(0xff)
#define BMM350_DATA_T_23_16_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_7_0_MSK                      UINT8_C(0xff)
#define BMM350_DATA_ST_7_0_POS                      UINT8_C(0x0)
#define BMM350_DATA_ST_15_8_MSK                     UINT8_C(0xff)
#define BMM350_DATA_ST_15_8_POS                     UINT8_C(0x0)
#define BMM350_DATA_ST_23_16_MSK                    UINT8_C(0xff)
#define BMM350_DATA_ST_23_16_POS                    UINT8_C(0x0)
#define BMM350_SIGN_INVERT_T_MSK                    UINT8_C(0x10)
#define BMM350_SIGN_INVERT_T_POS                    UINT8_C(0x4)
#define BMM350_SIGN_INVERT_X_MSK                    UINT8_C(0x20)
#define BMM350_SIGN_INVERT_X_POS                    UINT8_C(0x5)
#define BMM350_SIGN_INVERT_Y_MSK                    UINT8_C(0x40)
#define BMM350_SIGN_INVERT_Y_POS                    UINT8_C(0x6)
#define BMM350_SIGN_INVERT_Z_MSK                    UINT8_C(0x80)
#define BMM350_SIGN_INVERT_Z_POS                    UINT8_C(0x7)
#define BMM350_DIS_BR_NM_MSK                        UINT8_C(0x1)
#define BMM350_DIS_BR_NM_POS                        UINT8_C(0x0)
#define BMM350_DIS_FGR_NM_MSK                       UINT8_C(0x2)
#define BMM350_DIS_FGR_NM_POS                       UINT8_C(0x1)
#define BMM350_DIS_CRST_AT_ALL_MSK                  UINT8_C(0x4)
#define BMM350_DIS_CRST_AT_ALL_POS                  UINT8_C(0x2)
#define BMM350_DIS_BR_FM_MSK                        UINT8_C(0x8)
#define BMM350_DIS_BR_FM_POS                        UINT8_C(0x3)
#define BMM350_FRC_EN_BUFF_MSK                      UINT8_C(0x1)
#define BMM350_FRC_EN_BUFF_POS                      UINT8_C(0x0)
#define BMM350_FRC_INA_EN1_MSK                      UINT8_C(0x2)
#define BMM350_FRC_INA_EN1_POS                      UINT8_C(0x1)
#define BMM350_FRC_INA_EN2_MSK                      UINT8_C(0x4)
#define BMM350_FRC_INA_EN2_POS                      UINT8_C(0x2)
#define BMM350_FRC_ADC_EN_MSK                       UINT8_C(0x8)
#define BMM350_FRC_ADC_EN_POS                       UINT8_C(0x3)
#define BMM350_FRC_INA_RST_MSK                      UINT8_C(0x10)
#define BMM350_FRC_INA_RST_POS                      UINT8_C(0x4)
#define BMM350_FRC_ADC_RST_MSK                      UINT8_C(0x20)
#define BMM350_FRC_ADC_RST_POS                      UINT8_C(0x5)
#define BMM350_FRC_INA_XSEL_MSK                     UINT8_C(0x1)
#define BMM350_FRC_INA_XSEL_POS                     UINT8_C(0x0)
#define BMM350_FRC_INA_YSEL_MSK                     UINT8_C(0x2)
#define BMM350_FRC_INA_YSEL_POS                     UINT8_C(0x1)
#define BMM350_FRC_INA_ZSEL_MSK                     UINT8_C(0x4)
#define BMM350_FRC_INA_ZSEL_POS                     UINT8_C(0x2)
#define BMM350_FRC_ADC_TEMP_EN_MSK                  UINT8_C(0x8)
#define BMM350_FRC_ADC_TEMP_EN_POS                  UINT8_C(0x3)
#define BMM350_FRC_TSENS_EN_MSK                     UINT8_C(0x10)
#define BMM350_FRC_TSENS_EN_POS                     UINT8_C(0x4)
#define BMM350_DSENS_FM_MSK                         UINT8_C(0x20)
#define BMM350_DSENS_FM_POS                         UINT8_C(0x5)
#define BMM350_DSENS_SEL_MSK                        UINT8_C(0x40)
#define BMM350_DSENS_SEL_POS                        UINT8_C(0x6)
#define BMM350_DSENS_SHORT_MSK                      UINT8_C(0x80)
#define BMM350_DSENS_SHORT_POS                      UINT8_C(0x7)
#define BMM350_ERR_MISS_BR_DONE_MSK                 UINT8_C(0x1)
#define BMM350_ERR_MISS_BR_DONE_POS                 UINT8_C(0x0)
#define BMM350_ERR_MISS_FGR_DONE_MSK                UINT8_C(0x2)
#define BMM350_ERR_MISS_FGR_DONE_POS                UINT8_C(0x1)
#define BMM350_TST_CHAIN_LN_MODE_MSK                UINT8_C(0x1)
#define BMM350_TST_CHAIN_LN_MODE_POS                UINT8_C(0x0)
#define BMM350_TST_CHAIN_LP_MODE_MSK                UINT8_C(0x2)
#define BMM350_TST_CHAIN_LP_MODE_POS                UINT8_C(0x1)
#define BMM350_EN_OVWR_TMR_IF_MSK                   UINT8_C(0x1)
#define BMM350_EN_OVWR_TMR_IF_POS                   UINT8_C(0x0)
#define BMM350_TMR_CKTRIGB_MSK                      UINT8_C(0x2)
#define BMM350_TMR_CKTRIGB_POS                      UINT8_C(0x1)
#define BMM350_TMR_DO_BR_MSK                        UINT8_C(0x4)
#define BMM350_TMR_DO_BR_POS                        UINT8_C(0x2)
#define BMM350_TMR_DO_FGR_MSK                       UINT8_C(0x18)
#define BMM350_TMR_DO_FGR_POS                       UINT8_C(0x3)
#define BMM350_TMR_EN_OSC_MSK                       UINT8_C(0x80)
#define BMM350_TMR_EN_OSC_POS                       UINT8_C(0x7)
#define BMM350_VCM_TRIM_X_MSK                       UINT8_C(0x1f)
#define BMM350_VCM_TRIM_X_POS                       UINT8_C(0x0)
#define BMM350_VCM_TRIM_Y_MSK                       UINT8_C(0x1f)
#define BMM350_VCM_TRIM_Y_POS                       UINT8_C(0x0)
#define BMM350_VCM_TRIM_Z_MSK                       UINT8_C(0x1f)
#define BMM350_VCM_TRIM_Z_POS                       UINT8_C(0x0)
#define BMM350_VCM_TRIM_DSENS_MSK                   UINT8_C(0x1f)
#define BMM350_VCM_TRIM_DSENS_POS                   UINT8_C(0x0)
#define BMM350_TWLB_MSK                             UINT8_C(0x30)
#define BMM350_TWLB_POS                             UINT8_C(0x4)
#define BMM350_PRG_PLS_TIM_MSK                      UINT8_C(0x30)
#define BMM350_PRG_PLS_TIM_POS                      UINT8_C(0x4)
#define BMM350_OTP_OVWR_EN_MSK                      UINT8_C(0x1)
#define BMM350_OTP_OVWR_EN_POS                      UINT8_C(0x0)
#define BMM350_OTP_MEM_CLK_MSK                      UINT8_C(0x2)
#define BMM350_OTP_MEM_CLK_POS                      UINT8_C(0x1)
#define BMM350_OTP_MEM_CS_MSK                       UINT8_C(0x4)
#define BMM350_OTP_MEM_CS_POS                       UINT8_C(0x2)
#define BMM350_OTP_MEM_PGM_MSK                      UINT8_C(0x8)
#define BMM350_OTP_MEM_PGM_POS                      UINT8_C(0x3)
#define BMM350_OTP_MEM_RE_MSK                       UINT8_C(0x10)
#define BMM350_OTP_MEM_RE_POS                       UINT8_C(0x4)
#define BMM350_SAMPLE_RDATA_PLS_MSK                 UINT8_C(0x80)
#define BMM350_SAMPLE_RDATA_PLS_POS                 UINT8_C(0x7)
#define BMM350_CFG_FW_MSK                           UINT8_C(0x1)
#define BMM350_CFG_FW_POS                           UINT8_C(0x0)
#define BMM350_EN_BR_X_MSK                          UINT8_C(0x2)
#define BMM350_EN_BR_X_POS                          UINT8_C(0x1)
#define BMM350_EN_BR_Y_MSK                          UINT8_C(0x4)
#define BMM350_EN_BR_Y_POS                          UINT8_C(0x2)
#define BMM350_EN_BR_Z_MSK                          UINT8_C(0x8)
#define BMM350_EN_BR_Z_POS                          UINT8_C(0x3)
#define BMM350_CFG_PAUSE_TIME_MSK                   UINT8_C(0x30)
#define BMM350_CFG_PAUSE_TIME_POS                   UINT8_C(0x4)
#define BMM350_CFG_FGR_PLS_DUR_MSK                  UINT8_C(0xf)
#define BMM350_CFG_FGR_PLS_DUR_POS                  UINT8_C(0x0)
#define BMM350_CFG_BR_Z_ORDER_MSK                   UINT8_C(0x10)
#define BMM350_CFG_BR_Z_ORDER_POS                   UINT8_C(0x4)
#define BMM350_CFG_BR_XY_CHOP_MSK                   UINT8_C(0x20)
#define BMM350_CFG_BR_XY_CHOP_POS                   UINT8_C(0x5)
#define BMM350_CFG_BR_PLS_DUR_MSK                   UINT8_C(0xc0)
#define BMM350_CFG_BR_PLS_DUR_POS                   UINT8_C(0x6)
#define BMM350_ENABLE_BR_FGR_TEST_MSK               UINT8_C(0x1)
#define BMM350_ENABLE_BR_FGR_TEST_POS               UINT8_C(0x0)
#define BMM350_SEL_AXIS_MSK                         UINT8_C(0xe)
#define BMM350_SEL_AXIS_POS                         UINT8_C(0x1)
#define BMM350_TMR_CFG_TEST_CLK_EN_MSK              UINT8_C(0x10)
#define BMM350_TMR_CFG_TEST_CLK_EN_POS              UINT8_C(0x4)
#define BMM350_TEST_VAL_BITS_7DOWNTO0_MSK           UINT8_C(0xff)
#define BMM350_TEST_VAL_BITS_7DOWNTO0_POS           UINT8_C(0x0)
#define BMM350_TEST_VAL_BITS_8_MSK                  UINT8_C(0x1)
#define BMM350_TEST_VAL_BITS_8_POS                  UINT8_C(0x0)
#define BMM350_TEST_P_SAMPLE_MSK                    UINT8_C(0x2)
#define BMM350_TEST_P_SAMPLE_POS                    UINT8_C(0x1)
#define BMM350_TEST_N_SAMPLE_MSK                    UINT8_C(0x4)
#define BMM350_TEST_N_SAMPLE_POS                    UINT8_C(0x2)
#define BMM350_TEST_APPLY_TO_REM_MSK                UINT8_C(0x10)
#define BMM350_TEST_APPLY_TO_REM_POS                UINT8_C(0x4)
#define BMM350_UFO_TRM_OSC_RANGE_MSK                UINT8_C(0xf)
#define BMM350_UFO_TRM_OSC_RANGE_POS                UINT8_C(0x0)
#define BMM350_ISO_CHIP_ID_MSK                      UINT8_C(0x78)
#define BMM350_ISO_CHIP_ID_POS                      UINT8_C(0x3)
#define BMM350_ISO_I2C_DEV_ID_MSK                   UINT8_C(0x80)
#define BMM350_ISO_I2C_DEV_ID_POS                   UINT8_C(0x7)
#define BMM350_I3C_FREQ_BITS_1DOWNTO0_MSK           UINT8_C(0xc)
#define BMM350_I3C_FREQ_BITS_1DOWNTO0_POS           UINT8_C(0x2)
#define BMM350_I3C_IBI_MDB_SEL_MSK                  UINT8_C(0x10)
#define BMM350_I3C_IBI_MDB_SEL_POS                  UINT8_C(0x4)
#define BMM350_TC_ASYNC_EN_MSK                      UINT8_C(0x20)
#define BMM350_TC_ASYNC_EN_POS                      UINT8_C(0x5)
#define BMM350_TC_SYNC_EN_MSK                       UINT8_C(0x40)
#define BMM350_TC_SYNC_EN_POS                       UINT8_C(0x6)
#define BMM350_I3C_SCL_GATING_EN_MSK                UINT8_C(0x80)
#define BMM350_I3C_SCL_GATING_EN_POS                UINT8_C(0x7)
#define BMM350_I3C_INACCURACY_BITS_6DOWNTO0_MSK     UINT8_C(0x7f)
#define BMM350_I3C_INACCURACY_BITS_6DOWNTO0_POS     UINT8_C(0x0)
#define BMM350_EST_EN_X_MSK                         UINT8_C(0x1)
#define BMM350_EST_EN_X_POS                         UINT8_C(0x0)
#define BMM350_EST_EN_Y_MSK                         UINT8_C(0x2)
#define BMM350_EST_EN_Y_POS                         UINT8_C(0x1)
#define BMM350_CRST_DIS_MSK                         UINT8_C(0x4)
#define BMM350_CRST_DIS_POS                         UINT8_C(0x2)
#define BMM350_BR_TFALL_MSK                         UINT8_C(0x7)
#define BMM350_BR_TFALL_POS                         UINT8_C(0x0)
#define BMM350_BR_TRISE_MSK                         UINT8_C(0x70)
#define BMM350_BR_TRISE_POS                         UINT8_C(0x4)
#define BMM350_TMR_SOFT_START_DIS_MSK               UINT8_C(0x80)
#define BMM350_TMR_SOFT_START_DIS_POS               UINT8_C(0x7)
#define BMM350_FOSC_LOW_RANGE_MSK                   UINT8_C(0x80)
#define BMM350_FOSC_LOW_RANGE_POS                   UINT8_C(0x7)
#define BMM350_VCRST_TRIM_FG_MSK                    UINT8_C(0x3f)
#define BMM350_VCRST_TRIM_FG_POS                    UINT8_C(0x0)
#define BMM350_VCRST_TRIM_BR_MSK                    UINT8_C(0x3f)
#define BMM350_VCRST_TRIM_BR_POS                    UINT8_C(0x0)
#define BMM350_BG_TRIM_VRP_MSK                      UINT8_C(0xc0)
#define BMM350_BG_TRIM_VRP_POS                      UINT8_C(0x6)
#define BMM350_BG_TRIM_TC_MSK                       UINT8_C(0xf)
#define BMM350_BG_TRIM_TC_POS                       UINT8_C(0x0)
#define BMM350_BG_TRIM_VRA_MSK                      UINT8_C(0xf0)
#define BMM350_BG_TRIM_VRA_POS                      UINT8_C(0x4)
#define BMM350_BG_TRIM_VRD_MSK                      UINT8_C(0xf)
#define BMM350_BG_TRIM_VRD_POS                      UINT8_C(0x0)
#define BMM350_OVWR_REF_IB_EN_MSK                   UINT8_C(0x10)
#define BMM350_OVWR_REF_IB_EN_POS                   UINT8_C(0x4)
#define BMM350_OVWR_VDDA_EN_MSK                     UINT8_C(0x20)
#define BMM350_OVWR_VDDA_EN_POS                     UINT8_C(0x5)
#define BMM350_OVWR_VDDP_EN_MSK                     UINT8_C(0x40)
#define BMM350_OVWR_VDDP_EN_POS                     UINT8_C(0x6)
#define BMM350_OVWR_VDDS_EN_MSK                     UINT8_C(0x80)
#define BMM350_OVWR_VDDS_EN_POS                     UINT8_C(0x7)
#define BMM350_REF_IB_EN_MSK                        UINT8_C(0x10)
#define BMM350_REF_IB_EN_POS                        UINT8_C(0x4)
#define BMM350_VDDA_EN_MSK                          UINT8_C(0x20)
#define BMM350_VDDA_EN_POS                          UINT8_C(0x5)
#define BMM350_VDDP_EN_MSK                          UINT8_C(0x40)
#define BMM350_VDDP_EN_POS                          UINT8_C(0x6)
#define BMM350_VDDS_EN_MSK                          UINT8_C(0x80)
#define BMM350_VDDS_EN_POS                          UINT8_C(0x7)
#define BMM350_OVWR_OTP_PROG_VDD_SW_EN_MSK          UINT8_C(0x8)
#define BMM350_OVWR_OTP_PROG_VDD_SW_EN_POS          UINT8_C(0x3)
#define BMM350_OVWR_EN_MFE_BG_FILT_BYPASS_MSK       UINT8_C(0x10)
#define BMM350_OVWR_EN_MFE_BG_FILT_BYPASS_POS       UINT8_C(0x4)
#define BMM350_OTP_PROG_VDD_SW_EN_MSK               UINT8_C(0x8)
#define BMM350_OTP_PROG_VDD_SW_EN_POS               UINT8_C(0x3)
#define BMM350_CP_COMP_CRST_EN_TM_MSK               UINT8_C(0x10)
#define BMM350_CP_COMP_CRST_EN_TM_POS               UINT8_C(0x4)
#define BMM350_CP_COMP_VDD_EN_TM_MSK                UINT8_C(0x20)
#define BMM350_CP_COMP_VDD_EN_TM_POS                UINT8_C(0x5)
#define BMM350_CP_INTREFS_EN_TM_MSK                 UINT8_C(0x40)
#define BMM350_CP_INTREFS_EN_TM_POS                 UINT8_C(0x6)
#define BMM350_ADC_LOCAL_CHOP_EN_MSK                UINT8_C(0x20)
#define BMM350_ADC_LOCAL_CHOP_EN_POS                UINT8_C(0x5)
#define BMM350_INA_MODE_MSK                         UINT8_C(0x40)
#define BMM350_INA_MODE_POS                         UINT8_C(0x6)
#define BMM350_VDDD_EXT_EN_MSK                      UINT8_C(0x20)
#define BMM350_VDDD_EXT_EN_POS                      UINT8_C(0x5)
#define BMM350_VDDP_EXT_EN_MSK                      UINT8_C(0x80)
#define BMM350_VDDP_EXT_EN_POS                      UINT8_C(0x7)
#define BMM350_ADC_DSENS_EN_MSK                     UINT8_C(0x10)
#define BMM350_ADC_DSENS_EN_POS                     UINT8_C(0x4)
#define BMM350_DSENS_EN_MSK                         UINT8_C(0x20)
#define BMM350_DSENS_EN_POS                         UINT8_C(0x5)
#define BMM350_OTP_TM_CLVWR_EN_MSK                  UINT8_C(0x40)
#define BMM350_OTP_TM_CLVWR_EN_POS                  UINT8_C(0x6)
#define BMM350_OTP_VDDP_DIS_MSK                     UINT8_C(0x80)
#define BMM350_OTP_VDDP_DIS_POS                     UINT8_C(0x7)
#define BMM350_FORCE_HIGH_VREF_IREF_OK_MSK          UINT8_C(0x10)
#define BMM350_FORCE_HIGH_VREF_IREF_OK_POS          UINT8_C(0x4)
#define BMM350_FORCE_HIGH_FOSC_OK_MSK               UINT8_C(0x20)
#define BMM350_FORCE_HIGH_FOSC_OK_POS               UINT8_C(0x5)
#define BMM350_FORCE_HIGH_MFE_BG_RDY_MSK            UINT8_C(0x40)
#define BMM350_FORCE_HIGH_MFE_BG_RDY_POS            UINT8_C(0x6)
#define BMM350_FORCE_HIGH_MFE_VTMR_RDY_MSK          UINT8_C(0x80)
#define BMM350_FORCE_HIGH_MFE_VTMR_RDY_POS          UINT8_C(0x7)
#define BMM350_ERR_END_OF_RECHARGE_MSK              UINT8_C(0x1)
#define BMM350_ERR_END_OF_RECHARGE_POS              UINT8_C(0x0)
#define BMM350_ERR_END_OF_DISCHARGE_MSK             UINT8_C(0x2)
#define BMM350_ERR_END_OF_DISCHARGE_POS             UINT8_C(0x1)
#define BMM350_CP_TMX_DIGTP_SEL_MSK                 UINT8_C(0x7)
#define BMM350_CP_TMX_DIGTP_SEL_POS                 UINT8_C(0x0)
#define BMM350_CP_CPOSC_EN_TM_MSK                   UINT8_C(0x80)
#define BMM350_CP_CPOSC_EN_TM_POS                   UINT8_C(0x7)
#define BMM350_TST_ATM1_CFG_MSK                     UINT8_C(0x3f)
#define BMM350_TST_ATM1_CFG_POS                     UINT8_C(0x0)
#define BMM350_TST_TB1_EN_MSK                       UINT8_C(0x80)
#define BMM350_TST_TB1_EN_POS                       UINT8_C(0x7)
#define BMM350_TST_ATM2_CFG_MSK                     UINT8_C(0x1f)
#define BMM350_TST_ATM2_CFG_POS                     UINT8_C(0x0)
#define BMM350_TST_TB2_EN_MSK                       UINT8_C(0x80)
#define BMM350_TST_TB2_EN_POS                       UINT8_C(0x7)
#define BMM350_REG_DTB1X_SEL_MSK                    UINT8_C(0x7f)
#define BMM350_REG_DTB1X_SEL_POS                    UINT8_C(0x0)
#define BMM350_SEL_DTB1X_PAD_MSK                    UINT8_C(0x80)
#define BMM350_SEL_DTB1X_PAD_POS                    UINT8_C(0x7)
#define BMM350_REG_DTB2X_SEL_MSK                    UINT8_C(0x7f)
#define BMM350_REG_DTB2X_SEL_POS                    UINT8_C(0x0)
#define BMM350_TMR_TST_CFG_MSK                      UINT8_C(0x7f)
#define BMM350_TMR_TST_CFG_POS                      UINT8_C(0x0)
#define BMM350_TMR_TST_HIZ_VTMR_MSK                 UINT8_C(0x80)
#define BMM350_TMR_TST_HIZ_VTMR_POS                 UINT8_C(0x7)

/****************************** OTP MACROS ***************************/
#define BMM350_OTP_CMD_DIR_READ                     UINT8_C(0x20)
#define BMM350_OTP_CMD_DIR_PRGM_1B                  UINT8_C(0x40)
#define BMM350_OTP_CMD_DIR_PRGM                     UINT8_C(0x60)
#define BMM350_OTP_CMD_PWR_OFF_OTP                  UINT8_C(0x80)
#define BMM350_OTP_CMD_EXT_READ                     UINT8_C(0xA0)
#define BMM350_OTP_CMD_EXT_PRGM                     UINT8_C(0xE0)
#define BMM350_OTP_CMD_MSK                          UINT8_C(0xE0)
#define BMM350_OTP_WORD_ADDR_MSK                    UINT8_C(0x1F)

#define BMM350_OTP_STATUS_ERROR_MSK                 UINT8_C(0xE0)
#define BMM350_OTP_STATUS_ERROR(val)                (val & BMM350_OTP_STATUS_ERROR_MSK)
#define BMM350_OTP_STATUS_NO_ERROR                  UINT8_C(0x00)
#define BMM350_OTP_STATUS_BOOT_ERR                  UINT8_C(0x20)
#define BMM350_OTP_STATUS_PAGE_RD_ERR               UINT8_C(0x40)
#define BMM350_OTP_STATUS_PAGE_PRG_ERR              UINT8_C(0x60)
#define BMM350_OTP_STATUS_SIGN_ERR                  UINT8_C(0x80)
#define BMM350_OTP_STATUS_INV_CMD_ERR               UINT8_C(0xA0)
#define BMM350_OTP_STATUS_CMD_DONE                  UINT8_C(0x01)

/****************************** OTP indices ***************************/
#define BMM350_TEMP_OFF_SENS                        UINT8_C(0x0D)

#define BMM350_MAG_OFFSET_X                         UINT8_C(0x0E)
#define BMM350_MAG_OFFSET_Y                         UINT8_C(0x0F)
#define BMM350_MAG_OFFSET_Z                         UINT8_C(0x10)

#define BMM350_MAG_SENS_X                           UINT8_C(0x10)
#define BMM350_MAG_SENS_Y                           UINT8_C(0x11)
#define BMM350_MAG_SENS_Z                           UINT8_C(0x11)

#define BMM350_MAG_TCO_X                            UINT8_C(0x12)
#define BMM350_MAG_TCO_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCO_Z                            UINT8_C(0x14)

#define BMM350_MAG_TCS_X                            UINT8_C(0x12)
#define BMM350_MAG_TCS_Y                            UINT8_C(0x13)
#define BMM350_MAG_TCS_Z                            UINT8_C(0x14)

#define BMM350_MAG_DUT_T_0                          UINT8_C(0x18)

#define BMM350_CROSS_X_Y                            UINT8_C(0x15)
#define BMM350_CROSS_Y_X                            UINT8_C(0x15)
#define BMM350_CROSS_Z_X                            UINT8_C(0x16)
#define BMM350_CROSS_Z_Y                            UINT8_C(0x16)

#define BMM350_SENS_CORR_Y                          (0.01f)
#define BMM350_TCS_CORR_Z                           (0.0001f)

/**************************** Signed bit macros **********************/
#define BMM350_SIGNED_8_BIT                         UINT8_C(8)
#define BMM350_SIGNED_12_BIT                        UINT8_C(12)
#define BMM350_SIGNED_16_BIT                        UINT8_C(16)
#define BMM350_SIGNED_21_BIT                        UINT8_C(21)
#define BMM350_SIGNED_24_BIT                        UINT8_C(24)


/**************************** PMU command status 0 macros **********************/
#define BMM350_PMU_CMD_STATUS_0_SUS                 UINT8_C(0x00)
#define BMM350_PMU_CMD_STATUS_0_NM                  UINT8_C(0x01)
#define BMM350_PMU_CMD_STATUS_0_UPD_OAE             UINT8_C(0x02)
#define BMM350_PMU_CMD_STATUS_0_FM                  UINT8_C(0x03)
#define BMM350_PMU_CMD_STATUS_0_FM_FAST             UINT8_C(0x04)
#define BMM350_PMU_CMD_STATUS_0_FGR                 UINT8_C(0x05)
#define BMM350_PMU_CMD_STATUS_0_FGR_FAST            UINT8_C(0x06)
#define BMM350_PMU_CMD_STATUS_0_BR                  UINT8_C(0x07)
#define BMM350_PMU_CMD_STATUS_0_BR_FAST             UINT8_C(0x07)

typedef struct bmm350_dev_s bmm350_dev_s;
/*************************************************************
 * ********************* Enumerations*******************************
 ************************************************************/

typedef enum{
	BMM35_E_NULL,
	BMM350_OK,
	BMM350_E_NULL_PTR,
	BMM350_E_PMU_CMD_S_NULL_PTR,
	Bmm350_DEV_NOT_READY,
	BMM350_E_W_COM_FAIL,  // Write communication fail
	BMM350_E_R_COM_FAIL,  // Read communication fail
	BMM350_E_DEV_NOT_FOUND,
	BMM350_E_INVALID_CONFIG, // If user didn't use an internal function then there is a bug in configuration error * refer to library maker or chip documentations *
	BMM350_E_BAD_PAD_DRIVE,
	BMM350_E_RESET_UNFINISHED,
	BMM350_E_INVALID_INPUT,
	BMM350_E_SELF_TEST_INVALID_AXIS,
	BMM350_E_OTP_BOOT,
	BMM350_E_OTP_PAGE_RD,
	BMM350_E_OTP_PAGE_PRG,
	BMM350_E_OTP_SIGN,
	BMM350_E_OTP_INV_CMD,
	BMM350_E_OTP_UNDEFINED,
	BMM350_E_ALL_AXIS_DISABLED,
	BMM350_E_PMU_CMD_VALUE,
	BMM350_E_INV_PWR_MODE,
	BMM350_E_INV_ODR_AVG,
	BMM350_E_SUSPEND_MODE,
	BMM350_E_INTR_INV,
	BMM350_INT_DATA_READY,
	BMM350_INT_DATA_NOT_READY

} bmm350_status_e;


typedef enum{
	BMM350_DISABLE,
	BMM350_ENABLE
} bmm350_enable_state_e;


typedef enum {
    BMM350_SUSPEND_MODE = BMM350_PMU_CMD_SUS,
    BMM350_NORMAL_MODE = BMM350_PMU_CMD_NM,
    BMM350_FORCED_MODE = BMM350_PMU_CMD_FM,
    BMM350_FORCED_MODE_FAST = BMM350_PMU_CMD_FM_FAST
} bmm350_power_modes_e;

typedef enum {
    BMM350_DATA_RATE_400HZ    = BMM350_ODR_400HZ,
    BMM350_DATA_RATE_200HZ    = BMM350_ODR_200HZ,
    BMM350_DATA_RATE_100HZ    = BMM350_ODR_100HZ,
    BMM350_DATA_RATE_50HZ     = BMM350_ODR_50HZ,
    BMM350_DATA_RATE_25HZ     = BMM350_ODR_25HZ,
    BMM350_DATA_RATE_12_5HZ   = BMM350_ODR_12_5HZ,
    BMM350_DATA_RATE_6_25HZ   = BMM350_ODR_6_25HZ,
    BMM350_DATA_RATE_3_125HZ  = BMM350_ODR_3_125HZ,
    BMM350_DATA_RATE_1_5625HZ = BMM350_ODR_1_5625HZ
} bmm350_data_rates_e;

typedef enum  {
    BMM350_UNMAP_FROM_PIN = BMM350_DISABLE,
    BMM350_MAP_TO_PIN = BMM350_ENABLE
} bmm350_intr_map;
typedef enum  {
    BMM350_PULSED,
    BMM350_LATCHED
} bmm350_intr_latch;

typedef enum  {
    BMM350_ACTIVE_LOW,
    BMM350_ACTIVE_HIGH
} bmm350_intr_polarity;


typedef enum  {
    BMM350_INTR_OPEN_DRAIN,
    BMM350_INTR_PUSH_PULL
} bmm350_intr_drive;

/*!
 * @brief bmm350 angle uint and range
 */
typedef enum{
	NON,
	RADIAN,
	DEGREE,
	FROM_0_TO_360,    		// Angle range in degrees
	FROM_NEG_180_TO_POS_180	// Angle range in degrees
} bmm350_ang_unt_rng_e;


/*************************************************************
 *********************** Function Pointers *******************
 ************************************************************/


typedef bmm350_status_e (*BMM350_read_fptr)(bmm350_dev_s *dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size);
typedef bmm350_status_e (*BMM350_write_fptr)(bmm350_dev_s *dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size);
typedef void (*delay_us_) (uint32_t us);



/*************************************************************
 * ********************* Structure Definitions****************
 ************************************************************/

/*!
 * @brief bmm350 un-compensated (raw) magnetometer data, signed 32-bit integer
 */
typedef struct{
    /*! Raw mag X data */
    int32_t raw_xdata;

    /*! Raw mag Y data */
    int32_t raw_ydata;

    /*! Raw mag Z data */
    int32_t raw_zdata;

    /*! Raw mag temperature value */
    int32_t raw_data_t;
} bmm350_raw_mag_data_s;


/*!
 * @brief bmm350 compensated  magnetometer data in Microtesla (µT) and Temperature in degrees Celsius (°C)
 */
typedef struct{
    /*! Compensated mag X data */
    float x;
    /*! Compensated mag Y data */
    float y;
    /*! Compensated mag Z data */
    float z;
    /*! Temperature */
    float temperature;
} bmm350_mag_temp_data_s;

/*!
 * @brief bmm350 magnetometer offset compensation coefficients for the magnetic axes (X, Y, Z)
 */
typedef struct{
    /*! Temperature offset */
    float t_offs;
    /*! Offset x-axis */
    float offset_x;
    /*! Offset y-axis */
    float offset_y;
    /*! Offset z-axis */
    float offset_z;
} bmm350_dut_offset_coef_s;

/*!
 * @brief bmm350 magnetometer sensitivity coefficient structure
 */
typedef struct{
    /*! Temperature sensitivity */
    float t_sens;
    /*! Sensitivity x-axis */
    float sens_x;
    /*! Sensitivity y-axis */
    float sens_y;
    /*! Sensitivity z-axis */
    float sens_z;
} bmm350_dut_sensit_coef_s;

/*!
 * @brief bmm350 temperature coefficient of offset (TCO).
 * Magnetic offset error changes with temperature.
 * This structure stores the coefficients used to correct that drift.
 */
typedef struct{
    float tco_x;
    float tco_y;
    float tco_z;
} bmm350_dut_tco_s;

/*!
 * @brief bmm350 temperature coefficient of sensitivity (TCS).
 * the sensitivity also drifts with temperature. This structure compensates for that.
 */
typedef struct{
    float tcs_x;
    float tcs_y;
    float tcs_z;
} bmm350_dut_tcs_s;

/*!
 * @brief bmm350 cross-axis sensitivity correction factors.
 * Magnetic sensors are not perfectly aligned to orthogonal axes. Cross-axis terms correct for
 * the effect of magnetic field along one axis influencing another axis’s output.
 */
typedef struct{
    float cross_x_y;
    float cross_y_x;
    float cross_z_x;
    float cross_z_y;
} bmm350_cross_axis_s;

/*!
 * @brief bmm350 interrupt pin (INT) control parameters for register 0x2E
 */
typedef struct{
	bmm350_intr_latch 		latch;

	bmm350_intr_drive 		drive;

	bmm350_intr_map 		map;

	bmm350_intr_polarity	polarity;

	bmm350_enable_state_e 	data_ready_interrupt;

} bmm350_intr_s;


/*!
 * @brief bmm350 compensation parameters for calculation
 */
typedef struct{
    /*! Structure to store dut offset coefficient */
    bmm350_dut_offset_coef_s dut_offset_coef;
    /*! Structure to store dut sensitivity coefficient */
    bmm350_dut_sensit_coef_s dut_sensit_coef;
    /*! Structure to store dut tco */
    bmm350_dut_tco_s dut_tco;
    /*! Structure to store dut tcs */
    bmm350_dut_tcs_s dut_tcs;
    /*! Initialize T0_reading parameter */
    float dut_t0;
    /*! Structure to define cross axis compensation */
    bmm350_cross_axis_s cross_axis;
} bmm350_mag_compensate_s;


/*!
 * @brief bmm350 structure that holds the contents of register 0x07
 */
typedef struct{
    /*! The previous PMU CMD is still in processing */
    uint8_t pmu_cmd_busy;
    /*! The previous PMU_CMD_AGGR_SET.odr has been overwritten */
    uint8_t odr_ovwr;
    /*! The previous PMU_CMD_AGGR_SET.avg has been overwritten */
    uint8_t avr_ovwr;
    /*! The chip is in normal power mode */
    uint8_t pwr_mode_is_normal;
    /*! CMD value is not allowed */
    uint8_t cmd_is_illegal;
    /*! Stores the latest PMU_CMD code processed */
    uint8_t pmu_cmd_value;
} bmm350_pmu_cmd_status_0_s;

/*!
 * @brief bmm350 structure that holds the contents of register 0x07
 */
typedef struct{
    /*! The actual effective PMU_CMD_AGGR_SET.avg value */
    uint8_t pmu_avg_s;
    /*! The actual effective PMU_CMD_AGGR_SET.odr value */
    uint8_t pmu_odr_s;

} bmm350_pmu_cmd_status_1_s;

/*!
 * @brief bmm350 structure that holds the contents of register 0x08
 */
typedef struct{
    /*! The actual effective PMU_CMD_AGGR_SET.avg value */
	bmm350_pmu_cmd_status_0_s status_0;
    /*! The actual effective PMU_CMD_AGGR_SET.odr value */
	bmm350_pmu_cmd_status_1_s status_1;

} bmm350_pmu_cmd_status_s;

/*!
 * @brief bmm350 device main structure
 */
struct bmm350_dev_s{

	/*! hi2c port */
	I2C_HandleTypeDef *hi2c;

	/*! I2C Address port */
	uint16_t address;

	/*! Chip Id of BMM350 */
	uint8_t chip_id;

	/*! Bus read function pointer */
	BMM350_read_fptr read;

	/*! Bus write function pointer */
	BMM350_write_fptr write;

	/*! delay(in us) function pointer */
	delay_us_ delay_us;

    /*! Variable to store status of axes enabled */
    uint8_t axis_en;

    /*! Structure for mag compensate */
    bmm350_mag_compensate_s mag_comp;

    /*! Array to store OTP data */
    uint16_t otp_data[BMM350_OTP_DATA_LENGTH];

    /*! Variant ID */
    uint8_t var_id;

    /*! error buffer */
    bmm350_status_e bmm350_err;

    /*! structure holds both PMU_CMD_STATUS_1 and PMU_CMD_STATUS_0 values */
    bmm350_pmu_cmd_status_s pmu_cmd_status;

    /*! structure holds X, Y, Z and Temperature data */
    bmm350_mag_temp_data_s data;

    /*! structure holds configuration of interrupts for INT_STATUS register and INT pin */
    bmm350_intr_s intr_setting;

    float yaw[2]; // [0] raw_yaw, [1] filtered yaw

} ;

struct bmm350_dev_s BMM350;
KalmanFilter_t BMM350_yaw_kf;

/*************************************************************
 *********************** Functions declaration****************
 ************************************************************/


/************************ USER FUNCTIONS ***********************/
bmm350_status_e BMM350_init(struct bmm350_dev_s *de, I2C_HandleTypeDef *hi2c);
bmm350_status_e readout_data(struct bmm350_dev_s *dev, bmm350_mag_temp_data_s *output_data);
bmm350_status_e read_intr(struct bmm350_dev_s *dev, bmm350_mag_temp_data_s *output_data);
float 			BMM350_read_yaw(struct bmm350_dev_s *dev, bmm350_ang_unt_rng_e unit, bmm350_ang_unt_rng_e range);

/************************ USER & INTERNAL FUNCTIONS ***********************/

bmm350_status_e check_device(struct bmm350_dev_s *dev);
bmm350_status_e BMM350_I2C_read(bmm350_dev_s *dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size);
bmm350_status_e BMM350_I2C_write(bmm350_dev_s *dev, uint16_t MemAddress, uint8_t *pData, uint16_t Size);
bmm350_status_e bmm350_get_compensated_mag_xyz_temp_data(struct bmm350_dev_s *dev, bmm350_mag_temp_data_s *mag_temp_data);
bmm350_status_e bmm350_get_pmu_cmd_status(struct bmm350_dev_s *dev, bmm350_pmu_cmd_status_s *pmu_cmd_stat);
bmm350_status_e bmm350_set_powermode(struct bmm350_dev_s *dev, bmm350_power_modes_e powermode);

/************************ INTERNAL FUNCTIONS ***********************/
bmm350_status_e BMM350_init_struct(struct bmm350_dev_s *dev, I2C_HandleTypeDef *hi2c);
bmm350_status_e interrupt_config(struct bmm350_dev_s *dev);
bmm350_status_e bmm350_read_uncomp_mag_temp_data(struct bmm350_dev_s *dev, bmm350_raw_mag_data_s *raw_data);
bmm350_status_e read_out_raw_data(struct bmm350_dev_s *dev, float *out_data);
bmm350_status_e bmm350_odr_avg_config(struct bmm350_dev_s *dev, uint16_t odr, uint16_t avg);
bmm350_status_e set_powermode(struct bmm350_dev_s *dev, bmm350_power_modes_e powermode);
bmm350_status_e read_otp_word(struct bmm350_dev_s *dev, uint8_t addr, uint16_t *lsb_msb);
bmm350_status_e bmm350_enable_interrupt(struct bmm350_dev_s *dev, bmm350_enable_state_e status);
bmm350_status_e bmm350_magnetic_reset_and_wait(struct bmm350_dev_s *dev);
bmm350_status_e otp_dump_after_boot(struct bmm350_dev_s *dev);
bmm350_status_e null_ptr_check(struct bmm350_dev_s *dev);
int32_t fix_sign(uint32_t inval, int8_t number_of_bits);
bmm350_status_e OTP_E_handler(struct bmm350_dev_s *dev, uint8_t otp_err);
void update_default_coefiecents(float *lsb_to_ut_degC);
void update_mag_off_sens(struct bmm350_dev_s *dev);

#endif /* SRC_BMM350_IMU_BMM350_H_ */
