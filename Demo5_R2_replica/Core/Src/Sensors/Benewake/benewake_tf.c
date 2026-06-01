/*
 * benewake_tf.c
 *
 *  Created on: Mar 15, 2026
 *      Author: Ibrahim Meselhy
 */


#include "benewake_tf.h"
#include <string.h>

TF_Data_t TF02_data;
TF_Data_t NOVA_data;
TF_Data_t TF_F_data;
TF_Data_t TF_B_data;
TF_Data_t TF_R_data;
TF_Data_t TF_L_data;
TF_Handle_t TF02;
TF_Handle_t TF_NOVA;
TF_Handle_t TF_NOVA3;
TF_Handle_t TF_F;
TF_Handle_t TF_B;
TF_Handle_t TF_R;
TF_Handle_t TF_L;
TF_Bus_t i2c5_bus = {0};


/* ============================================================================
 * Private constants
 * ========================================================================== */

#define TF_CMD_TIMEOUT_MS       150U    /* Max wait for a command response      */
#define TF_I2C_TIMEOUT_MS       50U     /* HAL I2C blocking timeout             */
#define TF_MAX_CMD_LEN          16U     /* Maximum command frame byte count     */
#define TF_MAX_RSP_LEN          20U     /* Maximum response frame byte count    */

/* IIC register addresses (TF-NOVA) */
#define TFNOVA_REG_DIST_LOW     0x00U
#define TFNOVA_REG_SAVE         0x20U
#define TFNOVA_REG_REBOOT       0x21U
#define TFNOVA_REG_SLAVE_ADDR   0x22U
#define TFNOVA_REG_ENABLE       0x25U
#define TFNOVA_REG_FPS_LOW      0x26U
#define TFNOVA_REG_IF_PROTOCOL  0x1EU
#define TFNOVA_REG_RESTORE      0x29U
#define TFNOVA_REG_MIN_DIST_L   0x2EU
#define TFNOVA_REG_MAX_DIST_L   0x30U
#define TFNOVA_REG_ONOFF_DIST_L 0x32U
#define TFNOVA_REG_ONOFF_EN     0x3AU

/* Command IDs (shared UART protocol) */
#define CMD_GET_VERSION         0x01U
#define CMD_SOFT_RESET          0x02U
#define CMD_SAMPLE_FREQ         0x03U
#define CMD_TRIGGER             0x04U   /* TF02-Pro only */
#define CMD_OUTPUT_FORMAT       0x05U
#define CMD_BAUD_RATE           0x06U
#define CMD_OUTPUT_EN           0x07U
#define CMD_IF_PROTOCOL         0x0AU
#define CMD_IIC_SLAVE_ADDR      0x0BU
#define CMD_RESTORE_DEFAULT     0x10U
#define CMD_SAVE_SETTINGS       0x11U
#define CMD_DIST_RANGE          0x3AU   /* TF-NOVA only  */
#define CMD_ON_OFF_MODE         0x3BU

/* Output format codes */
#define FMT_9BYTE_CM            0x01U
#define FMT_9BYTE_MM            0x06U

#define TF_BUS_TRANSFER_TIMEOUT_MS   10U

/* ============================================================================
 * Private helper prototypes
 * ========================================================================== */

static uint8_t 	   calc_checksum(const uint8_t *buf, uint16_t len);

static TF_Status_t uart_send_cmd(TF_Handle_t *htf,
		const uint8_t *cmd, uint8_t cmd_len,
		uint8_t *rsp, uint8_t rsp_len);

static TF_Status_t i2c_write_regs(TF_Handle_t *htf, uint8_t reg,
		const uint8_t *data, uint16_t len);

static TF_Status_t i2c_read_regs(TF_Handle_t *htf, uint8_t reg,
		uint8_t *data, uint16_t len);

static TF_Status_t i2c_kick_tx(TF_Handle_t *htf);   /* arm async TX → starts pipeline */

static TF_Status_t parse_data_frame(TF_Handle_t *htf,
		const uint8_t *frame, TF_Data_t *data);

static bool       ring_buf_get(TF_Handle_t *htf, uint8_t *byte_out);

static uint16_t   ring_buf_available(const TF_Handle_t *htf);


TF_Status_t TF_Init_UART(TF_Handle_t *htf, TF_SensorType_t type,
		UART_HandleTypeDef *huart)
{
	if (!htf || !huart) return TF_ERR_PARAM;

	memset(htf, 0, sizeof(*htf));
	htf->type        = type;
	htf->interface   = TF_IF_UART;
	htf->unit        = TF_UNIT_CM;
	htf->huart       = huart;
	htf->i2c_addr    = TF_DEFAULT_I2C_ADDR;
	htf->initialised = true;

	/* Kick off continuous single-byte IT reception */
	HAL_StatusTypeDef hal_st = HAL_UART_Receive_IT(huart, &htf->rx_byte, 1U);
	if (hal_st != HAL_OK) {
		htf->initialised = false;
		return TF_ERR_HAL;
	}
	return TF_OK;
}

TF_Status_t TF_Init_I2C(TF_Handle_t *htf, TF_SensorType_t type,
		I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
	if (!htf || !hi2c) return TF_ERR_PARAM;
	if (i2c_addr < 0x08U || i2c_addr > 0x77U) return TF_ERR_PARAM;

	memset(htf, 0, sizeof(*htf));
	htf->type        = type;
	htf->interface   = TF_IF_I2C;
	htf->unit        = TF_UNIT_CM;
	htf->hi2c        = hi2c;
	htf->i2c_addr    = i2c_addr;
	htf->i2c_mode    = TF_I2C_BLOCKING;
	htf->dma_state   = TF_DMA_STATE_IDLE;
	htf->initialised = true;

	return TF_OK;
}

TF_Status_t TF_Init_I2C_Async(TF_Handle_t *htf,
		I2C_HandleTypeDef *hi2c, uint8_t i2c_addr,
		TF_I2C_Mode_t mode)
{
	if (!htf || !hi2c) return TF_ERR_PARAM;
	if (i2c_addr < 0x08U || i2c_addr > 0x77U) return TF_ERR_PARAM;

	memset(htf, 0, sizeof(*htf));
	htf->type        = TF_SENSOR_TFNOVA;   /* async path is TF-NOVA only     */
	htf->interface   = TF_IF_I2C;
	htf->unit        = TF_UNIT_CM;
	htf->hi2c        = hi2c;
	htf->i2c_addr    = i2c_addr;
	htf->i2c_mode    = mode;
	htf->dma_state   = TF_DMA_STATE_IDLE;
	/* Pre-load the register address byte — it never changes */
	htf->dma_reg_addr = TFNOVA_REG_DIST_LOW;
	htf->initialised = true;

	//
	//    /* Arm the very first TX immediately so the pipeline is live from the start */
	//    if (mode != TF_I2C_BLOCKING) {
	//        TF_Status_t st = i2c_kick_tx(htf);
	//        if (st != TF_OK) {
	//            htf->initialised = false;
	//            return st;
	//        }
	//    }
	return TF_OK;
}


void TF_RegisterCallback(TF_Handle_t *htf,
		void (*cb)(const TF_Data_t *data, void *ctx),
		void *user_ctx)
{
	if (!htf) return;
	htf->on_data  = cb;
	htf->user_ctx = user_ctx;
}


void TF_UART_RxCpltCallback(TF_Handle_t *htf, UART_HandleTypeDef *huart)
{
	if (!htf || !huart) return;
	if (htf->interface != TF_IF_UART) return;
	if (htf->huart != huart) return;

	/* Write received byte into ring buffer */
	uint16_t next_head = (htf->rx_head + 1U) & (TF_UART_RX_BUF_SIZE - 1U);
	if (next_head != htf->rx_tail) {               /* drop if full            */
		htf->rx_buf[htf->rx_head] = htf->rx_byte;
		htf->rx_head = next_head;
	}

	/* Restart single-byte IT reception */
	HAL_UART_Receive_IT(huart, &htf->rx_byte, 1U);
}

TF_Status_t TF_I2C_TxCpltCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c)
{
	if (!htf || !hi2c) return TF_ERR_PARAM;
	if (htf->hi2c != hi2c) return TF_ERR_PARAM;
	if (htf->interface != TF_IF_I2C) return TF_ERR_UNSUPPORTED;
	if (htf->i2c_mode == TF_I2C_BLOCKING) return TF_ERR_UNSUPPORTED;
	if (!htf->i2c_transfer_active)           return TF_ERR_TIMEOUT;  // not this sensor's transfer
	if (htf->dma_state != TF_DMA_STATE_TX_PEND) return TF_ERR_TIMEOUT;

	/* Register address sent — now kick RX for the 8 data bytes */
	htf->dma_state = TF_DMA_STATE_RX_PEND;
	uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
	HAL_StatusTypeDef hs;

	if (htf->i2c_mode == TF_I2C_DMA) {
		hs = HAL_I2C_Master_Receive_DMA(htf->hi2c, dev_addr,
				htf->dma_rx_buf, 8U);
	} else {
		hs = HAL_I2C_Master_Receive_IT(htf->hi2c, dev_addr,
				htf->dma_rx_buf, 8U);
	}

	if (hs != HAL_OK) {
		htf->i2c_transfer_active = false;
		htf->dma_state = TF_DMA_STATE_ERROR;
		return TF_ERR_HAL;
	}
	return TF_OK;
}

void TF_I2C_RxCpltCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c)
{
	if (!htf || !hi2c) return;
	if (htf->hi2c != hi2c) return;
	if (htf->interface != TF_IF_I2C) return;
	if (htf->i2c_mode == TF_I2C_BLOCKING) return;
	if (!htf->i2c_transfer_active)            return;  // not this sensor's transfer
	if (htf->dma_state != TF_DMA_STATE_RX_PEND) return;

	/* 8 bytes received and sitting in dma_rx_buf — signal TF_ReadData() */
	htf->i2c_transfer_active = false;  // release the bus
	htf->dma_state = TF_DMA_STATE_DONE;
}

void TF_I2C_ErrorCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c)
{
	if (!htf || !hi2c) return;
	if (htf->hi2c != hi2c) return;
	if (htf->interface != TF_IF_I2C) return;
	if (htf->i2c_mode == TF_I2C_BLOCKING) return;

	htf->i2c_transfer_active = false;
	htf->dma_state = TF_DMA_STATE_ERROR;
}

TF_Status_t TF_ReadData(TF_Handle_t *htf, TF_Data_t *data)
{
	if (!htf || !data)       return TF_ERR_PARAM;
	if (!htf->initialised)   return TF_ERR_NOT_INIT;

	/* ---- I2C path ---- */
	if (htf->interface == TF_IF_I2C) {
		if (htf->type == TF_SENSOR_TFNOVA) {

			/* ---- TF-NOVA async (DMA or IT) -------------------------------- */
			if (htf->i2c_mode != TF_I2C_BLOCKING) {

				switch (htf->dma_state) {

				case TF_DMA_STATE_IDLE:
					/* Pipeline stalled (first call, or after an error recovery).
					 * Kick the TX stage and tell the caller to come back later. */
					i2c_kick_tx(htf);
					return TF_ERR_NO_DATA;

				case TF_DMA_STATE_TX_PEND:
				case TF_DMA_STATE_RX_PEND:
					/* Transfer still in flight — data not ready yet */
					return TF_ERR_NO_DATA;

				case TF_DMA_STATE_ERROR:
					/* Wait for bus to be ready before re-arming —
					 * prevents hammering the bus on persistent AF errors */
					if (HAL_I2C_GetState(htf->hi2c) == HAL_I2C_STATE_READY) {
						htf->dma_state = TF_DMA_STATE_IDLE;
						i2c_kick_tx(htf);
					}
					return TF_ERR_HAL;

				case TF_DMA_STATE_DONE: {
					/* Fresh data sitting in dma_rx_buf — parse it */
					uint8_t frame[TF_UART_FRAME_LEN];
					frame[0] = TF_FRAME_HEADER;
					frame[1] = TF_FRAME_HEADER;
					frame[2] = htf->dma_rx_buf[0];  /* DIST_L  */
					frame[3] = htf->dma_rx_buf[1];  /* DIST_H  */
					frame[4] = htf->dma_rx_buf[2];  /* PEAK_L  */
					frame[5] = htf->dma_rx_buf[3];  /* PEAK_H  */
					frame[6] = htf->dma_rx_buf[4];  /* TEMP_L  */
					frame[7] = htf->dma_rx_buf[5];	/* TEMP_H*/
					frame[8] = calc_checksum(frame, 8U);

					/* Re-arm the next transfer BEFORE parsing so we never
					 * idle — the I2C bus is immediately re-used */
					htf->dma_state = TF_DMA_STATE_IDLE;
					i2c_kick_tx(htf);

					TF_Status_t st = parse_data_frame(htf, frame, data);
					if (st == TF_OK) {
						htf->last_dma_data = *data;
						if (htf->on_data) htf->on_data(data, htf->user_ctx);
					}
					return st;
				}

				default:
					return TF_ERR_HAL;
				}
			}

			/* ---- TF-NOVA blocking ------------------------------ */
			{
				uint8_t raw[8];
				TF_Status_t st = i2c_read_regs(htf, TFNOVA_REG_DIST_LOW, raw, 8U);
				if (st != TF_OK) return st;

				uint8_t frame[TF_UART_FRAME_LEN];
				frame[0] = TF_FRAME_HEADER;
				frame[1] = TF_FRAME_HEADER;
				frame[2] = raw[0];
				frame[3] = raw[1];
				frame[4] = raw[2];
				frame[5] = raw[3];
				frame[6] = raw[4];
				frame[7] = 0U;
				frame[8] = calc_checksum(frame, 8U);
				return parse_data_frame(htf, frame, data);
			}

		} else {
			/* TF02-Pro: use "Obtain data frame" command over I2C */
			uint8_t fmt_byte = (htf->unit == TF_UNIT_MM) ? FMT_9BYTE_MM : FMT_9BYTE_CM;
			/* checksum for [5A 05 00 fmt] */
			uint8_t cmd[5] = {
					TF_CMD_HEADER, 0x05U, 0x00U, fmt_byte,
					(uint8_t)((TF_CMD_HEADER + 0x05U + 0x00U + fmt_byte) & 0xFFU)
			};
			uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
			HAL_StatusTypeDef hs;

			hs = HAL_I2C_Master_Transmit(htf->hi2c, dev_addr,
					cmd, sizeof(cmd), TF_I2C_TIMEOUT_MS);
			if (hs != HAL_OK) return TF_ERR_HAL;

			HAL_Delay(10U);  /* sensor needs time to prepare response */

			uint8_t frame[TF_UART_FRAME_LEN];
			hs = HAL_I2C_Master_Receive(htf->hi2c, dev_addr,
					frame, TF_UART_FRAME_LEN, TF_I2C_TIMEOUT_MS);
			if (hs != HAL_OK) return TF_ERR_HAL;
			return parse_data_frame(htf, frame, data);
		}
	}

	/* ---- UART path: scan ring buffer for 0x59 0x59 header ---- */
	uint8_t frame[TF_UART_FRAME_LEN];
	uint16_t avail = ring_buf_available(htf);

	while (avail >= TF_UART_FRAME_LEN) {
		uint8_t peek;
		/* Peek at the first byte without consuming it */
		uint16_t tmp_tail = htf->rx_tail;
		peek = htf->rx_buf[tmp_tail];

		if (peek != TF_FRAME_HEADER) {
			/* Not a header — discard byte and advance */
			ring_buf_get(htf, &peek);
			avail--;
			continue;
		}

		/* Peek second byte */
		uint16_t next = (tmp_tail + 1U) & (TF_UART_RX_BUF_SIZE - 1U);
		if (htf->rx_buf[next] != TF_FRAME_HEADER) {
			/* First byte matched but second didn't — discard first */
			ring_buf_get(htf, &peek);
			avail--;
			continue;
		}

		/* We have at least one potential frame starting here.
		 * Consume all 9 bytes. */
		if (avail < TF_UART_FRAME_LEN) break;

		for (uint8_t i = 0U; i < TF_UART_FRAME_LEN; i++) {
			ring_buf_get(htf, &frame[i]);
		}

		TF_Status_t st = parse_data_frame(htf, frame, data);
		if (st == TF_OK && htf->on_data) {
			htf->on_data(data, htf->user_ctx);
		}
		return st;
	}

	return TF_ERR_NO_DATA;
}

TF_Status_t TF_ReadDataBlocking(TF_Handle_t *htf, TF_Data_t *data,
		uint32_t timeout_ms)
{
	if (!htf || !data)     return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	uint32_t t_start = HAL_GetTick();
	TF_Status_t st;

	do {
		st = TF_ReadData(htf, data);
		if (st == TF_OK) return TF_OK;
		HAL_Delay(1U);
	} while ((HAL_GetTick() - t_start) < timeout_ms);

	return TF_ERR_TIMEOUT;
}

TF_Status_t TF_GetVersion(TF_Handle_t *htf,
		uint8_t *major, uint8_t *minor, uint8_t *revision)
{
	if (!htf || !major || !minor || !revision) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	/* Command: [5A 04 01 5F] */
	uint8_t cmd[] = { TF_CMD_HEADER, 0x04U, CMD_GET_VERSION, 0x5FU };
	uint8_t rsp[7];

	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st != TF_OK) return st;

	/* Response: [5A 07 01 V1 V2 V3 SU]  Version = V3.V2.V1 */
	*revision = rsp[3];
	*minor    = rsp[4];
	*major    = rsp[5];
	return TF_OK;
}

TF_Status_t TF_SetFrameRate(TF_Handle_t *htf, uint16_t fps)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		uint8_t reg_data[2] = { (uint8_t)(fps & 0xFFU),
				(uint8_t)((fps >> 8U) & 0xFFU) };
		return i2c_write_regs(htf, TFNOVA_REG_FPS_LOW, reg_data, 2U);
	}

	/* UART command: [5A 06 03 LL HH SU] */
	uint8_t cmd[6];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x06U;
	cmd[2] = CMD_SAMPLE_FREQ;
	cmd[3] = (uint8_t)(fps & 0xFFU);
	cmd[4] = (uint8_t)((fps >> 8U) & 0xFFU);
	cmd[5] = calc_checksum(cmd, 5U);

	uint8_t rsp[6];
	return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
}

TF_Status_t TF_SetUnit(TF_Handle_t *htf, TF_Unit_t unit)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	uint8_t fmt = (unit == TF_UNIT_MM) ? FMT_9BYTE_MM : FMT_9BYTE_CM;

	/* UART command: [5A 05 05 FMT SU] */
	uint8_t cmd[5];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x05U;
	cmd[2] = CMD_OUTPUT_FORMAT;
	cmd[3] = fmt;
	cmd[4] = calc_checksum(cmd, 4U);

	uint8_t rsp[5];
	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st == TF_OK) htf->unit = unit;
	return st;
}



TF_Status_t TF_SetChecksumEnable(TF_Handle_t *htf, bool enable)
{
	if (!htf)              return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	/* Command: [5A 05 08 EN SU] */
	uint8_t cmd[5];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x05U;
	cmd[2] = 0x08U;               /* ID_FRAME_CHECKSUM_EN */
	cmd[3] = enable ? 0x01U : 0x00U;
	cmd[4] = calc_checksum(cmd, 4U);

	/* Fire and forget — don't validate the response checksum since
	 * checksum may not be enabled yet when sending this command */
	HAL_StatusTypeDef hs;
	if (htf->interface == TF_IF_UART) {
		HAL_UART_AbortReceive(htf->huart);
		hs = HAL_UART_Transmit(htf->huart, cmd, sizeof(cmd), TF_CMD_TIMEOUT_MS);
		HAL_Delay(50U);
		HAL_UART_Receive_IT(htf->huart, &htf->rx_byte, 1U);
	} else {
		uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
		hs = HAL_I2C_Master_Transmit(htf->hi2c, dev_addr,
				cmd, sizeof(cmd), TF_I2C_TIMEOUT_MS);
	}

	return (hs == HAL_OK) ? TF_OK : TF_ERR_HAL;
}


TF_Status_t TF_SetOutputEnable(TF_Handle_t *htf, bool enable)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		uint8_t val = enable ? 0x01U : 0x00U;
		return i2c_write_regs(htf, TFNOVA_REG_ENABLE, &val, 1U);
	}

	/* UART command: [5A 05 07 EN SU] */
	uint8_t cmd[5];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x05U;
	cmd[2] = CMD_OUTPUT_EN;
	cmd[3] = enable ? 0x01U : 0x00U;
	cmd[4] = calc_checksum(cmd, 4U);

	uint8_t rsp[5];
	return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
}

TF_Status_t TF_TriggerOnce(TF_Handle_t *htf)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;
	if (htf->type != TF_SENSOR_TF02PRO) return TF_ERR_UNSUPPORTED;

	/* Command: [5A 04 04 62] */
	uint8_t cmd[] = { TF_CMD_HEADER, 0x04U, CMD_TRIGGER, 0x62U };
	/* Response is the data frame itself — handled by TF_ReadDataBlocking() */
	HAL_StatusTypeDef hs = HAL_UART_Transmit(htf->huart, cmd, sizeof(cmd), 50U);
	return (hs == HAL_OK) ? TF_OK : TF_ERR_HAL;
}

TF_Status_t TF_SetIOMode(TF_Handle_t *htf, const TF_IO_Config_t *cfg)
{
	if (!htf || !cfg)      return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		/* TF-NOVA I2C: distance unit in registers is mm */
		uint16_t dist_mm = cfg->dist_cm * 10U;
		uint16_t zone_mm = cfg->zone_cm * 10U;
		uint8_t regs[10];
		regs[0] = (uint8_t)(dist_mm & 0xFFU);
		regs[1] = (uint8_t)((dist_mm >> 8U) & 0xFFU);
		regs[2] = (uint8_t)(zone_mm & 0xFFU);
		regs[3] = (uint8_t)((zone_mm >> 8U) & 0xFFU);
		regs[4] = (uint8_t)(cfg->delay1_ms & 0xFFU);
		regs[5] = (uint8_t)((cfg->delay1_ms >> 8U) & 0xFFU);
		regs[6] = (uint8_t)(cfg->delay2_ms & 0xFFU);
		regs[7] = (uint8_t)((cfg->delay2_ms >> 8U) & 0xFFU);

		TF_Status_t st = i2c_write_regs(htf, TFNOVA_REG_ONOFF_DIST_L, regs, 8U);
		if (st != TF_OK) return st;

		uint8_t en = (uint8_t)cfg->mode;
		return i2c_write_regs(htf, TFNOVA_REG_ONOFF_EN, &en, 1U);
	}

	/* UART command for both sensors:
	 * TF-NOVA  : [5A 0E 3B Opt Mode Dist_L Dist_H Zone_L Zone_H D1_L D1_H D2_L D2_H SU]
	 * TF02-Pro : [5A 09 3B MODE DL DH ZoneL ZoneH 00]  (slightly different) */
	if (htf->type == TF_SENSOR_TFNOVA) {
		uint8_t cmd[14];
		cmd[0]  = TF_CMD_HEADER;
		cmd[1]  = 0x0EU;
		cmd[2]  = CMD_ON_OFF_MODE;
		cmd[3]  = 0x01U;                                    /* Opt = write */
		cmd[4]  = (uint8_t)cfg->mode;
		cmd[5]  = (uint8_t)(cfg->dist_cm & 0xFFU);
		cmd[6]  = (uint8_t)((cfg->dist_cm >> 8U) & 0xFFU);
		cmd[7]  = (uint8_t)(cfg->zone_cm & 0xFFU);
		cmd[8]  = (uint8_t)((cfg->zone_cm >> 8U) & 0xFFU);
		cmd[9]  = (uint8_t)(cfg->delay1_ms & 0xFFU);
		cmd[10] = (uint8_t)((cfg->delay1_ms >> 8U) & 0xFFU);
		cmd[11] = (uint8_t)(cfg->delay2_ms & 0xFFU);
		cmd[12] = (uint8_t)((cfg->delay2_ms >> 8U) & 0xFFU);
		cmd[13] = calc_checksum(cmd, 13U);

		uint8_t rsp[14];
		return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));

	} else {
		/* TF02-Pro: [5A 09 3B MODE DL DH ZoneL ZoneH 00] */
		uint8_t cmd[9];
		cmd[0] = TF_CMD_HEADER;
		cmd[1] = 0x09U;
		cmd[2] = CMD_ON_OFF_MODE;
		cmd[3] = (uint8_t)cfg->mode;
		cmd[4] = (uint8_t)(cfg->dist_cm & 0xFFU);
		cmd[5] = (uint8_t)((cfg->dist_cm >> 8U) & 0xFFU);
		cmd[6] = (uint8_t)(cfg->zone_cm & 0xFFU);
		cmd[7] = (uint8_t)((cfg->zone_cm >> 8U) & 0xFFU);
		cmd[8] = 0x00U;   /* checksum disabled by default */

		uint8_t rsp[9];
		return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	}
}

TF_Status_t TF_SetBaudRate(TF_Handle_t *htf, uint32_t baudrate)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	/* Command: [5A 08 06 B0 B1 B2 B3 SU]  (little endian 32-bit baud) */
	uint8_t cmd[8];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x08U;
	cmd[2] = CMD_BAUD_RATE;
	cmd[3] = (uint8_t)(baudrate & 0xFFU);
	cmd[4] = (uint8_t)((baudrate >> 8U)  & 0xFFU);
	cmd[5] = (uint8_t)((baudrate >> 16U) & 0xFFU);
	cmd[6] = (uint8_t)((baudrate >> 24U) & 0xFFU);
	cmd[7] = calc_checksum(cmd, 7U);

	uint8_t rsp[9];
	return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
}

TF_Status_t TF_SetDistRange(TF_Handle_t *htf, uint16_t min_mm, uint16_t max_mm)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;
	if (htf->type != TF_SENSOR_TFNOVA) return TF_ERR_UNSUPPORTED;

	if (htf->interface == TF_IF_I2C) {
		uint8_t regs[4];
		regs[0] = (uint8_t)(min_mm & 0xFFU);
		regs[1] = (uint8_t)((min_mm >> 8U) & 0xFFU);
		TF_Status_t st = i2c_write_regs(htf, TFNOVA_REG_MIN_DIST_L, regs, 2U);
		if (st != TF_OK) return st;
		regs[0] = (uint8_t)(max_mm & 0xFFU);
		regs[1] = (uint8_t)((max_mm >> 8U) & 0xFFU);
		return i2c_write_regs(htf, TFNOVA_REG_MAX_DIST_L, regs, 2U);
	}

	/* UART: [5A 09 3A 01 MinL MinH MaxL MaxH SU] */
	uint8_t cmd[9];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x09U;
	cmd[2] = CMD_DIST_RANGE;
	cmd[3] = 0x01U;  /* write */
	cmd[4] = (uint8_t)(min_mm & 0xFFU);
	cmd[5] = (uint8_t)((min_mm >> 8U) & 0xFFU);
	cmd[6] = (uint8_t)(max_mm & 0xFFU);
	cmd[7] = (uint8_t)((max_mm >> 8U) & 0xFFU);
	cmd[8] = calc_checksum(cmd, 8U);

	uint8_t rsp[9];
	return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
}

TF_Status_t TF_SetI2CAddress(TF_Handle_t *htf, uint8_t new_addr)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;
	if (new_addr < 0x08U || new_addr > 0x77U) return TF_ERR_PARAM;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		TF_Status_t st = i2c_write_regs(htf, TFNOVA_REG_SLAVE_ADDR, &new_addr, 1U);
		if (st == TF_OK) htf->i2c_addr = new_addr;
		return st;
	}

	/* UART: [5A 06 0B 01 ADDR SU]  (TF-NOVA write form) */
	uint8_t cmd[6];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x06U;
	cmd[2] = CMD_IIC_SLAVE_ADDR;
	cmd[3] = 0x01U;      /* write */
	cmd[4] = new_addr;
	cmd[5] = calc_checksum(cmd, 5U);

	uint8_t rsp[6];
	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st == TF_OK) htf->i2c_addr = new_addr;
	return st;
}

TF_Status_t TF_SaveSettings(TF_Handle_t *htf)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		uint8_t val = 0x01U;
		return i2c_write_regs(htf, TFNOVA_REG_SAVE, &val, 1U);
	}

	/* UART: [5A 04 11 6F] */
	uint8_t cmd[] = { TF_CMD_HEADER, 0x04U, CMD_SAVE_SETTINGS, 0x6FU };
	uint8_t rsp[5];
	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st == TF_OK && rsp[3] != 0x00U) return TF_ERR_HAL;
	return st;
}

TF_Status_t TF_RestoreDefaults(TF_Handle_t *htf)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		uint8_t val = 0x01U;
		return i2c_write_regs(htf, TFNOVA_REG_RESTORE, &val, 1U);
	}

	/* UART: [5A 04 10 6E] */
	uint8_t cmd[] = { TF_CMD_HEADER, 0x04U, CMD_RESTORE_DEFAULT, 0x6EU };
	uint8_t rsp[5];
	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st == TF_OK && rsp[3] != 0x00U) return TF_ERR_HAL;
	return st;
}


TF_Status_t TF_SetInterface(TF_Handle_t *htf, TF_Interface_t new_if)
{
	if (!htf)                           return TF_ERR_PARAM;
	if (!htf->initialised)              return TF_ERR_NOT_INIT;
	if (htf->type != TF_SENSOR_TFNOVA) return TF_ERR_UNSUPPORTED;

	/* Prevent no-op — already on requested interface */
	if (htf->interface == new_if)       return TF_OK;

	/* If switching TO I2C, must currently be on UART (and vice versa) */
	/* uart_send_cmd() already routes correctly based on htf->interface  */

	uint8_t cmd[6];
	cmd[0] = TF_CMD_HEADER;
	cmd[1] = 0x06U;
	cmd[2] = CMD_IF_PROTOCOL;
	cmd[3] = 0x01U;               /* write                                   */
	cmd[4] = (uint8_t)new_if;    /* 0x00 = UART, 0x01 = IIC                 */
	cmd[5] = calc_checksum(cmd, 5U);

	uint8_t rsp[6];
	TF_Status_t st = uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
	if (st != TF_OK) return st;

	if (rsp[3] != 0x00U) return TF_ERR_HAL;

	st = TF_SaveSettings(htf);
	if (st != TF_OK) return st;

	return TF_OK;
	/* !! Power cycle required before new interface will respond !! */
}

TF_Status_t TF_SoftReset(TF_Handle_t *htf)
{
	if (!htf) return TF_ERR_PARAM;
	if (!htf->initialised) return TF_ERR_NOT_INIT;

	if (htf->interface == TF_IF_I2C && htf->type == TF_SENSOR_TFNOVA) {
		uint8_t val = 0x02U;  /* reboot */
		return i2c_write_regs(htf, TFNOVA_REG_REBOOT, &val, 1U);
	}

	/* UART: [5A 04 02 60] */
	uint8_t cmd[] = { TF_CMD_HEADER, 0x04U, CMD_SOFT_RESET, 0x60U };
	uint8_t rsp[5];
	return uart_send_cmd(htf, cmd, sizeof(cmd), rsp, sizeof(rsp));
}

const char *TF_StatusStr(TF_Status_t status)
{
	switch (status) {
	case TF_OK:             return "OK";
	case TF_ERR_PARAM:      return "ERR_PARAM";
	case TF_ERR_TIMEOUT:    return "ERR_TIMEOUT";
	case TF_ERR_CHECKSUM:   return "ERR_CHECKSUM";
	case TF_ERR_NO_DATA:    return "ERR_NO_DATA";
	case TF_ERR_HAL:        return "ERR_HAL";
	case TF_ERR_NOT_INIT:   return "ERR_NOT_INIT";
	case TF_ERR_UNSUPPORTED:return "ERR_UNSUPPORTED";
	default:                return "UNKNOWN";
	}
}

const char *TF_SensorName(TF_SensorType_t type)
{
	switch (type) {
	case TF_SENSOR_TFNOVA:  return "TF-NOVA";
	case TF_SENSOR_TF02PRO: return "TF02-Pro";
	default:                return "Unknown";
	}
}

void TF_Bus_Register(TF_Bus_t *bus,
                     TF_Handle_t *htf, TF_Data_t *data)
{
    if (bus->count >= TF_BUS_MAX_SENSORS) return;
    bus->sensors[bus->count] = htf;
    bus->results[bus->count] = data;
    bus->count++;
}


void TF_Bus_Tick(TF_Bus_t *bus)
{
    if (bus->count == 0) return;

    if (bus->busy) {
        if ((HAL_GetTick() - bus->transfer_timestamp) > TF_BUS_TRANSFER_TIMEOUT_MS) {
            HAL_I2C_Master_Abort_IT(bus->sensors[bus->active]->hi2c,
                                    (uint16_t)(bus->sensors[bus->active]->i2c_addr << 1U));
            bus->sensors[bus->active]->i2c_transfer_active = false;
            bus->sensors[bus->active]->dma_state           = TF_DMA_STATE_ERROR;
            bus->results[bus->active]->valid               = false;
            bus->busy   = false;
            bus->active = (bus->active + 1U) % bus->count;
        }
        return;
    }

    TF_Handle_t *htf = bus->sensors[bus->active];
    if (HAL_I2C_GetState(htf->hi2c) != HAL_I2C_STATE_READY) return;


    TF_ReadData(htf, bus->results[bus->active]);

    if (htf->dma_state == TF_DMA_STATE_TX_PEND ||
        htf->dma_state == TF_DMA_STATE_RX_PEND) {
        bus->transfer_timestamp = HAL_GetTick();
        bus->busy = true;
    } else {
        bus->active = (bus->active + 1U) % bus->count;
    }
}
void TF_Bus_TxCpltCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c)
{
    if (!bus->busy) return;
  TF_I2C_TxCpltCallback(bus->sensors[bus->active], hi2c);

}


void TF_Bus_RxCpltCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c)
{
    if (!bus->busy) return;
    TF_I2C_RxCpltCallback(bus->sensors[bus->active], hi2c);

    bus->busy   = false;
    bus->active = (bus->active + 1U) % bus->count;
}


void TF_Bus_ErrorCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c)
{
    if (!bus->busy) return;
    TF_I2C_ErrorCallback(bus->sensors[bus->active], hi2c);
    bus->busy   = false;
    bus->active = (bus->active + 1U) % bus->count;
}


/* ============================================================================
 * Private helpers
 * ========================================================================== */

/**
 * @brief  Calculate the lower 8 bits of the byte sum of the first `len` bytes.
 */
static uint8_t calc_checksum(const uint8_t *buf, uint16_t len)
{
	uint32_t sum = 0U;
	for (uint16_t i = 0U; i < len; i++) {
		sum += buf[i];
	}
	return (uint8_t)(sum & 0xFFU);
}

/**
 * Parse a 9-byte data frame and populate a TF_Data_t struct.
 *
 * The frame layout is identical for both sensors:
 *   [0x59][0x59][Dist_L][Dist_H][Sig_L][Sig_H][Byte6][Byte7][Checksum]
 *
 * Byte6 / Byte7 differ between sensors and output formats:
 *   TF-NOVA  9byte/cm : Byte6 = Temp (raw °C), Byte7 = Confidence
 *   TF-NOVA  9byte/mm : same layout, Dist unit changes
 *   TF02-Pro 9byte/cm : Byte6 = Temp_L, Byte7 = Temp_H (raw 16-bit)
 *   TF02-Pro 9byte/mm : same
 */
static TF_Status_t parse_data_frame(TF_Handle_t *htf,
		const uint8_t *frame, TF_Data_t *data)
{
	/* Validate header */
	if (frame[0] != TF_FRAME_HEADER || frame[1] != TF_FRAME_HEADER) {
		return TF_ERR_CHECKSUM;
	}

	/* Validate checksum */
	uint8_t expected_cs = calc_checksum(frame, TF_UART_FRAME_LEN - 1U);
	if (expected_cs != frame[TF_UART_FRAME_LEN - 1U]) {
		return TF_ERR_CHECKSUM;
	}

	data->distance   = (uint16_t)(frame[2] | ((uint16_t)frame[3] << 8U));
	data->strength   = (uint16_t)(frame[4] | ((uint16_t)frame[5] << 8U));

	if (htf->type == TF_SENSOR_TFNOVA) {
		if (htf->interface == TF_IF_I2C) {
			/* I2C register: 16-bit value in units of 0.01°C */
			uint16_t raw_temp = (uint16_t)(frame[6] | ((uint16_t)frame[7] << 8U));
			data->temperature = (int16_t)(raw_temp / 10U); /* → tenths of °C */
			data->confidence  = 0U;  /* not in register map */
		} else {
			/* UART frame: Byte6 = integer °C, Byte7 = confidence */
			data->temperature = (int16_t)frame[6] * 10;
			data->confidence  = frame[7];
		}
	} else {
		/* TF02-Pro: Byte6=Temp_L, Byte7=Temp_H, temperature(°C) = raw/8-256 */
		uint16_t raw_temp = (uint16_t)(frame[6] | ((uint16_t)frame[7] << 8U));
		int32_t  temp_c   = (int32_t)raw_temp / 8 - 256;
		data->temperature = (int16_t)(temp_c * 10);   /* convert to tenths °C */
		data->confidence  = 0U;                        /* not available        */
	}

	/* Mark out-of-range readings as invalid (TF02-Pro sentinel = 4500 cm) */
	data->valid = !((htf->type == TF_SENSOR_TF02PRO) &&
			(data->distance == TF02PRO_OUT_OF_RANGE));

	return TF_OK;
}

/**
 * Send a UART command frame and receive the response (blocking).
 *
 * Briefly pauses the ring-buffer IT reception, sends the command via polling,
 * waits for the response via polling, then re-enables IT reception.
 */
static TF_Status_t uart_send_cmd(TF_Handle_t *htf,
		const uint8_t *cmd, uint8_t cmd_len,
		uint8_t *rsp, uint8_t rsp_len)
{
	if (htf->interface == TF_IF_I2C) {
		/* Route I2C-mode configuration via I2C master transmit */
		uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
		HAL_StatusTypeDef hs;
		hs = HAL_I2C_Master_Transmit(htf->hi2c, dev_addr,
				(uint8_t *)cmd, cmd_len,
				TF_I2C_TIMEOUT_MS);
		if (hs != HAL_OK) return TF_ERR_HAL;
		HAL_Delay(100U);  /* sensor processing time per datasheet */
		hs = HAL_I2C_Master_Receive(htf->hi2c, dev_addr,
				rsp, rsp_len,
				TF_I2C_TIMEOUT_MS);
		return (hs == HAL_OK) ? TF_OK : TF_ERR_HAL;
	}

	/* UART path */
	HAL_StatusTypeDef hs;

	/* Abort any ongoing IT reception to prevent ring-buffer corruption */
	HAL_UART_AbortReceive(htf->huart);

	/* Send command */
	hs = HAL_UART_Transmit(htf->huart, (uint8_t *)cmd, cmd_len, TF_CMD_TIMEOUT_MS);
	if (hs != HAL_OK) {
		/* Re-enable IT reception before returning */
		HAL_UART_Receive_IT(htf->huart, &htf->rx_byte, 1U);
		return TF_ERR_HAL;
	}

	/* Receive response via blocking call */
	hs = HAL_UART_Receive(htf->huart, rsp, rsp_len, TF_CMD_TIMEOUT_MS);

	/* Re-enable single-byte IT reception */
	HAL_UART_Receive_IT(htf->huart, &htf->rx_byte, 1U);

	if (hs != HAL_OK) return TF_ERR_TIMEOUT;

	/* Basic response validation: header byte should be TF_CMD_HEADER */
	if (rsp[0] != TF_CMD_HEADER) return TF_ERR_CHECKSUM;

	return TF_OK;
}

/**
 * Arm the async TX stage of the DMA/IT pipeline.
 *
 * Sends the single register address byte (0x00 = DIST_LOW) to the sensor.
 * HAL fires TxCpltCallback when done, which then arms the RX stage.
 * Called from TF_Init_I2C_Async(), TF_ReadData() (IDLE / ERROR recovery),
 * and from TF_ReadData() after consuming a DONE frame.
 */
static TF_Status_t i2c_kick_tx(TF_Handle_t *htf)
{
	/* Don't attempt a transfer if the peripheral is busy —
	 * this prevents hammering the bus on repeated AF errors */
	if (HAL_I2C_GetState(htf->hi2c) != HAL_I2C_STATE_READY) {
		htf->dma_state = TF_DMA_STATE_IDLE;
		return TF_ERR_HAL;
	}

	uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
	htf->dma_state    = TF_DMA_STATE_TX_PEND;
	htf->i2c_transfer_active = true;
	HAL_StatusTypeDef hs;

	/* if (htf->i2c_mode == TF_I2C_DMA) {
        hs = HAL_I2C_Master_Transmit_DMA(htf->hi2c, dev_addr,
                                          &htf->dma_reg_addr, 1U);
    } else {
         TF_I2C_IT
        hs = HAL_I2C_Master_Transmit_IT(htf->hi2c, dev_addr,
                                         &htf->dma_reg_addr, 1U);
    }
    ---The reason this is commented out is because it is not worth using a whole DMA stream channel
     	 just to transmit like 3 bytes using DMA---
	 */
	hs = HAL_I2C_Master_Transmit_IT(htf->hi2c, dev_addr,
			&htf->dma_reg_addr, 1U);

	if (hs != HAL_OK) {
		htf->dma_state = TF_DMA_STATE_ERROR;
		htf->i2c_transfer_active = false;
		return TF_ERR_HAL;
	}
	return TF_OK;
}

/**
 * Write `len` bytes to consecutive I2C registers starting at `reg`.
 */
static TF_Status_t i2c_write_regs(TF_Handle_t *htf, uint8_t reg,
		const uint8_t *data, uint16_t len)
{
	uint8_t buf[TF_MAX_CMD_LEN];
	if (len + 1U > TF_MAX_CMD_LEN) return TF_ERR_PARAM;

	buf[0] = reg;
	for (uint16_t i = 0U; i < len; i++) buf[i + 1U] = data[i];

	/* Pause async pipeline if running */
	if (htf->i2c_mode != TF_I2C_BLOCKING) {
		/* Wait for any in-flight transfer to finish */
		uint32_t t = HAL_GetTick();
		while (htf->dma_state == TF_DMA_STATE_TX_PEND ||
				htf->dma_state == TF_DMA_STATE_RX_PEND) {
			if ((HAL_GetTick() - t) > 50U) {
				htf->dma_state = TF_DMA_STATE_IDLE;
				break;
			}
		}
	}

	uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
	HAL_StatusTypeDef hs = HAL_I2C_Master_Transmit(htf->hi2c, dev_addr,
			buf, (uint16_t)(len + 1U),
			TF_I2C_TIMEOUT_MS);

	/* Resume pipeline */
	if (htf->i2c_mode != TF_I2C_BLOCKING && hs == HAL_OK) {
		htf->dma_state = TF_DMA_STATE_IDLE;
		i2c_kick_tx(htf);
	}

	return (hs == HAL_OK) ? TF_OK : TF_ERR_HAL;
}

/**
 * Read `len` bytes from consecutive I2C registers starting at `reg`.
 */
static TF_Status_t i2c_read_regs(TF_Handle_t *htf, uint8_t reg,
		uint8_t *data, uint16_t len)
{
	uint16_t dev_addr = (uint16_t)(htf->i2c_addr << 1U);
	HAL_StatusTypeDef hs;

	/* Write register address */
	hs = HAL_I2C_Master_Transmit(htf->hi2c, dev_addr, &reg, 1U,
			TF_I2C_TIMEOUT_MS);

	if (hs != HAL_OK) return TF_ERR_HAL;

	/* Read data */
	hs = HAL_I2C_Master_Receive(htf->hi2c, dev_addr, data, len,
			TF_I2C_TIMEOUT_MS);
	return (hs == HAL_OK) ? TF_OK : TF_ERR_HAL;
	//    return (hs == HAL_OK) ? TF_OK : -9;

}

/**
 * Consume one byte from the ring buffer. Returns false if empty.
 */
static bool ring_buf_get(TF_Handle_t *htf, uint8_t *byte_out)
{
	if (htf->rx_tail == htf->rx_head) return false;
	*byte_out   = htf->rx_buf[htf->rx_tail];
	htf->rx_tail = (htf->rx_tail + 1U) & (TF_UART_RX_BUF_SIZE - 1U);
	return true;
}

/**
 * Return the number of bytes available in the ring buffer.
 */
static uint16_t ring_buf_available(const TF_Handle_t *htf)
{
	return (uint16_t)((htf->rx_head - htf->rx_tail) & (TF_UART_RX_BUF_SIZE - 1U));
}




















