/*
 * benewake_tf.h
 *
 *  Created on: Mar 14, 2026
 *      Author: Ibrahim Meselhy
 */
/*
 * This is a library for Benewake TF-NOVA and TF02-Pro sensors.
 * This library supports 3 modes:
 *  - UART()
 *  - I2C ()
 *  - I/O(on-off mode, GPIO pin)
 *
 *	Latest Update:
 *	- I2C pathway for TF_NOVA has had the following changes:
 *		- I2C Bus Handle to handle multiple sensors on the same I2C bus.
 *		- Timeout stamp added to introduce timeout incase one sensor stops working.
 *
 * */

#ifndef SRC_BENEWAKE_BENEWAKE_TF_H_
#define SRC_BENEWAKE_BENEWAKE_TF_H_

#include <stdint.h>
#include <stdbool.h>
#include "../../Platform/H7_system.h"
#include "../../BIOS/COM/i2c.h"


/** Default I2C slave address (7-bit) for both sensors */
#define TF_DEFAULT_I2C_ADDR         0x3U

/** UART frame length (bytes) */
#define TF_UART_FRAME_LEN           9U

/** UART frame header bytes */
#define TF_FRAME_HEADER             0x59U

/** Command frame header */
#define TF_CMD_HEADER               0x5AU

/** Out-of-range / low-signal sentinel value for TF02-Pro (cm) */
#define TF02PRO_OUT_OF_RANGE        4500U

/** Receive ring-buffer size — must be a power of 2, >= 2*TF_UART_FRAME_LEN */
#define TF_UART_RX_BUF_SIZE         64U

#define TF_BUS_MAX_SENSORS  8

/* Status codes returned by all functions */
typedef enum {
    TF_OK               =  0,   /* Success                                  */
    TF_ERR_PARAM        = -1,   /* Invalid parameter                        */
    TF_ERR_TIMEOUT      = -2,   /* Operation timed out                      */
    TF_ERR_CHECKSUM     = -3,   /* Frame checksum mismatch                  */
    TF_ERR_NO_DATA      = -4,   /* No complete frame available yet          */
    TF_ERR_HAL          = -5,   /* Underlying HAL call failed               */
    TF_ERR_NOT_INIT     = -6,   /* Handle not initialised                   */
    TF_ERR_UNSUPPORTED  = -7,   /* Feature not supported by this sensor     */
} TF_Status_t;

/*Sensor Types*/
typedef enum {
    TF_SENSOR_TFNOVA   = 0,
    TF_SENSOR_TF02PRO  = 1,
} TF_SensorType_t;


/* Communication interface */
typedef enum {
    TF_IF_UART = 0,
    TF_IF_I2C  = 1,
} TF_Interface_t;

/**
 * I2C transfer mode for TF-NOVA.
 *
 * TF_I2C_BLOCKING  — HAL blocking calls.
 * TF_I2C_IT        — Interrupt-driven TX then RX.
 * TF_I2C_DMA       — DMA-driven TX then RX.
 *                    Best choice for TF-NOVA since it uses direct register reads.
 *
 * TF02-Pro always uses blocking I2C regardless of this setting because its
 * "obtain data frame" command requires a fixed 100 ms sensor-processing gap
 * between TX and RX — there is nothing useful the DMA can overlap with.
 */

typedef enum {
    TF_I2C_BLOCKING = 0,
    TF_I2C_IT       = 1,
    TF_I2C_DMA      = 2,
} TF_I2C_Mode_t;


/**
 * Internal DMA/IT transfer state for TF-NOVA I2C async reads.
 */

typedef enum {
    TF_DMA_STATE_IDLE     = 0,
    TF_DMA_STATE_TX_PEND  = 1,
    TF_DMA_STATE_RX_PEND  = 2,
    TF_DMA_STATE_DONE     = 3,
    TF_DMA_STATE_ERROR    = 4,
} TF_DMA_State_t;

/**
 *  Distance unit reported in TF_Data_t.
 *  TF_UNIT_CM is the power-on default for both sensors.
 */

typedef enum {
    TF_UNIT_CM = 0,
    TF_UNIT_MM = 1,
} TF_Unit_t;

/**
 * I/O (on-off) operating mode — mirrors the sensor's MODE field.
 */

typedef enum {
    TF_IO_MODE_DISABLED  = 0,
    TF_IO_MODE_NEAR_HIGH = 1,   /* Pin HIGH when object is closer than Dist */
    TF_IO_MODE_NEAR_LOW  = 2,   /* Pin LOW  when object is closer than Dist */
} TF_IO_Mode_t;

/**
 * Decoded measurement data from one sensor frame.
 */
typedef struct {
    uint16_t distance;          /* Distance value (unit depends on TF_Unit_t) */
    uint16_t strength;          /* Signal strength (0–65535)                  */
    int16_t  temperature;       /* Chip temperature in tenths of °C
                                     (TF-NOVA: raw°C × 10; TF02-Pro: Temp/8-256) */
    uint8_t  confidence;        /* Confidence 0–100 (TF-NOVA only, else 0)    */
    bool     valid;             /* false if out-of-range or checksum error     */
} TF_Data_t;

/**
 * On-off mode configuration (for TF_SetIOMode).
 */
typedef struct {
    TF_IO_Mode_t mode;          /* Operating mode                            */
    uint16_t     dist_cm;       /* Critical distance threshold (cm)          */
    uint16_t     zone_cm;       /* Hysteresis zone size (cm)                 */
    uint16_t     delay1_ms;     /* Approaching delay (ms)                    */
    uint16_t     delay2_ms;     /* Leaving delay (ms)                        */
} TF_IO_Config_t;

/**
 * Main driver handle.  Allocate one per physical sensor.
 * Initialise via TF_Init_UART() or TF_Init_I2C().
 */
typedef struct {
    /* --- sensor identity --- */
    TF_SensorType_t  type;
    TF_Interface_t   interface;
    TF_Unit_t        unit;

    /* --- HAL peripheral handles (only the active one is used) --- */
    UART_HandleTypeDef *huart;
    I2C_HandleTypeDef  *hi2c;

    /* --- I2C address (7-bit, default TF_DEFAULT_I2C_ADDR) --- */
    uint8_t  i2c_addr;

    TF_I2C_Mode_t    i2c_mode;

    volatile TF_DMA_State_t  dma_state;
    uint8_t                  dma_reg_addr;          /* Single TX byte buffer  */
    uint8_t                  dma_rx_buf[8];         /* 8 raw register bytes   */
    TF_Data_t                last_dma_data;         /* Most recent parsed result */

    /* --- UART ring buffer (filled by IRQ, consumed by TF_ReadData) --- */
    volatile uint8_t  rx_buf[TF_UART_RX_BUF_SIZE];
    volatile uint16_t rx_head;     /* Write index (updated in ISR)          */
    volatile uint16_t rx_tail;     /* Read  index (updated in TF_ReadData)  */

    uint8_t  rx_byte;              /* Current receive byte for IT mode       */

    volatile bool i2c_transfer_active;  // true while this sensor owns the bus
    /* --- optional user callback, called when a new frame is decoded --- */
    void (*on_data)(const TF_Data_t *data, void *user_ctx);
    void *user_ctx;

    /* --- internal state --- */
    bool     initialised;
} TF_Handle_t;


typedef struct {
    TF_Handle_t     *sensors[TF_BUS_MAX_SENSORS];
    TF_Data_t       *results[TF_BUS_MAX_SENSORS];
    uint8_t          count;
    volatile uint8_t active;   /* index of sensor currently owning bus */
    volatile bool    busy;     /* true = transfer in flight             */
    uint32_t         transfer_timestamp;   /* timestamp for timeout */
} TF_Bus_t;

extern TF_Bus_t i2c5_bus;

extern TF_Data_t TF02_data;
extern TF_Data_t NOVA_data;
extern TF_Data_t TF_F_data;
extern TF_Data_t TF_B_data;
extern TF_Data_t TF_R_data;
extern TF_Data_t TF_L_data;
extern TF_Handle_t TF02;
extern TF_Handle_t TF_NOVA;
extern TF_Handle_t TF_NOVA3;

extern TF_Handle_t TF_F;
extern TF_Handle_t TF_B;
extern TF_Handle_t TF_R;
extern TF_Handle_t TF_L;



/* ============================================================================
 * Initialisation API
 * ========================================================================== */

/**
 * Initialise a TF sensor handle for UART communication.
 *
 *
 * @param  htf    Pointer to an uninitialized TF_Handle_t.
 * @param  type   Sensor model (TF_SENSOR_TFNOVA or TF_SENSOR_TF02PRO).
 * @param  huart  Pointer to the HAL UART handle.
 * @return TF_OK on success, TF_ERR_PARAM if any pointer is NULL.
 */
TF_Status_t TF_Init_UART(TF_Handle_t *htf, TF_SensorType_t type,
                          UART_HandleTypeDef *huart);

/**
 * Initialise a TF sensor handle for I2C communication (blocking mode).
 *
 * Equivalent to calling TF_Init_I2C_Async() with TF_I2C_BLOCKING.
 *
 * @param  htf      Pointer to an uninitialized TF_Handle_t.
 * @param  type     Sensor model.
 * @param  hi2c     Pointer to the HAL I2C handle.
 * @param  i2c_addr 7-bit I2C address.
 * @return TF_OK on success.
 */
TF_Status_t TF_Init_I2C(TF_Handle_t *htf, TF_SensorType_t type,
                         I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);

/**
 * Initialise a TF-NOVA handle for I2C with selectable transfer mode.
 *
 * @param  htf      Pointer to an uninitialized TF_Handle_t.
 * @param  hi2c     Pointer to the HAL I2C handle (TF-NOVA only).
 * @param  i2c_addr 7-bit I2C address.
 * @param  mode     TF_I2C_BLOCKING, TF_I2C_IT, or TF_I2C_DMA.
 * @return TF_OK on success, TF_ERR_UNSUPPORTED if type is TF02-Pro.
 */
TF_Status_t TF_Init_I2C_Async(TF_Handle_t *htf,
                                I2C_HandleTypeDef *hi2c, uint8_t i2c_addr,
                                TF_I2C_Mode_t mode);

/**
 * Register an optional callback invoked each time a valid frame arrives.
 * The callback is called from TF_ReadData().
 */
void TF_RegisterCallback(TF_Handle_t *htf,
                         void (*cb)(const TF_Data_t *data, void *ctx),
                         void *user_ctx);

/* ============================================================================
 * I2C DMA / IT callbacks — forward from your HAL_I2C callbacks
 * ========================================================================== */

/**
 * Forward HAL's I2C TX-complete event to the library state machine.
 *
 * Place in HAL_I2C_MasterTxCpltCallback():
 *
 * Only needed when using TF_I2C_DMA or TF_I2C_IT mode.
 * Safe to call even if the handle is in blocking mode (no-op).
 */
TF_Status_t TF_I2C_TxCpltCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c);

/**
 * Forward HAL's I2C RX-complete event to the library state machine.
 *
 * When dma_state transitions to TF_DMA_STATE_DONE the next call to
 * TF_ReadData() will return TF_OK with fresh data and immediately re-arm
 * the next DMA transfer.
 *
 * Only needed when using TF_I2C_DMA or TF_I2C_IT mode.
 */
void TF_I2C_RxCpltCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c);
/**
 * Forward HAL's I2Cx_ER_IRQHandler
 */
void TF_I2C_ErrorCallback(TF_Handle_t *htf, I2C_HandleTypeDef *hi2c);

/* ============================================================================
 * Data acquisition API
 * ========================================================================== */

/**
 * Read the latest distance measurement.
 *
 * UART mode : Scans the ring buffer for a complete, valid 9-byte frame.
 *             Returns TF_ERR_NO_DATA if no frame is available yet.
 * I2C  mode : Actively requests a data frame from the sensor.
 *
 * @param  htf   Initialised handle.
 * @param  data  Output structure to populate.
 * @return TF_OK, TF_ERR_NO_DATA, TF_ERR_CHECKSUM, or TF_ERR_HAL.
 */
TF_Status_t TF_ReadData(TF_Handle_t *htf, TF_Data_t *data);

/**
 * Block until a valid frame arrives or the timeout expires.
 *
 * for one-shot measurements or startup checks.
 *
 * @param  htf        Initialised handle.
 * @param  data       Output structure.
 * @param  timeout_ms Maximum wait time in milliseconds.
 * @return TF_OK or TF_ERR_TIMEOUT.
 */
TF_Status_t TF_ReadDataBlocking(TF_Handle_t *htf, TF_Data_t *data,
                                 uint32_t timeout_ms);

/* ============================================================================
 * Configuration command API (both UART and I2C)
 * ========================================================================== */

/**
 * @brief  Query the firmware version string.
 *
 * @param  htf      Initialised handle.
 * @param  major    Output: major version number.
 * @param  minor    Output: minor version number.
 * @param  revision Output: revision number.
 * @return TF_OK or error.
 */
TF_Status_t TF_GetVersion(TF_Handle_t *htf,
                           uint8_t *major, uint8_t *minor, uint8_t *revision);

/**
 * Set the measurement frame rate.
 *
 * TF-NOVA  : 1–900 Hz.
 * TF02-Pro : 1–1000 Hz (must satisfy 2000/n, n ≥ 2).
 *            Set to 0 to enable trigger mode (TF02-Pro only).
 *
 * @param  htf  Initialised handle.
 * @param  fps  Desired frame rate in Hz.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetFrameRate(TF_Handle_t *htf, uint16_t fps);

/**
 * Set the output distance unit.
 *
 * Sends the appropriate output-format command to the sensor.
 * Also updates htf->unit so TF_ReadData decodes distances correctly.
 *
 * @param  htf   Initialised handle.
 * @param  unit  TF_UNIT_CM or TF_UNIT_MM.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetUnit(TF_Handle_t *htf, TF_Unit_t unit);

/**
 * Enable or disable continuous data output.
 *
 * When disabled, no frames are emitted. Re-enable to resume streaming.
 * In trigger mode (TF02-Pro, fps=0), use TF_TriggerOnce() instead.
 *
 * @param  htf     Initialised handle.
 * @param  enable  true = enable output, false = disable.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetOutputEnable(TF_Handle_t *htf, bool enable);

/**
 * Request a single measurement (TF02-Pro trigger mode only).
 *
 * The sensor must have been configured with TF_SetFrameRate(htf, 0) first.
 * Follow with TF_ReadDataBlocking() to retrieve the result.
 *
 * @param  htf  Initialised handle (must be TF_SENSOR_TF02PRO).
 * @return TF_OK or TF_ERR_UNSUPPORTED.
 */
TF_Status_t TF_TriggerOnce(TF_Handle_t *htf);

/**
 * Configure the on-off (I/O proximity) mode.
 *
 * Sets the distance threshold, hysteresis zone, and switching delays.
 * Pin behaviour depends on TF_IO_Config_t.mode.
 *
 * @param  htf  Initialised handle.
 * @param  cfg  Pointer to filled TF_IO_Config_t structure.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetIOMode(TF_Handle_t *htf, const TF_IO_Config_t *cfg);

/**
 * Set the UART baud rate.
 *
 * The new rate takes effect after TF_SaveSettings().
 * The caller must then re-initialise the STM32 UART peripheral to match.
 *
 * Supported rates: 9600, 14400, 19200, 38400, 56000, 57600, 115200,
 *                  128000, 230400, 256000, 460800, 500000 (TF-NOVA),
 *                  512000, 600000 (TF-NOVA), 750000, 921600.
 *
 * @param  htf       Initialised handle.
 * @param  baudrate  Target baud rate.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetBaudRate(TF_Handle_t *htf, uint32_t baudrate);

/**
 * Set distance output limits (TF-NOVA: ID_DIST_RANGE).
 *
 * Measurements outside [min_mm, max_mm] are suppressed / replaced by the
 * sensor's out-of-range value.
 *
 * @param  htf     Initialised handle (TF-NOVA only).
 * @param  min_mm  Minimum reportable distance in mm.
 * @param  max_mm  Maximum reportable distance in mm (max 65535).
 * @return TF_OK or TF_ERR_UNSUPPORTED (if called on TF02-Pro).
 */
TF_Status_t TF_SetDistRange(TF_Handle_t *htf, uint16_t min_mm, uint16_t max_mm);

/**
 * Change the I2C slave address stored on the sensor.
 *
 * Valid range: [0x08, 0x77].  Takes effect after TF_SaveSettings() and reboot.
 * Also updates htf->i2c_addr so subsequent calls use the new address.
 *
 * @param  htf       Initialised handle.
 * @param  new_addr  New 7-bit I2C address.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetI2CAddress(TF_Handle_t *htf, uint8_t new_addr);

/**
 * Save all current settings to non-volatile flash on the sensor.
 *
 * Must be called after any configuration command to make it persistent
 * across power cycles.
 *
 * @param  htf  Initialised handle.
 * @return TF_OK or error.
 */
TF_Status_t TF_SaveSettings(TF_Handle_t *htf);

/**
 * Enable or disable checksum validation on incoming command responses.
 *
 * Disabled by default on both sensors. When disabled the sensor still
 * includes the checksum byte in the response frame but its value is
 * unreliable — do not validate it.
 *
 * Note: This affects command RESPONSE frames only, not the streaming
 * data frames (0x59 0x59) which always carry a valid checksum.
 *
 * @param  htf     Initialised handle.
 * @param  enable  true = enable checksum, false = disable.
 * @return TF_OK or error.
 */
TF_Status_t TF_SetChecksumEnable(TF_Handle_t *htf, bool enable);

/**
 * Restore factory default settings.
 *
 * @param  htf  Initialised handle.
 * @return TF_OK or error.
 */
TF_Status_t TF_RestoreDefaults(TF_Handle_t *htf);

/**
 * Perform a soft reset (reboot) of the sensor.
 *
 * @param  htf  Initialised handle.
 * @return TF_OK or error.
 */
TF_Status_t TF_SoftReset(TF_Handle_t *htf);

/**
 * Switch the TF-NOVA communication interface and save to flash.
 *
 * MUST be called while the sensor is connected via UART (current default).
 * After this function returns TF_OK, you must:
 *   1. Power cycle the sensor
 *   2. Re-initialise using TF_Init_I2C() or TF_Init_I2C_Async()
 *
 * This is a one-time setup — the setting persists across power cycles.
 *
 * @param  htf     Initialised UART handle (TF_SENSOR_TFNOVA only).
 * @param  new_if  TF_IF_UART (0) or TF_IF_I2C (1).
 * @return TF_OK on success.
 *         TF_ERR_UNSUPPORTED if called on TF02-Pro.
 *         TF_ERR_HAL if the current interface is already I2C
 *         (sensor won't respond to this command over I2C).
 */
TF_Status_t TF_SetInterface(TF_Handle_t *htf, TF_Interface_t new_if);

/**
 * Used to register a new sensor handle along with it's data handle onto the I2C bus.
 * @param bus  TF_Bus Handle .
 * @param htf  TF_Handle that is to be registered.
 * @param data TF_data handle to store the data of the TF sensor that is to be registered.
 *
 */
void TF_Bus_Register(TF_Bus_t *bus,
                     TF_Handle_t *htf, TF_Data_t *data);

/**
 * Used to tick(start the reading process) a sensor, call in your while loop.
 *
 * @param  htf  Initialised handle.
 *
 */
void TF_Bus_Tick(TF_Bus_t *bus);


/**
* Place in HAL_I2C_MasterTxCpltCallback(), only place this, do not place individual sensor callbacks.
*/
void TF_Bus_TxCpltCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c);

/**
* Place in HAL_I2C_MasterRxCpltCallback(), only place this, do not place individual sensor callbacks.
* This is responsible for sensor squencing.
*/
void TF_Bus_RxCpltCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c);

/**
* Forward HAL's I2Cx_ER_IRQHandler, only place this, do not place individual sensor callbacks.
* This is "de-activates" the current sensor and moves on to the next sensor.
*/
void TF_Bus_ErrorCallback(TF_Bus_t *bus, I2C_HandleTypeDef *hi2c);


/* ============================================================================
 * ISR integration — call from your USARTx_IRQHandler
 * ========================================================================== */

/**
 * Feed the driver's UART receive state machine from the HAL IRQ callback.
 *
 * Place this call inside HAL_UART_RxCpltCallback():
 *
 * The function restarts single-byte IT reception automatically.
 *
 * @param  htf    Initialised handle.
 * @param  huart  UART handle passed by HAL (used to match against htf->huart).
 */
void TF_UART_RxCpltCallback(TF_Handle_t *htf, UART_HandleTypeDef *huart);







/* ============================================================================
 * Utility
 * ========================================================================== */

/**
 * Return a human-readable string for a TF_Status_t code.
 */
const char *TF_StatusStr(TF_Status_t status);

/**
 * Return the sensor model name string.
 */
const char *TF_SensorName(TF_SensorType_t type);

#endif /* SRC_BENEWAKE_BENEWAKE_TF_H_ */
