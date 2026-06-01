

### H7_FDCAN_init

```c
H7_state_e H7_FDCAN_init(H7_FDCANHandler_s *fdcan, u32 STD_Filters_Bank, u32 EXD_Filters_Bank, u32 rxFIFO0Elmnts, u32 rxFIFO1Elmnts, u32 txFIFOElmnts, u32 txFIFO_operation_mode, u32 FIFOBuffer, H7_fdcan_baudrate_e data_rate, H7_fdcan_data_size size);

```
- Used for initializing FDCAN peripheral in adapter.c file.
- Arguments
	- *fdcan*: [H7Lib1.0](../H7Lib1.0.md) handler for FDCAN peripheral, structures already declared in [Adapter](../Adapter.md) module.
	- *STD_Filters_Bank* : Specifies how many standard filters will be used.
	- *EXD_Filters_Bank* : Specifies how many extended filters will be used.
	- *rxFIFO0Elmnts* : FIFO0 buffer size. How many messages are stored before they are cleared to receive other messages. [Determining Elements number](FDCAN.md#determining-elements-number).
	- *rxFIFO1Elmnts* :FIFO1 buffer size. [Determining Elements number](FDCAN.md#determining-elements-number).
	- *txFIFOElmnts* : Transmit buffer FIFO, same concept as rx_FIFO, for more details refer to [Determining Elements number](FDCAN.md#determining-elements-number).
	- *txFIFO_operation_mode* 
		- There are two values
		  ```c
			/** @defgroup FDCAN_txFifoQueue_Mode FDCAN Tx FIFO/Queue Mode
			* @{ from stm32h7xx_hal_fdcan.h
			*/
			#define FDCAN_TX_FIFO_OPERATION ((uint32_t)0x00000000U) 
			#define FDCAN_TX_QUEUE_OPERATION ((uint32_t)FDCAN_TXBC_TFQM) 
		  ```
		  - *FDCAN_TX_FIFO_OPERATION*: Messages are transmitted as FIFO order.
		  - *FDCAN_TX_QUEUE_OPERATION* : Messages are transmitted depending priority, lower ID have the highest priority.

	- *FIFOBuffer*: Specify which interrupt to activate.
		- There are many macros for that, but in our application is reception through FIFO buffer.
		- ```c
		  #define FDCAN_IT_RX_FIFO0_NEW_MESSAGE FDCAN_IE_RF0NE
		  #define FDCAN_IT_RX_FIFO1_NEW_MESSAGE FDCAN_IE_RF1NE
		  ```
		- High-priority messages → RX FIFO 0
		- Low-priority messages → RX FIFO 1

	- *data_rate*: Speed of FDCAN port, [H7Lib1.0](../H7Lib1.0.md) version supports:
		- ```c
			typedef enum{
				FDCAN_500kbps,
				FDCAN_1Mbps
			} H7_fdcan_baudrate_e;
		  ```
	- *size* : Size of the data, should be a member of @H7_fdcan_data_size enum.
		- ```c
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
		  ```
		- *H7_FDCAN_8_BYTES* is the default, unless larger size is supported in future [Versions](../H7Lib1.0.md#versions).
