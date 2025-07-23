/**
 * @file uart.h
 * @author Silas Hörz
 * @date Jul 23, 2025
 * @version 1.1.0
 * @brief Universal, DMA-based UART driver library for STM32F3xx.
 *
 * This library provides robust, interrupt- and DMA-driven UART communication.
 * It is designed as a lower-level transport layer, responsible for pure
 * data transmission, packet framing, and CRC32 integrity checking.
 * The library is made instantiable through the use of a UART_Handle_t structure.
 */

#ifndef UART_H_
#define UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_ll_usart.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_crc.h"
#include <stdint.h>
#include <stddef.h> // For size_t

/* Public Defines ------------------------------------------------------------*/

/** @brief Size of the circular DMA buffer for receiving data. */
#define USART_RX_DMA_BUFFER_SIZE 1024

/** @brief Maximum length of the payload data in a single packet. */
#define PKT_MAX_LEN 600

/** @brief Total size of the transmission buffer. Header(5) + MaxData(600) + CRC(4) */
#define PKT_TX_BUFFER_SIZE (PKT_MAX_LEN + 9)

/* Public Type Definitions ---------------------------------------------------*/

/**
 * @brief States for the incoming packet parser's state machine.
 */
typedef enum {
    PKT_WAIT_START,    ///< Waiting for the start byte (0x55)
    PKT_READ_ROLE,     ///< Reading the sender's role byte
    PKT_READ_CMD,      ///< Reading the command byte
    PKT_READ_LEN_L,    ///< Reading the low byte of the data length
    PKT_READ_LEN_H,    ///< Reading the high byte of the data length
    PKT_READ_DATA,     ///< Reading the payload data
    PKT_READ_CRC_B0,   ///< Reading the first byte of the CRC
    PKT_READ_CRC_B1,   ///< Reading the second byte of the CRC
    PKT_READ_CRC_B2,   ///< Reading the third byte of the CRC
    PKT_READ_CRC_B3,   ///< Reading the fourth byte of the CRC
    PKT_WAIT_STOP      ///< Waiting for the stop byte (0xAA)
} UART_PktState_t;

/**
 * @brief Enumeration of all possible UART-related errors.
 */
typedef enum {
    UART_ERR_NONE = 0,
    UART_ERR_CRC_MISMATCH,      ///< Calculated CRC does not match the received CRC.
    UART_ERR_STOP_BYTE_MISSING, ///< The packet's stop byte was not found where expected.
    UART_ERR_PACKET_TOO_LONG,   ///< The packet's declared length exceeds the buffer size.
    UART_ERR_TIMEOUT,           ///< A timeout occurred during packet reception (e.g., incomplete packet).
    UART_ERR_UNEXPECTED_ROLE,   ///< Protocol-level error: packet received from an invalid role.
    UART_ERR_OVERRUN,           ///< UART hardware overrun error.
    UART_ERR_FRAMING,           ///< UART hardware framing error.
    UART_ERR_NOISE,             ///< UART hardware noise detection error.
    UART_ERR_UNKNOWN_CMD,       ///< Protocol-level error: an unknown command was received.
    UART_ERR_TX_BUSY            ///< Transmission attempt failed because a transfer was already in progress.
} UART_ErrorType_t;

/**
 * @brief Structure for tracking communication statistics and errors.
 * @note  Ideal for monitoring via a "Live Expressions" window in a debugger.
 */
typedef struct {
    uint32_t packets_received_ok;
    uint32_t errors_total;
    uint32_t error_crc_mismatch;
    uint32_t error_stop_byte_missing;
    uint32_t error_packet_too_long;
    uint32_t error_timeout;
    uint32_t error_unexpected_role;
    uint32_t error_uart_overrun;
    uint32_t error_uart_framing;
    uint32_t error_uart_noise;
    uint32_t error_unknown_cmd;
    uint32_t error_tx_busy;
} UART_CommStats_t;


/**
 * @brief Structure to hold the state of the packet parser.
 */
typedef struct {
    UART_PktState_t state;     ///< Current state of the parser state machine.
    uint8_t         role;      ///< Role of the sender from the current packet.
    uint8_t         cmd;       ///< Command from the current packet.
    uint16_t        len;       ///< Length of data in the current packet.
    uint8_t         data[PKT_MAX_LEN]; ///< Buffer for the packet's data.
    uint16_t        data_pos;  ///< Current position in the data buffer.
    uint32_t        received_crc; ///< CRC value received in the packet.
    volatile uint64_t last_pong_received_time; ///< Timestamp of the last PONG, used for master timeout.
} UART_PktParser_t;


// Forward declaration of the handle structure.
struct UART_Handle;

/**
 * @brief Function pointer type for the packet reception callback.
 * @param huart Pointer to the UART handle that received the packet.
 * @param role The role of the sender of the packet.
 * @param cmd The command of the packet.
 * @param data Pointer to the payload data.
 * @param len Length of the payload data.
 */
typedef void (*UART_PacketReceivedCallback_t)(struct UART_Handle* huart, uint8_t role, uint8_t cmd, uint8_t *data, uint16_t len);

/**
 * @brief Function pointer type for the error callback.
 * @param huart Pointer to the UART handle where the error occurred.
 * @param error_type The type of error that occurred.
 */
typedef void (*UART_ErrorCallback_t)(struct UART_Handle* huart, UART_ErrorType_t error_type);

/**
 * @brief The main handle structure that bundles all data for one UART instance.
 * @note  This structure must be initialized by the user application before being
 * passed to the driver.
 */
typedef struct UART_Handle {
    // --- Hardware Configuration (must be set by user before calling UART_Init) ---
    USART_TypeDef* instance;       ///< Peripheral instance (e.g., USART2).
    DMA_TypeDef* dma_instance;   ///< DMA instance (e.g., DMA1).
    uint32_t                        rx_dma_channel; ///< RX DMA Channel (e.g., LL_DMA_CHANNEL_6).
    uint32_t                        tx_dma_channel; ///< TX DMA Channel (e.g., LL_DMA_CHANNEL_7).

    // --- Driver Internal State (managed by the driver) ---
    UART_PktParser_t                parser;         ///< The packet parser state.
    volatile UART_CommStats_t       stats;          ///< Communication statistics.
    volatile uint8_t                tx_in_progress; ///< Flag indicating if a TX DMA transfer is active.

    // --- Callbacks (set by user via UART_Init) ---
    UART_PacketReceivedCallback_t   pfnPacketReceivedCallback; ///< Callback for successfully received packets.
    UART_ErrorCallback_t            pfnErrorCallback;          ///< Callback for errors.

    // --- Buffers (managed by the driver) ---
    uint8_t                         rx_buffer[USART_RX_DMA_BUFFER_SIZE]; ///< Circular buffer for DMA reception.
    uint8_t                         tx_buffer[PKT_TX_BUFFER_SIZE];     ///< Buffer for DMA transmission.
} UART_Handle_t;

/* Exported Functions --------------------------------------------------------*/

/**
 * @brief Initializes a UART instance with DMA, CRC, and all necessary peripherals.
 * @param huart Pointer to the UART_Handle_t structure to be configured. This handle
 * MUST have its hardware configuration members (instance, dma_instance,
 * rx_dma_channel, tx_dma_channel) pre-filled.
 * @param baudrate The desired baud rate (e.g., 1000000).
 * @param pkt_rx_callback Pointer to the callback function for successfully received packets.
 * @param error_callback Pointer to the callback function for errors.
 */
void UART_Init(UART_Handle_t* huart, uint32_t baudrate, UART_PacketReceivedCallback_t pkt_rx_callback, UART_ErrorCallback_t error_callback);

/**
 * @brief Sends a formatted packet via UART using DMA.
 * @param huart Pointer to the UART handle.
 * @param role_to_send The role of this device, to be written into the packet header.
 * @param cmd The command of the packet.
 * @param data Pointer to the payload data. Can be NULL if len is 0.
 * @param len Length of the payload data.
 * @retval 0 on success, -1 if TX is busy or the packet is too large.
 */
int8_t UART_SendPacketDMA(UART_Handle_t* huart, uint8_t role_to_send, uint8_t cmd, const uint8_t *data, uint16_t len);

/**
 * @brief Processes newly received data from the circular DMA buffer.
 * @note  This function is intended for internal use and is called from interrupt handlers.
 * @param huart Pointer to the UART handle.
 */
void UART_RxCheck(UART_Handle_t* huart);

/**
 * @brief Sends a string blockingly over UART (for simple debugging).
 * @param huart Pointer to the UART handle.
 * @param str The null-terminated string to send.
 */
void UART_SendStringBlocking(UART_Handle_t* huart, const char *str);


#ifdef __cplusplus
}
#endif

#endif /* UART_H_ */
