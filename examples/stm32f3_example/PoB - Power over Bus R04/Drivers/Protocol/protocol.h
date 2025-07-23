/**
 * @file protocol.h
 * @author Silas Hörz
 * @date Jul 23, 2025
 * @version 1.1.0
 * @brief Defines the application-specific communication protocol layer.
 *
 * This layer builds on top of the universal UART library and implements a
 * master-slave communication logic. It defines commands, roles, and a state
 * machine to manage the connection state.
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "uart.h" // The underlying UART transport layer
#include <stdint.h>

/* Public Defines ------------------------------------------------------------*/
#define PROTOCOL_VERSION_MAJOR 1
#define PROTOCOL_VERSION_MINOR 1 // Version incremented after refactoring
#define PROTOCOL_BAUDRATE 1000000

// Role Definitions
#define PROTOCOL_ROLE_MASTER 0x01
#define PROTOCOL_ROLE_SLAVE  0x02

// Protocol Commands
#define PROTOCOL_CMD_PING          0x01 // Master checks if slave is present
#define PROTOCOL_CMD_PONG          0x02 // Slave acknowledges master's ping
#define PROTOCOL_CMD_REQUEST_STATS 0x10 // Master requests statistics from slave
#define PROTOCOL_CMD_SEND_STATS    0x11 // Slave sends its statistics to master

/* Public Type Definitions ---------------------------------------------------*/

/**
 * @brief States of the protocol's connection state machine.
 */
typedef enum {
    PROTOCOL_STATE_IDLE,             ///< Initial state before role assignment.
    PROTOCOL_STATE_DISCONNECTED,     ///< Slave is waiting for a ping from the master.
    PROTOCOL_STATE_FINDING_SLAVE,    ///< Master is actively searching for a slave by sending pings.
    PROTOCOL_STATE_CONNECTED_MASTER, ///< Master is connected to the slave.
    PROTOCOL_STATE_CONNECTED_SLAVE,  ///< Slave is connected to the master.
    PROTOCOL_STATE_RECONNECTING,     ///< A device has lost connection and is trying to re-establish it.
    PROTOCOL_STATE_ERROR             ///< A critical, non-recoverable error has occurred.
} Protocol_State_t;

/**
 * @brief Structure for the UART statistics transmitted by the slave.
 * @note  The 'packed' attribute ensures byte-exact transmission over UART.
 */
typedef struct __attribute__((packed)) {
    uint32_t packets_received_ok;
    uint32_t errors_total;
    uint32_t error_crc_mismatch;
    uint32_t error_stop_byte_missing;
    uint32_t error_packet_too_long;
    uint32_t error_timeout;
    uint32_t error_uart_overrun;
    uint32_t error_uart_framing;
    uint32_t error_uart_noise;
    uint32_t error_unknown_cmd;
    uint32_t error_tx_busy;
} Protocol_SlaveStats_t;

/**
 * @brief Structure for collecting all relevant debug information of the protocol layer.
 * @note  Ideal for monitoring via "Live Expressions" in a debugger.
 */
typedef struct {
    volatile uint32_t last_cmd_rx;          ///< The last command code received.
    volatile uint32_t last_role_rx;         ///< The role of the sender of the last received packet.
    volatile uint32_t master_ping_count;    ///< Number of pings sent by the master.
    volatile uint32_t slave_pong_count;     ///< Number of pongs sent by the slave.
    volatile uint32_t stats_req_count;      ///< Number of statistics requests sent by the master.
    volatile uint32_t stats_sent_count;     ///< Number of statistics packets sent by the slave.
    volatile uint32_t unexpected_role_errors; ///< Count of packets received from a device with the same role.
    volatile uint32_t unknown_cmd_errors;   ///< Count of received commands that are not defined.
} Protocol_Debug_t;


/* Exported Variables --------------------------------------------------------*/

/** @brief The current state of the protocol state machine. */
extern volatile Protocol_State_t protocol_current_state;

/** @brief A collection of debug statistics for the protocol layer. */
extern volatile Protocol_Debug_t protocol_debug_stats;

/** @brief The most recently received statistics from the slave device. */
extern Protocol_SlaveStats_t latest_slave_stats;

/* Exported Functions --------------------------------------------------------*/

/**
 * @brief Initializes the communication protocol stack.
 * @param huart A pointer to an initialized UART_Handle_t structure. The handle
 * must be configured with the correct peripheral instances (e.g., USART2, DMA1)
 * and DMA channels before calling this function.
 * @param my_device_role The role of this device (PROTOCOL_ROLE_MASTER or PROTOCOL_ROLE_SLAVE).
 */
void PROTOCOL_Init(UART_Handle_t* huart, uint8_t my_device_role);

/**
 * @brief Main processing function of the protocol (State Machine).
 * @note  This function must be called periodically in the application's main loop.
 */
void PROTOCOL_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H_ */
