/**
 * @file protocol.c
 * @author Silas Hörz
 * @date Jul 23, 2025
 * @version 1.1.0
 * @brief Implements the application-specific communication protocol layer.
 */

#include "protocol.h"
#include <string.h> // For memcpy

/* Private Defines -----------------------------------------------------------*/
#define MASTER_PING_INTERVAL_MS         100  // Master sends a ping every 100ms when searching.
#define MASTER_PONG_TIMEOUT_MS          100  // Master waits max 100ms for a pong before reconnecting.
#define MASTER_STATS_REQUEST_INTERVAL_MS 20  // Master requests stats every 20ms when connected.
#define SLAVE_MASTER_TIMEOUT_MS         1000 // Slave waits max 1s for a master ping before disconnecting.

/* Private Variables ---------------------------------------------------------*/
static UART_Handle_t *g_huart = NULL; // Local pointer to the UART handle used by this layer.
static uint8_t my_current_role = 0;   // The role (Master/Slave) of this device.

// Exported variable definitions
volatile Protocol_State_t protocol_current_state = PROTOCOL_STATE_IDLE;
volatile Protocol_Debug_t protocol_debug_stats = {0};
Protocol_SlaveStats_t latest_slave_stats = {0};

// Master-specific timers
static uint32_t master_last_ping_time_ms = 0;
static uint32_t master_last_stats_request_time_ms = 0;

// Slave-specific timers
static uint32_t slave_last_master_ping_time_ms = 0;

/* Private Function Prototypes -----------------------------------------------*/
// State machine logic functions
static void Protocol_MasterLogic(uint32_t current_time_ms);
static void Protocol_SlaveLogic(uint32_t current_time_ms);
static void Protocol_ResetConnectionState(void);

// Handler functions for the command table
static void Protocol_HandlePing(uint8_t sender_role, uint8_t *data, uint16_t len);
static void Protocol_HandlePong(uint8_t sender_role, uint8_t *data, uint16_t len);
static void Protocol_HandleRequestStats(uint8_t sender_role, uint8_t *data, uint16_t len);
static void Protocol_HandleSendStats(uint8_t sender_role, uint8_t *data, uint16_t len);
// Add prototypes for new custom command handlers here.

// Callbacks for the lower UART layer
static void PROTOCOL_OnPacketReceived(UART_Handle_t *huart, uint8_t role, uint8_t cmd, uint8_t *data, uint16_t len);
static void PROTOCOL_OnErrorOccurred(UART_Handle_t *huart, UART_ErrorType_t error_type);

/* Command Table -------------------------------------------------------------*/
/**
 * @brief Function pointer type for a command handler.
 */
typedef void (*Protocol_CmdHandler_t)(uint8_t sender_role, uint8_t *data, uint16_t len);

/**
 * @brief Associates a command ID with its handler function.
 */
typedef struct {
    const uint8_t cmd;
    const Protocol_CmdHandler_t handler;
} Protocol_Cmd_t;

/**
 * @brief The command table.
 * @note  To add a new command to the protocol:
 * 1. Add a PROTOCOL_CMD_... definition in protocol.h.
 * 2. Implement a corresponding Protocol_Handle...() function in this file.
 * 3. Add an entry for the new command in this table.
 */
static const Protocol_Cmd_t protocol_command_table[] = {
    {PROTOCOL_CMD_PING, Protocol_HandlePing},
    {PROTOCOL_CMD_PONG, Protocol_HandlePong},
    {PROTOCOL_CMD_REQUEST_STATS, Protocol_HandleRequestStats},
    {PROTOCOL_CMD_SEND_STATS, Protocol_HandleSendStats},
    // Add new command entries here.
};

/* Exported Function Implementations -----------------------------------------*/

void PROTOCOL_Init(UART_Handle_t *huart, uint8_t my_device_role) {
    g_huart = huart;
    my_current_role = my_device_role;

    // Initialize the lower UART layer, passing our callback functions.
    // The UART driver will call these functions upon packet reception or error.
    UART_Init(g_huart, PROTOCOL_BAUDRATE, PROTOCOL_OnPacketReceived, PROTOCOL_OnErrorOccurred);

    // Set the initial state based on the device's role.
    Protocol_ResetConnectionState();
}

void PROTOCOL_Process(void) {
    // HAL_GetTick() provides a millisecond time base from the SysTick timer.
    // It is a standard part of the STM32 HAL/LL library.
    uint32_t current_time_ms = HAL_GetTick();

    if (my_current_role == PROTOCOL_ROLE_MASTER) {
        Protocol_MasterLogic(current_time_ms);
    } else if (my_current_role == PROTOCOL_ROLE_SLAVE) {
        Protocol_SlaveLogic(current_time_ms);
    }
}

/* Callback Implementations --------------------------------------------------*/

/**
 * @brief Callback executed by the UART layer when a complete packet is received.
 */
static void PROTOCOL_OnPacketReceived(UART_Handle_t *huart, uint8_t sender_role, uint8_t cmd, uint8_t *data, uint16_t len) {
    protocol_debug_stats.last_cmd_rx = cmd;
    protocol_debug_stats.last_role_rx = sender_role;

    // A device should not receive a packet from a device with the same role.
    if (my_current_role == sender_role) {
        protocol_debug_stats.unexpected_role_errors++;
        PROTOCOL_OnErrorOccurred(huart, UART_ERR_UNEXPECTED_ROLE);
        return;
    }

    // Find and call the appropriate handler from the command table.
    for (size_t i = 0; i < (sizeof(protocol_command_table) / sizeof(protocol_command_table[0])); i++) {
        if (protocol_command_table[i].cmd == cmd) {
            protocol_command_table[i].handler(sender_role, data, len);
            return;
        }
    }

    // If no handler is found, it's an unknown command.
    protocol_debug_stats.unknown_cmd_errors++;
    PROTOCOL_OnErrorOccurred(huart, UART_ERR_UNKNOWN_CMD);
}

/**
 * @brief Callback executed by the UART layer when a hardware or parsing error occurs.
 */
static void PROTOCOL_OnErrorOccurred(UART_Handle_t *huart, UART_ErrorType_t error_type) {
    switch (error_type) {
    // For critical errors that cause desynchronization, attempt to reconnect.
    case UART_ERR_TIMEOUT:
    case UART_ERR_OVERRUN:
        if (protocol_current_state == PROTOCOL_STATE_CONNECTED_MASTER ||
            protocol_current_state == PROTOCOL_STATE_CONNECTED_SLAVE) {
            protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        }
        break;
    // An unexpected role is a critical protocol error.
    case UART_ERR_UNEXPECTED_ROLE:
        protocol_current_state = PROTOCOL_STATE_ERROR;
        break;
    // Other errors are counted by the UART layer but may not require a state change here.
    default:
        break;
    }
}

/* Private Function Implementations ------------------------------------------*/

/**
 * @brief Resets the protocol state machine to its initial state.
 */
static void Protocol_ResetConnectionState(void) {
    if (my_current_role == PROTOCOL_ROLE_MASTER) {
        protocol_current_state = PROTOCOL_STATE_FINDING_SLAVE;
        // Set last ping time to ensure a ping is sent immediately.
        master_last_ping_time_ms = HAL_GetTick() - MASTER_PING_INTERVAL_MS;
    } else if (my_current_role == PROTOCOL_ROLE_SLAVE) {
        protocol_current_state = PROTOCOL_STATE_DISCONNECTED;
        slave_last_master_ping_time_ms = 0;
    }
    // Optionally, reset UART statistics as well.
    // memset((void*)&g_huart->stats, 0, sizeof(UART_CommStats_t));
}

/**
 * @brief Implements the state machine logic for the Master role.
 */
static void Protocol_MasterLogic(uint32_t current_time_ms) {
    switch (protocol_current_state) {
    case PROTOCOL_STATE_IDLE:
    case PROTOCOL_STATE_RECONNECTING:
        Protocol_ResetConnectionState();
        break;

    case PROTOCOL_STATE_FINDING_SLAVE:
        // Periodically send PING packets to find a slave.
        if ((current_time_ms - master_last_ping_time_ms) >= MASTER_PING_INTERVAL_MS) {
            UART_SendPacketDMA(g_huart, my_current_role, PROTOCOL_CMD_PING, NULL, 0);
            master_last_ping_time_ms = current_time_ms;
            protocol_debug_stats.master_ping_count++;
        }
        // Transition to CONNECTED state happens in the PONG handler.
        break;

    case PROTOCOL_STATE_CONNECTED_MASTER:
        // Check for pong timeout. If no pong is received, assume disconnection.
        // The last_pong_received_time is updated in the PONG handler.
        if ((current_time_ms - g_huart->parser.last_pong_received_time) > MASTER_PONG_TIMEOUT_MS) {
            protocol_current_state = PROTOCOL_STATE_RECONNECTING;
            break;
        }
        // Periodically request statistics from the slave.
        if ((current_time_ms - master_last_stats_request_time_ms) >= MASTER_STATS_REQUEST_INTERVAL_MS) {
            UART_SendPacketDMA(g_huart, my_current_role, PROTOCOL_CMD_REQUEST_STATS, NULL, 0);
            master_last_stats_request_time_ms = current_time_ms;
            protocol_debug_stats.stats_req_count++;
        }
        break;

    case PROTOCOL_STATE_ERROR:
        // If a critical error occurs, try to reconnect.
        protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        break;

    default:
        // For any other unexpected state, default to reconnecting.
        protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        break;
    }
}

/**
 * @brief Implements the state machine logic for the Slave role.
 */
static void Protocol_SlaveLogic(uint32_t current_time_ms) {
    switch (protocol_current_state) {
    case PROTOCOL_STATE_IDLE:
    case PROTOCOL_STATE_RECONNECTING:
        Protocol_ResetConnectionState();
        break;

    case PROTOCOL_STATE_CONNECTED_SLAVE:
        // If no ping is received from the master for a certain time, assume disconnection.
        if ((current_time_ms - slave_last_master_ping_time_ms) > SLAVE_MASTER_TIMEOUT_MS) {
            protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        }
        break;

    case PROTOCOL_STATE_DISCONNECTED:
        // In the DISCONNECTED state, the slave passively waits for a PING.
        // The state transition to CONNECTED happens within the PING handler.
        break;

    case PROTOCOL_STATE_ERROR:
        protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        break;

    default:
        protocol_current_state = PROTOCOL_STATE_RECONNECTING;
        break;
    }
}

/* Command Handler Implementations -------------------------------------------*/
// This section contains the functions that execute specific commands.
// You can add your own custom command handlers here.

/**
 * @brief Handles an incoming PING command.
 */
static void Protocol_HandlePing(uint8_t sender_role, uint8_t *data, uint16_t len) {
    if (my_current_role == PROTOCOL_ROLE_SLAVE && sender_role == PROTOCOL_ROLE_MASTER) {
        slave_last_master_ping_time_ms = HAL_GetTick(); // Update timeout timer
        UART_SendPacketDMA(g_huart, my_current_role, PROTOCOL_CMD_PONG, NULL, 0);
        protocol_debug_stats.slave_pong_count++;
        // If not already connected, change state to connected.
        if (protocol_current_state != PROTOCOL_STATE_CONNECTED_SLAVE) {
            protocol_current_state = PROTOCOL_STATE_CONNECTED_SLAVE;
        }
    }
}

/**
 * @brief Handles an incoming PONG command.
 */
static void Protocol_HandlePong(uint8_t sender_role, uint8_t *data, uint16_t len) {
    if (my_current_role == PROTOCOL_ROLE_MASTER && sender_role == PROTOCOL_ROLE_SLAVE) {
        // The timestamp is updated directly in the UART handle for timeout checks.
        g_huart->parser.last_pong_received_time = HAL_GetTick();
        // If we were searching, we are now connected.
        if (protocol_current_state != PROTOCOL_STATE_CONNECTED_MASTER) {
            protocol_current_state = PROTOCOL_STATE_CONNECTED_MASTER;
            // Trigger a statistics request immediately upon connection.
            master_last_stats_request_time_ms = HAL_GetTick() - MASTER_STATS_REQUEST_INTERVAL_MS;
        }
    }
}

/**
 * @brief Handles a REQUEST_STATS command.
 */
static void Protocol_HandleRequestStats(uint8_t sender_role, uint8_t *data, uint16_t len) {
    if (my_current_role == PROTOCOL_ROLE_SLAVE && sender_role == PROTOCOL_ROLE_MASTER) {
        Protocol_SlaveStats_t my_stats;
        // Copy the relevant UART statistics from the low-level driver.
        memcpy(&my_stats, (void*)&g_huart->stats, sizeof(Protocol_SlaveStats_t));

        UART_SendPacketDMA(g_huart, my_current_role, PROTOCOL_CMD_SEND_STATS, (uint8_t*)&my_stats, sizeof(my_stats));
        protocol_debug_stats.stats_sent_count++;
    }
}

/**
 * @brief Handles a SEND_STATS command.
 */
static void Protocol_HandleSendStats(uint8_t sender_role, uint8_t *data, uint16_t len) {
    if (my_current_role == PROTOCOL_ROLE_MASTER && sender_role == PROTOCOL_ROLE_SLAVE) {
        // Ensure the received data has the correct size before copying.
        if (len == sizeof(Protocol_SlaveStats_t)) {
            memcpy(&latest_slave_stats, data, len);
        }
    }
}
