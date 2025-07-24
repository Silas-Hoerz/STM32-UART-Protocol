/**
 * @file uart.c
 * @author Silas Hörz
 * @date Jul 23, 2025
 * @version 1.1.0
 * @brief Implements the universal, DMA-based UART driver library.
 */

/**
 * All communication uses a fixed frame format to ensure reliable data transfer.
 * The structure is designed for integrity with a clear start/stop, header,
 * payload, and a robust CRC32 checksum.
 *
 * Framed packet structure:
 * `[START] [ROLE] [CMD] [LEN_L] [LEN_H] [DATA...] [CRC32] [STOP]`
 *
 * - **Start Byte (1 Byte):** `0x55`. Marks the beginning of a packet.
 * - **Sender Role (1 Byte):** Defines the sender (e.g., `0x01` Master, `0x02` Slave).
 * - **Command ID (1 Byte):** Identifies the message type.
 * - **Length (2 Bytes):** The 16-bit payload length, transmitted as Little Endian (`LENGTH_L` then `LENGTH_H`).
 * - **Data Payload (0-PKT_MAX_LEN Bytes):** The message content.
 * - **CRC32 Checksum (4 Bytes):** A 32-bit CRC. It's calculated over the `Sender Role`, `Command ID`, `Length` (2 Bytes) and the entire `Data Payload`. Transmitted LSB first.
 * - **Stop Byte (1 Byte):** `0xAA`. Marks the end of a packet.
 */

#include "uart.h"
#include <string.h> // For memcpy, strlen

/* Private variables ---------------------------------------------------------*/

/**
 * @brief Global handle pointers for use in Interrupt Service Routines (ISRs).
 * @note This implementation uses a separate global handle for each possible UART
 * instance. This allows the ISRs (which have fixed names) to access the
 * correct handle. To support more UARTs (e.g., USART1, USART3), you would
 * add more pointers here (e.g., g_huart1_handle) and their corresponding
 * ISRs. The Init function assigns the correct pointer.
 */
static UART_Handle_t* g_huart2_handle = NULL;
// static UART_Handle_t* g_huart1_handle = NULL; // Example for another instance

/* Private function prototypes ---------------------------------------------*/
static void UART_ProcessData(UART_Handle_t* huart, const void *data, size_t len);
static void UART_InternalPacketReceived(UART_Handle_t* huart, uint8_t role, uint8_t cmd, uint8_t *data, uint16_t len);
static void UART_InternalErrorOccurred(UART_Handle_t* huart, UART_ErrorType_t error_type);

/* Function implementations ------------------------------------------------*/

void UART_Init(UART_Handle_t* huart, uint32_t baudrate, UART_PacketReceivedCallback_t pkt_rx_callback, UART_ErrorCallback_t error_callback) {
    // --- 0. Assign global handle for ISR access ---
    if (huart->instance == USART2) {
        g_huart2_handle = huart;
    }
    // else if (huart->instance == USART1) { g_huart1_handle = huart; } // Extend for other UARTs

    // --- 1. Initialize the handle structure ---
    memset(&huart->parser, 0, sizeof(huart->parser));
    memset((void*)&huart->stats, 0, sizeof(huart->stats));
    huart->tx_in_progress = 0;
    huart->pfnPacketReceivedCallback = pkt_rx_callback;
    huart->pfnErrorCallback = error_callback;
    huart->parser.state = PKT_WAIT_START;

    // --- 2. Enable Clocks ---
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA); // GPIO Port for USART2 pins
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

    // --- 3. Configure GPIO Pins (PA2 TX, PA15 RX for USART2) ---
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // --- 4. Configure USART ---
    LL_USART_InitTypeDef USART_InitStruct = {0};
    USART_InitStruct.BaudRate = baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(huart->instance, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(huart->instance);
    LL_USART_Enable(huart->instance);

    // --- 5. Configure DMA Channels ---
    // RX DMA (e.g., DMA1 Channel 6 for USART2_RX)
    LL_DMA_SetDataTransferDirection(huart->dma_instance, huart->rx_dma_channel, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(huart->dma_instance, huart->rx_dma_channel, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(huart->dma_instance, huart->rx_dma_channel, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(huart->dma_instance, huart->rx_dma_channel, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(huart->dma_instance, huart->rx_dma_channel, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(huart->dma_instance, huart->rx_dma_channel, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(huart->dma_instance, huart->rx_dma_channel, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetPeriphAddress(huart->dma_instance, huart->rx_dma_channel, LL_USART_DMA_GetRegAddr(huart->instance, LL_USART_DMA_REG_DATA_RECEIVE));
    LL_DMA_SetMemoryAddress(huart->dma_instance, huart->rx_dma_channel, (uint32_t)huart->rx_buffer);
    LL_DMA_SetDataLength(huart->dma_instance, huart->rx_dma_channel, USART_RX_DMA_BUFFER_SIZE);

    // TX DMA (e.g., DMA1 Channel 7 for USART2_TX)
    LL_DMA_SetDataTransferDirection(huart->dma_instance, huart->tx_dma_channel, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(huart->dma_instance, huart->tx_dma_channel, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(huart->dma_instance, huart->tx_dma_channel, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(huart->dma_instance, huart->tx_dma_channel, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(huart->dma_instance, huart->tx_dma_channel, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(huart->dma_instance, huart->tx_dma_channel, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(huart->dma_instance, huart->tx_dma_channel, LL_DMA_MDATAALIGN_BYTE);

    // --- 6. Configure CRC Unit ---
    LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);
    LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
    LL_CRC_SetPolynomialCoef(CRC, LL_CRC_DEFAULT_CRC32_POLY);
    LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_32B);
    LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);

    // --- 7. Enable DMA Requests & Interrupts ---
    LL_USART_EnableDMAReq_RX(huart->instance);
    LL_USART_EnableDMAReq_TX(huart->instance);
    LL_USART_EnableIT_IDLE(huart->instance);
    LL_USART_EnableIT_ERROR(huart->instance);

    LL_DMA_EnableIT_HT(huart->dma_instance, huart->rx_dma_channel); // Half-transfer interrupt
    LL_DMA_EnableIT_TC(huart->dma_instance, huart->rx_dma_channel); // Transfer-complete interrupt
    LL_DMA_EnableIT_TC(huart->dma_instance, huart->tx_dma_channel); // Transfer-complete interrupt

    // --- 8. Configure NVIC for Interrupts ---
    NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel6_IRQn); // USART2_RX DMA
    NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
    NVIC_EnableIRQ(DMA1_Channel7_IRQn); // USART2_TX DMA
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    // --- 9. Enable DMA Channel to start reception ---
    LL_DMA_EnableChannel(huart->dma_instance, huart->rx_dma_channel);
}

void UART_RxCheck(UART_Handle_t* huart) {
    static size_t old_pos = 0;
    size_t pos;

    pos = USART_RX_DMA_BUFFER_SIZE - LL_DMA_GetDataLength(huart->dma_instance, huart->rx_dma_channel);
    if (pos != old_pos) {
        if (pos > old_pos) {
            // Continuous block of data
            UART_ProcessData(huart, &huart->rx_buffer[old_pos], pos - old_pos);
        } else {
            // Data has wrapped around the circular buffer
            UART_ProcessData(huart, &huart->rx_buffer[old_pos], USART_RX_DMA_BUFFER_SIZE - old_pos);
            if (pos > 0) {
                UART_ProcessData(huart, &huart->rx_buffer[0], pos);
            }
        }
        old_pos = pos;
    }
}



static void UART_ProcessData(UART_Handle_t* huart, const void *data, size_t len) {
    const uint8_t *d = data;
    for (; len > 0; --len, ++d) {
        uint8_t b = *d;
        switch (huart->parser.state) {
            case PKT_WAIT_START:
                if (b == 0x55) {
                    huart->parser.state = PKT_READ_ROLE;
                    LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);
                }
                break;
            case PKT_READ_ROLE:
                huart->parser.role = b;
                LL_CRC_FeedData8(CRC, b);
                huart->parser.state = PKT_READ_CMD;
                break;
            case PKT_READ_CMD:
                huart->parser.cmd = b;
                LL_CRC_FeedData8(CRC, b);
                huart->parser.state = PKT_READ_LEN_L;
                break;
            case PKT_READ_LEN_L:
                huart->parser.len = b;
                LL_CRC_FeedData8(CRC, b);
                huart->parser.state = PKT_READ_LEN_H;
                break;
            case PKT_READ_LEN_H:
                huart->parser.len |= ((uint16_t) b << 8);
                LL_CRC_FeedData8(CRC, b);
                huart->parser.data_pos = 0;
                if (huart->parser.len > PKT_MAX_LEN) {
                    UART_InternalErrorOccurred(huart, UART_ERR_PACKET_TOO_LONG);
                    huart->parser.state = PKT_WAIT_START;
                } else if (huart->parser.len == 0) {
                    huart->parser.state = PKT_READ_CRC_B0;
                } else {
                    huart->parser.state = PKT_READ_DATA;
                }
                break;
            case PKT_READ_DATA:
                huart->parser.data[huart->parser.data_pos++] = b;
                if (huart->parser.data_pos >= huart->parser.len) {
                    for (uint16_t i = 0; i < huart->parser.len; i++) {
                        LL_CRC_FeedData8(CRC, huart->parser.data[i]);
                    }
                    huart->parser.state = PKT_READ_CRC_B0;
                }
                break;
            case PKT_READ_CRC_B0: huart->parser.received_crc = (uint32_t)b; huart->parser.state = PKT_READ_CRC_B1; break;
            case PKT_READ_CRC_B1: huart->parser.received_crc |= ((uint32_t)b << 8); huart->parser.state = PKT_READ_CRC_B2; break;
            case PKT_READ_CRC_B2: huart->parser.received_crc |= ((uint32_t)b << 16); huart->parser.state = PKT_READ_CRC_B3; break;
            case PKT_READ_CRC_B3: huart->parser.received_crc |= ((uint32_t)b << 24); huart->parser.state = PKT_WAIT_STOP; break;
            case PKT_WAIT_STOP:
                if (b == 0xAA) {
                    uint32_t calculated_crc = LL_CRC_ReadData32(CRC);
                    if (calculated_crc == huart->parser.received_crc) {
                        UART_InternalPacketReceived(huart, huart->parser.role, huart->parser.cmd, huart->parser.data, huart->parser.len);
                    } else {
                        UART_InternalErrorOccurred(huart, UART_ERR_CRC_MISMATCH);
                    }
                } else {
                    UART_InternalErrorOccurred(huart, UART_ERR_STOP_BYTE_MISSING);
                }
                huart->parser.state = PKT_WAIT_START;
                break;
        }
    }
}

int8_t UART_SendPacketDMA(UART_Handle_t* huart, uint8_t role_to_send, uint8_t cmd, const uint8_t *data, uint16_t len) {
    if (huart->tx_in_progress) {
        UART_InternalErrorOccurred(huart, UART_ERR_TX_BUSY);
        return -1;
    }
    if (len > PKT_MAX_LEN) {
        UART_InternalErrorOccurred(huart, UART_ERR_PACKET_TOO_LONG);
        return -1;
    }

    huart->tx_in_progress = 1;
    uint16_t current_pos = 0;

    // 1. Build packet header
    huart->tx_buffer[current_pos++] = 0x55; // START
    huart->tx_buffer[current_pos++] = role_to_send;
    huart->tx_buffer[current_pos++] = cmd;
    huart->tx_buffer[current_pos++] = (uint8_t)(len & 0xFF);
    huart->tx_buffer[current_pos++] = (uint8_t)((len >> 8) & 0xFF);

    // 2. Copy payload data
    if (data != NULL && len > 0) {
        memcpy(&huart->tx_buffer[current_pos], data, len);
    }

    // 3. Calculate CRC32 (over role, cmd, length, and data)
    LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);
    LL_CRC_FeedData8(CRC, huart->tx_buffer[1]); // Role
    LL_CRC_FeedData8(CRC, huart->tx_buffer[2]); // Cmd
    LL_CRC_FeedData8(CRC, huart->tx_buffer[3]); // Len_L
    LL_CRC_FeedData8(CRC, huart->tx_buffer[4]); // Len_H
    for (uint16_t i = 0; i < len; i++) {
        LL_CRC_FeedData8(CRC, data[i]);
    }
    uint32_t calculated_crc = LL_CRC_ReadData32(CRC);

    current_pos += len;

    // 4. Append CRC and Stop Byte
    memcpy(&huart->tx_buffer[current_pos], &calculated_crc, 4);
    current_pos += 4;
    huart->tx_buffer[current_pos++] = 0xAA; // STOP

    // 5. Start DMA Transfer
    LL_DMA_SetDataLength(huart->dma_instance, huart->tx_dma_channel, current_pos);
    LL_DMA_SetMemoryAddress(huart->dma_instance, huart->tx_dma_channel, (uint32_t)huart->tx_buffer);
    LL_DMA_SetPeriphAddress(huart->dma_instance, huart->tx_dma_channel, LL_USART_DMA_GetRegAddr(huart->instance, LL_USART_DMA_REG_DATA_TRANSMIT));
    LL_DMA_EnableChannel(huart->dma_instance, huart->tx_dma_channel);

    return 0;
}

void UART_SendStringBlocking(UART_Handle_t* huart, const char *str) {
    const uint8_t *d = (const uint8_t *)str;
    size_t len = strlen(str);
    for (; len > 0; --len, ++d) {
        LL_USART_TransmitData8(huart->instance, *d);
        while (!LL_USART_IsActiveFlag_TXE(huart->instance)) {}
    }
    while (!LL_USART_IsActiveFlag_TC(huart->instance)) {}
}

static void UART_InternalPacketReceived(UART_Handle_t* huart, uint8_t role, uint8_t cmd, uint8_t *data, uint16_t len) {
    huart->stats.packets_received_ok++;
    if (huart->pfnPacketReceivedCallback != NULL) {
        huart->pfnPacketReceivedCallback(huart, role, cmd, data, len);
    }
}

static void UART_InternalErrorOccurred(UART_Handle_t* huart, UART_ErrorType_t error_type) {
    huart->stats.errors_total++;
    switch (error_type) {
        case UART_ERR_CRC_MISMATCH: huart->stats.error_crc_mismatch++; break;
        case UART_ERR_STOP_BYTE_MISSING: huart->stats.error_stop_byte_missing++; break;
        case UART_ERR_PACKET_TOO_LONG: huart->stats.error_packet_too_long++; break;
        case UART_ERR_TIMEOUT: huart->stats.error_timeout++; break;
        case UART_ERR_UNEXPECTED_ROLE: huart->stats.error_unexpected_role++; break;
        case UART_ERR_OVERRUN: huart->stats.error_uart_overrun++; break;
        case UART_ERR_FRAMING: huart->stats.error_uart_framing++; break;
        case UART_ERR_NOISE: huart->stats.error_uart_noise++; break;
        case UART_ERR_UNKNOWN_CMD: huart->stats.error_unknown_cmd++; break;
        case UART_ERR_TX_BUSY: huart->stats.error_tx_busy++; break;
        default: break;
    }
    if (huart->pfnErrorCallback != NULL) {
        huart->pfnErrorCallback(huart, error_type);
    }
}

/* Interrupt Handlers --------------------------------------------------------*/

/**
 * @brief ISR for DMA Transfer Complete on USART2_TX (DMA1 Channel 7).
 */
void DMA1_Channel7_IRQHandler(void) {
    if (g_huart2_handle && LL_DMA_IsActiveFlag_TC7(g_huart2_handle->dma_instance)) {
        LL_DMA_ClearFlag_TC7(g_huart2_handle->dma_instance);
        LL_DMA_DisableChannel(g_huart2_handle->dma_instance, g_huart2_handle->tx_dma_channel);
        while(!LL_USART_IsActiveFlag_TC(g_huart2_handle->instance)) {} // Wait for last byte to be sent
        g_huart2_handle->tx_in_progress = 0;
    }
}

/**
 * @brief ISR for DMA Half Transfer and Transfer Complete on USART2_RX (DMA1 Channel 6).
 */
void DMA1_Channel6_IRQHandler(void) {
    // Half Transfer Complete
    if (g_huart2_handle && LL_DMA_IsActiveFlag_HT6(g_huart2_handle->dma_instance)) {
        LL_DMA_ClearFlag_HT6(g_huart2_handle->dma_instance);
        UART_RxCheck(g_huart2_handle);
    }
    // Transfer Complete
    if (g_huart2_handle && LL_DMA_IsActiveFlag_TC6(g_huart2_handle->dma_instance)) {
        LL_DMA_ClearFlag_TC6(g_huart2_handle->dma_instance);
        UART_RxCheck(g_huart2_handle);
    }
}

/**
 * @brief ISR for USART2 global interrupts (IDLE line, errors).
 */
void USART2_IRQHandler(void) {
    if (!g_huart2_handle) return;

    // IDLE Line Detection: indicates the end of a burst of data.
    if (LL_USART_IsActiveFlag_IDLE(g_huart2_handle->instance)) {
        LL_USART_ClearFlag_IDLE(g_huart2_handle->instance);
        UART_RxCheck(g_huart2_handle);
        // If the parser is not in the initial state, it means a packet was incomplete.
        if (g_huart2_handle->parser.state != PKT_WAIT_START) {
            UART_InternalErrorOccurred(g_huart2_handle, UART_ERR_TIMEOUT);
            g_huart2_handle->parser.state = PKT_WAIT_START;
        }
    }

    // Overrun Error
    if (LL_USART_IsActiveFlag_ORE(g_huart2_handle->instance)) {
        LL_USART_ClearFlag_ORE(g_huart2_handle->instance);
        UART_InternalErrorOccurred(g_huart2_handle, UART_ERR_OVERRUN);
        g_huart2_handle->parser.state = PKT_WAIT_START;
    }

    // Framing Error
    if (LL_USART_IsActiveFlag_FE(g_huart2_handle->instance)) {
        LL_USART_ClearFlag_FE(g_huart2_handle->instance);
        UART_InternalErrorOccurred(g_huart2_handle, UART_ERR_FRAMING);
        g_huart2_handle->parser.state = PKT_WAIT_START;
    }

    // Noise Error
    if (LL_USART_IsActiveFlag_NE(g_huart2_handle->instance)) {
        LL_USART_ClearFlag_NE(g_huart2_handle->instance);
        UART_InternalErrorOccurred(g_huart2_handle, UART_ERR_NOISE);
        g_huart2_handle->parser.state = PKT_WAIT_START;
    }
}
