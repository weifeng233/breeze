/**
 * @file uart.h
 * @brief UART communication protocol implementation for Breeze Framework
 *
 * This header provides UART (Universal Asynchronous Receiver-Transmitter)
 * communication protocol implementation with configurable parameters,
 * error handling, and timeout mechanisms.
 */

#ifndef BREEZE_COMM_UART_H
#define BREEZE_COMM_UART_H

#include "comm_interface.h"
#include "comm_hal.h"
#include "comm_buffer.h"
#include "../core/error_codes.h"
#include "../core/config.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART parity settings
 */
typedef enum {
    BREEZE_UART_PARITY_NONE = 0,    /**< No parity */
    BREEZE_UART_PARITY_ODD = 1,     /**< Odd parity */
    BREEZE_UART_PARITY_EVEN = 2     /**< Even parity */
} BreezeUartParity;

/**
 * @brief UART stop bits settings
 */
typedef enum {
    BREEZE_UART_STOP_BITS_1 = 1,    /**< 1 stop bit */
    BREEZE_UART_STOP_BITS_2 = 2     /**< 2 stop bits */
} BreezeUartStopBits;

/**
 * @brief UART data bits settings
 */
typedef enum {
    BREEZE_UART_DATA_BITS_5 = 5,    /**< 5 data bits */
    BREEZE_UART_DATA_BITS_6 = 6,    /**< 6 data bits */
    BREEZE_UART_DATA_BITS_7 = 7,    /**< 7 data bits */
    BREEZE_UART_DATA_BITS_8 = 8     /**< 8 data bits */
} BreezeUartDataBits;

/**
 * @brief UART flow control settings
 */
typedef enum {
    BREEZE_UART_FLOW_CONTROL_NONE = 0,      /**< No flow control */
    BREEZE_UART_FLOW_CONTROL_HARDWARE = 1,  /**< Hardware flow control (RTS/CTS) */
    BREEZE_UART_FLOW_CONTROL_SOFTWARE = 2   /**< Software flow control (XON/XOFF) */
} BreezeUartFlowControl;

/**
 * @brief UART configuration structure
 */
typedef struct {
    uint32_t baudrate;                      /**< Baudrate (e.g., 9600, 115200) */
    BreezeUartDataBits data_bits;           /**< Number of data bits */
    BreezeUartStopBits stop_bits;           /**< Number of stop bits */
    BreezeUartParity parity;                /**< Parity setting */
    BreezeUartFlowControl flow_control;     /**< Flow control setting */
    uint32_t timeout_ms;                    /**< Timeout in milliseconds */
    size_t tx_buffer_size;                  /**< Transmit buffer size */
    size_t rx_buffer_size;                  /**< Receive buffer size */
    void* port_handle;                      /**< Platform-specific port handle */
} BreezeUartConfig;

/**
 * @brief UART instance structure
 */
typedef struct {
    BreezeCommInterface comm_interface;     /**< Base communication interface */
    BreezeUartConfig config;                /**< UART configuration */
    uint8_t* tx_buffer_data;                /**< Transmit buffer data */
    uint8_t* rx_buffer_data;                /**< Receive buffer data */
    volatile int tx_in_progress;            /**< Transmission in progress flag */
    volatile int rx_in_progress;            /**< Reception in progress flag */
    uint32_t last_activity_time;            /**< Last activity timestamp */
} BreezeUartInstance;

/**
 * @brief Default UART configuration
 */
#define BREEZE_UART_CONFIG_DEFAULT { \
    .baudrate = 115200, \
    .data_bits = BREEZE_UART_DATA_BITS_8, \
    .stop_bits = BREEZE_UART_STOP_BITS_1, \
    .parity = BREEZE_UART_PARITY_NONE, \
    .flow_control = BREEZE_UART_FLOW_CONTROL_NONE, \
    .timeout_ms = 1000, \
    .tx_buffer_size = 256, \
    .rx_buffer_size = 256, \
    .port_handle = NULL \
}

/**
 * @brief Common baudrate definitions
 */
#define BREEZE_UART_BAUDRATE_9600    9600
#define BREEZE_UART_BAUDRATE_19200   19200
#define BREEZE_UART_BAUDRATE_38400   38400
#define BREEZE_UART_BAUDRATE_57600   57600
#define BREEZE_UART_BAUDRATE_115200  115200
#define BREEZE_UART_BAUDRATE_230400  230400
#define BREEZE_UART_BAUDRATE_460800  460800
#define BREEZE_UART_BAUDRATE_921600  921600

/**
 * @brief Initialize UART instance
 * @param uart Pointer to UART instance
 * @param config Pointer to UART configuration
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_init(BreezeUartInstance* uart, const BreezeUartConfig* config);

/**
 * @brief Deinitialize UART instance
 * @param uart Pointer to UART instance
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_deinit(BreezeUartInstance* uart);

/**
 * @brief Send data via UART
 * @param uart Pointer to UART instance
 * @param data Pointer to data to send
 * @param length Length of data to send
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_send(BreezeUartInstance* uart, const uint8_t* data, size_t length);

/**
 * @brief Receive data from UART
 * @param uart Pointer to UART instance
 * @param data Pointer to buffer for received data
 * @param length Pointer to length (input: buffer size, output: received length)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_receive(BreezeUartInstance* uart, uint8_t* data, size_t* length);

/**
 * @brief Send data via UART with timeout
 * @param uart Pointer to UART instance
 * @param data Pointer to data to send
 * @param length Length of data to send
 * @param timeout_ms Timeout in milliseconds
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_send_timeout(BreezeUartInstance* uart, 
                                        const uint8_t* data, 
                                        size_t length, 
                                        uint32_t timeout_ms);

/**
 * @brief Receive data from UART with timeout
 * @param uart Pointer to UART instance
 * @param data Pointer to buffer for received data
 * @param length Pointer to length (input: buffer size, output: received length)
 * @param timeout_ms Timeout in milliseconds
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_receive_timeout(BreezeUartInstance* uart, 
                                           uint8_t* data, 
                                           size_t* length, 
                                           uint32_t timeout_ms);

/**
 * @brief Check if data is available for reading
 * @param uart Pointer to UART instance
 * @return Number of bytes available for reading
 */
size_t breeze_uart_available(const BreezeUartInstance* uart);

/**
 * @brief Flush UART buffers
 * @param uart Pointer to UART instance
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_flush(BreezeUartInstance* uart);

/**
 * @brief Reset UART instance
 * @param uart Pointer to UART instance
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_reset(BreezeUartInstance* uart);

/**
 * @brief Set UART configuration
 * @param uart Pointer to UART instance
 * @param config Pointer to new configuration
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_set_config(BreezeUartInstance* uart, const BreezeUartConfig* config);

/**
 * @brief Get UART configuration
 * @param uart Pointer to UART instance
 * @param config Pointer to store current configuration
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_get_config(const BreezeUartInstance* uart, BreezeUartConfig* config);

/**
 * @brief Set UART baudrate
 * @param uart Pointer to UART instance
 * @param baudrate New baudrate
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_set_baudrate(BreezeUartInstance* uart, uint32_t baudrate);

/**
 * @brief Get UART baudrate
 * @param uart Pointer to UART instance
 * @return Current baudrate, 0 if error
 */
uint32_t breeze_uart_get_baudrate(const BreezeUartInstance* uart);

/**
 * @brief Set UART timeout
 * @param uart Pointer to UART instance
 * @param timeout_ms Timeout in milliseconds
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_set_timeout(BreezeUartInstance* uart, uint32_t timeout_ms);

/**
 * @brief Get UART timeout
 * @param uart Pointer to UART instance
 * @return Current timeout in milliseconds, 0 if error
 */
uint32_t breeze_uart_get_timeout(const BreezeUartInstance* uart);

/**
 * @brief Send single byte via UART
 * @param uart Pointer to UART instance
 * @param byte Byte to send
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_send_byte(BreezeUartInstance* uart, uint8_t byte);

/**
 * @brief Receive single byte from UART
 * @param uart Pointer to UART instance
 * @param byte Pointer to store received byte
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_receive_byte(BreezeUartInstance* uart, uint8_t* byte);

/**
 * @brief Send string via UART
 * @param uart Pointer to UART instance
 * @param str Null-terminated string to send
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_send_string(BreezeUartInstance* uart, const char* str);

/**
 * @brief Receive line from UART (until newline or timeout)
 * @param uart Pointer to UART instance
 * @param buffer Buffer to store received line
 * @param buffer_size Size of buffer
 * @param received_length Pointer to store actual received length
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_receive_line(BreezeUartInstance* uart, 
                                        char* buffer, 
                                        size_t buffer_size, 
                                        size_t* received_length);

/**
 * @brief Check if UART transmission is complete
 * @param uart Pointer to UART instance
 * @return 1 if transmission complete, 0 if in progress
 */
int breeze_uart_is_tx_complete(const BreezeUartInstance* uart);

/**
 * @brief Check if UART reception is in progress
 * @param uart Pointer to UART instance
 * @return 1 if reception in progress, 0 if idle
 */
int breeze_uart_is_rx_active(const BreezeUartInstance* uart);

/**
 * @brief Get UART statistics
 * @param uart Pointer to UART instance
 * @param stats Pointer to store statistics
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_get_stats(const BreezeUartInstance* uart, BreezeCommStats* stats);

/**
 * @brief Clear UART statistics
 * @param uart Pointer to UART instance
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_uart_clear_stats(BreezeUartInstance* uart);

/**
 * @brief Validate UART configuration
 * @param config Pointer to configuration to validate
 * @return BREEZE_SUCCESS if valid, error code otherwise
 */
BreezeErrorCode breeze_uart_validate_config(const BreezeUartConfig* config);

/**
 * @brief Create UART instance with default configuration
 * @param uart Pointer to UART instance
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_uart_create_default(BreezeUartInstance* uart) {
    BreezeUartConfig default_config = BREEZE_UART_CONFIG_DEFAULT;
    return breeze_uart_init(uart, &default_config);
}

/**
 * @brief Check if UART is initialized
 * @param uart Pointer to UART instance
 * @return 1 if initialized, 0 if not initialized
 */
static inline int breeze_uart_is_initialized(const BreezeUartInstance* uart) {
    return uart ? uart->comm_interface.is_initialized : 0;
}

/**
 * @brief Get UART error count
 * @param uart Pointer to UART instance
 * @return Error count
 */
static inline uint32_t breeze_uart_get_error_count(const BreezeUartInstance* uart) {
    return uart ? uart->comm_interface.stats.errors : 0;
}

/**
 * @brief Get UART timeout count
 * @param uart Pointer to UART instance
 * @return Timeout count
 */
static inline uint32_t breeze_uart_get_timeout_count(const BreezeUartInstance* uart) {
    return uart ? uart->comm_interface.stats.timeouts : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_COMM_UART_H */