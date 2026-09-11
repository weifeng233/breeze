/**
 * @file comm_interface.h
 * @brief Communication protocols interface for Breeze Framework
 *
 * This header provides standardized communication interface structures and
 * hardware abstraction layer for various communication protocols.
 */

#ifndef BREEZE_COMM_INTERFACE_H
#define BREEZE_COMM_INTERFACE_H

#include "../core/error_codes.h"
#include "../core/config.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum buffer size for communication operations
 */
#ifndef BREEZE_COMM_BUFFER_SIZE
#define BREEZE_COMM_BUFFER_SIZE 1024
#endif

/**
 * @brief Communication protocol types
 */
typedef enum {
    BREEZE_COMM_PROTOCOL_UART = 0,     /**< UART/Serial communication */
    BREEZE_COMM_PROTOCOL_SPI = 1,      /**< SPI communication */
    BREEZE_COMM_PROTOCOL_I2C = 2,      /**< I2C communication */
    BREEZE_COMM_PROTOCOL_CAN = 3,      /**< CAN bus communication */
    BREEZE_COMM_PROTOCOL_ETHERNET = 4, /**< Ethernet communication */
    BREEZE_COMM_PROTOCOL_USB = 5,      /**< USB communication */
    BREEZE_COMM_PROTOCOL_CUSTOM = 99   /**< Custom protocol */
} BreezeCommProtocol;

/**
 * @brief Communication operation modes
 */
typedef enum {
    BREEZE_COMM_MODE_BLOCKING = 0,     /**< Blocking operation */
    BREEZE_COMM_MODE_NON_BLOCKING = 1, /**< Non-blocking operation */
    BREEZE_COMM_MODE_INTERRUPT = 2,    /**< Interrupt-driven operation */
    BREEZE_COMM_MODE_DMA = 3           /**< DMA-driven operation */
} BreezeCommMode;

/**
 * @brief Communication buffer structure
 */
typedef struct {
    uint8_t* data;          /**< Buffer data pointer */
    size_t size;            /**< Buffer size */
    size_t used;            /**< Used bytes in buffer */
    size_t read_pos;        /**< Current read position */
    size_t write_pos;       /**< Current write position */
    int is_circular;        /**< Circular buffer flag */
} BreezeCommBuffer;

/**
 * @brief Communication statistics
 */
typedef struct {
    uint32_t bytes_sent;        /**< Total bytes sent */
    uint32_t bytes_received;    /**< Total bytes received */
    uint32_t packets_sent;      /**< Total packets sent */
    uint32_t packets_received;  /**< Total packets received */
    uint32_t errors;            /**< Total error count */
    uint32_t timeouts;          /**< Total timeout count */
    uint32_t retries;           /**< Total retry count */
} BreezeCommStats;

/**
 * @brief Communication callback function types
 */
typedef void (*BreezeCommTxCallback)(void* context, BreezeErrorCode result);
typedef void (*BreezeCommRxCallback)(void* context, const uint8_t* data, size_t length);
typedef void (*BreezeCommErrorCallback)(void* context, BreezeErrorCode error);

/**
 * @brief Communication interface configuration
 */
typedef struct {
    BreezeCommProtocol protocol;        /**< Communication protocol */
    BreezeCommMode mode;                /**< Operation mode */
    uint32_t baudrate;                  /**< Baudrate (for applicable protocols) */
    uint32_t timeout_ms;                /**< Timeout in milliseconds */
    uint8_t address;                    /**< Device address (for applicable protocols) */
    void* hw_config;                    /**< Hardware-specific configuration */
    BreezeCommTxCallback tx_callback;   /**< Transmit completion callback */
    BreezeCommRxCallback rx_callback;   /**< Receive data callback */
    BreezeCommErrorCallback err_callback; /**< Error callback */
    void* callback_context;             /**< Callback context pointer */
} BreezeCommConfig;

/**
 * @brief Forward declaration of communication interface
 */
typedef struct BreezeCommInterface BreezeCommInterface;

/**
 * @brief Communication interface function pointers
 */
typedef struct {
    BreezeErrorCode (*init)(BreezeCommInterface* comm, const BreezeCommConfig* config);
    BreezeErrorCode (*deinit)(BreezeCommInterface* comm);
    BreezeErrorCode (*send)(BreezeCommInterface* comm, const uint8_t* data, size_t length);
    BreezeErrorCode (*receive)(BreezeCommInterface* comm, uint8_t* data, size_t* length);
    BreezeErrorCode (*flush)(BreezeCommInterface* comm);
    BreezeErrorCode (*reset)(BreezeCommInterface* comm);
    BreezeErrorCode (*get_stats)(BreezeCommInterface* comm, BreezeCommStats* stats);
    BreezeErrorCode (*set_config)(BreezeCommInterface* comm, const BreezeCommConfig* config);
    BreezeErrorCode (*get_config)(BreezeCommInterface* comm, BreezeCommConfig* config);
} BreezeCommOperations;

/**
 * @brief Main communication interface structure
 */
struct BreezeCommInterface {
    BreezeCommConfig config;            /**< Interface configuration */
    BreezeCommOperations ops;           /**< Interface operations */
    BreezeCommBuffer tx_buffer;         /**< Transmit buffer */
    BreezeCommBuffer rx_buffer;         /**< Receive buffer */
    BreezeCommStats stats;              /**< Communication statistics */
    void* hw_handle;                    /**< Hardware handle */
    int is_initialized;                 /**< Initialization status */
    void* private_data;                 /**< Private implementation data */
};

/**
 * @brief Initialize communication interface
 * @param comm Pointer to communication interface
 * @param config Pointer to configuration
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_init(BreezeCommInterface* comm, 
                                              const BreezeCommConfig* config) {
    if (!comm || !config) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (comm->is_initialized) {
        return BREEZE_ERROR_ALREADY_INITIALIZED;
    }
    
    if (!comm->ops.init) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.init(comm, config);
}

/**
 * @brief Deinitialize communication interface
 * @param comm Pointer to communication interface
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_deinit(BreezeCommInterface* comm) {
    if (!comm) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (!comm->ops.deinit) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.deinit(comm);
}

/**
 * @brief Send data through communication interface
 * @param comm Pointer to communication interface
 * @param data Pointer to data to send
 * @param length Length of data to send
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_send(BreezeCommInterface* comm, 
                                              const uint8_t* data, 
                                              size_t length) {
    if (!comm || !data || length == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (!comm->ops.send) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.send(comm, data, length);
}

/**
 * @brief Receive data from communication interface
 * @param comm Pointer to communication interface
 * @param data Pointer to buffer for received data
 * @param length Pointer to length (input: buffer size, output: received length)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_receive(BreezeCommInterface* comm, 
                                                 uint8_t* data, 
                                                 size_t* length) {
    if (!comm || !data || !length) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (!comm->ops.receive) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.receive(comm, data, length);
}

/**
 * @brief Flush communication buffers
 * @param comm Pointer to communication interface
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_flush(BreezeCommInterface* comm) {
    if (!comm) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (!comm->ops.flush) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.flush(comm);
}

/**
 * @brief Reset communication interface
 * @param comm Pointer to communication interface
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_reset(BreezeCommInterface* comm) {
    if (!comm) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (!comm->ops.reset) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    return comm->ops.reset(comm);
}

/**
 * @brief Get communication statistics
 * @param comm Pointer to communication interface
 * @param stats Pointer to statistics structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_get_stats(BreezeCommInterface* comm, 
                                                   BreezeCommStats* stats) {
    if (!comm || !stats) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!comm->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    if (comm->ops.get_stats) {
        return comm->ops.get_stats(comm, stats);
    }
    
    /* Default implementation - copy internal stats */
    *stats = comm->stats;
    return BREEZE_SUCCESS;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_COMM_INTERFACE_H */