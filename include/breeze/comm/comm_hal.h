/**
 * @file comm_hal.h
 * @brief Hardware Abstraction Layer for communication protocols
 *
 * This header provides hardware abstraction layer interfaces for various
 * communication protocols in the Breeze Framework.
 */

#ifndef BREEZE_COMM_HAL_H
#define BREEZE_COMM_HAL_H

#include "comm_interface.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware-specific UART configuration
 */
typedef struct {
    uint32_t baudrate;          /**< Baudrate */
    uint8_t data_bits;          /**< Data bits (5-8) */
    uint8_t stop_bits;          /**< Stop bits (1-2) */
    uint8_t parity;             /**< Parity (0=none, 1=odd, 2=even) */
    uint8_t flow_control;       /**< Flow control (0=none, 1=hardware, 2=software) */
    void* port_handle;          /**< Platform-specific port handle */
} BreezeCommUartConfig;

/**
 * @brief Hardware-specific SPI configuration
 */
typedef struct {
    uint32_t clock_speed;       /**< SPI clock speed in Hz */
    uint8_t mode;               /**< SPI mode (0-3) */
    uint8_t bit_order;          /**< Bit order (0=MSB first, 1=LSB first) */
    uint8_t cs_pin;             /**< Chip select pin */
    uint8_t cs_active_low;      /**< CS active low flag */
    void* spi_handle;           /**< Platform-specific SPI handle */
} BreezeCommSpiConfig;

/**
 * @brief Hardware-specific I2C configuration
 */
typedef struct {
    uint32_t clock_speed;       /**< I2C clock speed in Hz */
    uint8_t address_mode;       /**< Address mode (7-bit or 10-bit) */
    uint8_t slave_address;      /**< Slave device address */
    void* i2c_handle;           /**< Platform-specific I2C handle */
} BreezeCommI2cConfig;

/**
 * @brief Hardware abstraction layer operations
 */
typedef struct {
    /* Platform initialization */
    BreezeErrorCode (*platform_init)(void);
    BreezeErrorCode (*platform_deinit)(void);
    
    /* GPIO operations */
    BreezeErrorCode (*gpio_set)(uint8_t pin, uint8_t value);
    BreezeErrorCode (*gpio_get)(uint8_t pin, uint8_t* value);
    BreezeErrorCode (*gpio_config)(uint8_t pin, uint8_t mode);
    
    /* Timer operations */
    BreezeErrorCode (*timer_start)(uint32_t timeout_ms);
    BreezeErrorCode (*timer_stop)(void);
    uint8_t (*timer_expired)(void);
    
    /* Interrupt operations */
    BreezeErrorCode (*interrupt_enable)(uint8_t irq_num);
    BreezeErrorCode (*interrupt_disable)(uint8_t irq_num);
    BreezeErrorCode (*interrupt_register)(uint8_t irq_num, void (*handler)(void));
    
    /* Memory operations */
    void* (*memory_alloc)(size_t size);
    void (*memory_free)(void* ptr);
    void (*memory_copy)(void* dest, const void* src, size_t size);
    void (*memory_set)(void* ptr, uint8_t value, size_t size);
    
    /* Critical section operations */
    void (*critical_enter)(void);
    void (*critical_exit)(void);
    
} BreezeCommHalOps;

/**
 * @brief Global HAL operations instance
 */
extern BreezeCommHalOps g_breeze_comm_hal_ops;

/**
 * @brief Register HAL operations
 * @param ops Pointer to HAL operations structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_hal_register(const BreezeCommHalOps* ops);

/**
 * @brief Initialize platform-specific hardware
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_platform_init(void) {
    if (!g_breeze_comm_hal_ops.platform_init) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.platform_init();
}

/**
 * @brief Deinitialize platform-specific hardware
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_platform_deinit(void) {
    if (!g_breeze_comm_hal_ops.platform_deinit) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.platform_deinit();
}

/**
 * @brief Set GPIO pin value
 * @param pin GPIO pin number
 * @param value Pin value (0 or 1)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_gpio_set(uint8_t pin, uint8_t value) {
    if (!g_breeze_comm_hal_ops.gpio_set) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.gpio_set(pin, value);
}

/**
 * @brief Get GPIO pin value
 * @param pin GPIO pin number
 * @param value Pointer to store pin value
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_gpio_get(uint8_t pin, uint8_t* value) {
    if (!g_breeze_comm_hal_ops.gpio_get || !value) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    return g_breeze_comm_hal_ops.gpio_get(pin, value);
}

/**
 * @brief Configure GPIO pin mode
 * @param pin GPIO pin number
 * @param mode Pin mode (input/output/etc.)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_gpio_config(uint8_t pin, uint8_t mode) {
    if (!g_breeze_comm_hal_ops.gpio_config) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.gpio_config(pin, mode);
}

/**
 * @brief Start hardware timer
 * @param timeout_ms Timeout in milliseconds
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_timer_start(uint32_t timeout_ms) {
    if (!g_breeze_comm_hal_ops.timer_start) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.timer_start(timeout_ms);
}

/**
 * @brief Stop hardware timer
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_comm_hal_timer_stop(void) {
    if (!g_breeze_comm_hal_ops.timer_stop) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    return g_breeze_comm_hal_ops.timer_stop();
}

/**
 * @brief Check if timer has expired
 * @return 1 if expired, 0 if not expired
 */
static inline uint8_t breeze_comm_hal_timer_expired(void) {
    if (!g_breeze_comm_hal_ops.timer_expired) {
        return 0;
    }
    return g_breeze_comm_hal_ops.timer_expired();
}

/**
 * @brief Enter critical section
 */
static inline void breeze_comm_hal_critical_enter(void) {
    if (g_breeze_comm_hal_ops.critical_enter) {
        g_breeze_comm_hal_ops.critical_enter();
    }
}

/**
 * @brief Exit critical section
 */
static inline void breeze_comm_hal_critical_exit(void) {
    if (g_breeze_comm_hal_ops.critical_exit) {
        g_breeze_comm_hal_ops.critical_exit();
    }
}

/**
 * @brief Allocate memory
 * @param size Size in bytes
 * @return Pointer to allocated memory or NULL if failed
 */
static inline void* breeze_comm_hal_memory_alloc(size_t size) {
    if (!g_breeze_comm_hal_ops.memory_alloc) {
        return NULL;
    }
    return g_breeze_comm_hal_ops.memory_alloc(size);
}

/**
 * @brief Free allocated memory
 * @param ptr Pointer to memory to free
 */
static inline void breeze_comm_hal_memory_free(void* ptr) {
    if (g_breeze_comm_hal_ops.memory_free && ptr) {
        g_breeze_comm_hal_ops.memory_free(ptr);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_COMM_HAL_H */