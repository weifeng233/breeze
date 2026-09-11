/**
 * @file comm_hal.c
 * @brief Implementation of Hardware Abstraction Layer for communication protocols
 *
 * This file provides the implementation of global HAL operations registry.
 */

#include "../../include/breeze/comm/comm_hal.h"

/**
 * @brief Global HAL operations instance
 */
BreezeCommHalOps g_breeze_comm_hal_ops = {0};

/**
 * @brief Register HAL operations
 * @param ops Pointer to HAL operations structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_hal_register(const BreezeCommHalOps* ops) {
    if (!ops) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    g_breeze_comm_hal_ops = *ops;
    return BREEZE_SUCCESS;
}