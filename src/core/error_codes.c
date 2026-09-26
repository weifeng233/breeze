/**
 * @file error_codes.c
 * @brief Implementation of error handling system for Breeze Framework
 *
 * This file provides the implementation of global error handling utilities.
 */

#include "../../include/breeze/core/error_codes.h"
#include <stddef.h>   /* NULL */

/**
 * @brief Global error callback instance
 */
BreezeErrorCallback g_breeze_error_callback = NULL;