/**
 * @file config.c
 * @brief Implementation of module configuration management system
 *
 * This file provides the implementation of global module registry and
 * configuration management utilities.
 */

#include "../../include/breeze/core/config.h"

/**
 * @brief Global module registry
 */
BreezeModuleConfig g_breeze_module_registry[BREEZE_MAX_MODULES];

/**
 * @brief Global module count
 */
int g_breeze_module_count = 0;