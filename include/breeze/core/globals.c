/**
 * @file globals.c
 * @brief Global variable definitions for Breeze Framework
 *
 * This file contains the definitions of global variables used across
 * the Breeze Framework modules.
 */

#include "config.h"
#include "../debug/unit_test.h"

/**
 * @brief Global error callback
 */
BreezeErrorCallback g_breeze_error_callback = NULL;

/**
 * @brief Global module registry
 */
BreezeModuleConfig g_breeze_module_registry[BREEZE_MAX_MODULES];

/**
 * @brief Global module count
 */
int g_breeze_module_count = 0;

/**
 * @brief Global test statistics
 */
BreezeTestStats g_breeze_test_stats = {0, 0, 0, 0, 0, 0};