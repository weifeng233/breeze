/**
 * @file config.h
 * @brief Module configuration management system for Breeze Framework
 *
 * This header provides standardized configuration management utilities
 * used across all Breeze Framework modules.
 */

#ifndef BREEZE_CONFIG_H
#define BREEZE_CONFIG_H

#include "error_codes.h"
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum length for module names
 */
#define BREEZE_MODULE_NAME_MAX_LEN 32

/**
 * @brief Maximum number of registered modules
 */
#ifndef BREEZE_MAX_MODULES
#define BREEZE_MAX_MODULES 16
#endif

/**
 * @brief Configuration validation function type
 * @param config Pointer to configuration data
 * @return BREEZE_SUCCESS if valid, error code otherwise
 */
typedef BreezeErrorCode (*BreezeConfigValidator)(const void* config);

/**
 * @brief Configuration initialization function type
 * @param config Pointer to configuration data
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
typedef BreezeErrorCode (*BreezeConfigInitializer)(void* config);

/**
 * @brief Configuration cleanup function type
 * @param config Pointer to configuration data
 */
typedef void (*BreezeConfigCleanup)(void* config);

/**
 * @brief Module configuration descriptor
 */
typedef struct {
    char module_name[BREEZE_MODULE_NAME_MAX_LEN];  /**< Module name */
    void* config_data;                             /**< Configuration data pointer */
    size_t config_size;                            /**< Size of configuration data */
    BreezeConfigValidator validate;                /**< Validation function */
    BreezeConfigInitializer initialize;            /**< Initialization function */
    BreezeConfigCleanup cleanup;                   /**< Cleanup function */
    int is_initialized;                            /**< Initialization status */
} BreezeModuleConfig;

/**
 * @brief Global module registry
 */
extern BreezeModuleConfig g_breeze_module_registry[BREEZE_MAX_MODULES];
extern int g_breeze_module_count;

/**
 * @brief Register a module configuration
 * @param module_name Name of the module
 * @param config_data Pointer to configuration data
 * @param config_size Size of configuration data
 * @param validator Configuration validator function (optional)
 * @param initializer Configuration initializer function (optional)
 * @param cleanup Configuration cleanup function (optional)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_register_module(
    const char* module_name,
    void* config_data,
    size_t config_size,
    BreezeConfigValidator validator,
    BreezeConfigInitializer initializer,
    BreezeConfigCleanup cleanup) {
    
    int i;
    
    if (!module_name || !config_data) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (g_breeze_module_count >= BREEZE_MAX_MODULES) {
        return BREEZE_ERROR_OUT_OF_MEMORY;
    }
    
    /* Check if module already registered */
    for (i = 0; i < g_breeze_module_count; i++) {
        if (strcmp(g_breeze_module_registry[i].module_name, module_name) == 0) {
            return BREEZE_ERROR_ALREADY_INITIALIZED;
        }
    }
    
    /* Register new module */
    strncpy(g_breeze_module_registry[g_breeze_module_count].module_name, 
            module_name, BREEZE_MODULE_NAME_MAX_LEN - 1);
    g_breeze_module_registry[g_breeze_module_count].module_name[BREEZE_MODULE_NAME_MAX_LEN - 1] = '\0';
    g_breeze_module_registry[g_breeze_module_count].config_data = config_data;
    g_breeze_module_registry[g_breeze_module_count].config_size = config_size;
    g_breeze_module_registry[g_breeze_module_count].validate = validator;
    g_breeze_module_registry[g_breeze_module_count].initialize = initializer;
    g_breeze_module_registry[g_breeze_module_count].cleanup = cleanup;
    g_breeze_module_registry[g_breeze_module_count].is_initialized = 0;
    
    g_breeze_module_count++;
    
    return BREEZE_SUCCESS;
}

/**
 * @brief Find module configuration by name
 * @param module_name Name of the module
 * @return Pointer to module configuration or NULL if not found
 */
static inline BreezeModuleConfig* breeze_find_module(const char* module_name) {
    int i;
    
    if (!module_name) {
        return NULL;
    }
    
    for (i = 0; i < g_breeze_module_count; i++) {
        if (strcmp(g_breeze_module_registry[i].module_name, module_name) == 0) {
            return &g_breeze_module_registry[i];
        }
    }
    
    return NULL;
}

/**
 * @brief Initialize a module
 * @param module_name Name of the module
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_initialize_module(const char* module_name) {
    BreezeModuleConfig* module = breeze_find_module(module_name);
    BreezeErrorCode result;
    
    if (!module) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (module->is_initialized) {
        return BREEZE_ERROR_ALREADY_INITIALIZED;
    }
    
    /* Validate configuration if validator provided */
    if (module->validate) {
        result = module->validate(module->config_data);
        if (breeze_is_error(result)) {
            breeze_report_error(result, module_name, "Configuration validation failed");
            return result;
        }
    }
    
    /* Initialize module if initializer provided */
    if (module->initialize) {
        result = module->initialize(module->config_data);
        if (breeze_is_error(result)) {
            breeze_report_error(result, module_name, "Module initialization failed");
            return result;
        }
    }
    
    module->is_initialized = 1;
    return BREEZE_SUCCESS;
}

/**
 * @brief Cleanup a module
 * @param module_name Name of the module
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
static inline BreezeErrorCode breeze_cleanup_module(const char* module_name) {
    BreezeModuleConfig* module = breeze_find_module(module_name);
    
    if (!module) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!module->is_initialized) {
        return BREEZE_ERROR_NOT_INITIALIZED;
    }
    
    /* Cleanup module if cleanup function provided */
    if (module->cleanup) {
        module->cleanup(module->config_data);
    }
    
    module->is_initialized = 0;
    return BREEZE_SUCCESS;
}

/**
 * @brief Initialize all registered modules
 * @return BREEZE_SUCCESS if all successful, first error code otherwise
 */
static inline BreezeErrorCode breeze_initialize_all_modules(void) {
    int i;
    BreezeErrorCode result;
    
    for (i = 0; i < g_breeze_module_count; i++) {
        if (!g_breeze_module_registry[i].is_initialized) {
            result = breeze_initialize_module(g_breeze_module_registry[i].module_name);
            if (breeze_is_error(result)) {
                return result;
            }
        }
    }
    
    return BREEZE_SUCCESS;
}

/**
 * @brief Cleanup all registered modules
 */
static inline void breeze_cleanup_all_modules(void) {
    int i;
    
    for (i = 0; i < g_breeze_module_count; i++) {
        if (g_breeze_module_registry[i].is_initialized) {
            breeze_cleanup_module(g_breeze_module_registry[i].module_name);
        }
    }
}

/**
 * @brief Check if module is initialized
 * @param module_name Name of the module
 * @return 1 if initialized, 0 if not initialized or not found
 */
static inline int breeze_is_module_initialized(const char* module_name) {
    BreezeModuleConfig* module = breeze_find_module(module_name);
    return module ? module->is_initialized : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_CONFIG_H */