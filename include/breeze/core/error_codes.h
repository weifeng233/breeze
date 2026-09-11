/**
 * @file error_codes.h
 * @brief Standardized error handling system for Breeze Framework
 *
 * This header provides standardized error codes and error handling utilities
 * used across all Breeze Framework modules.
 */

#ifndef BREEZE_ERROR_CODES_H
#define BREEZE_ERROR_CODES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Standardized error codes for Breeze Framework
 */
typedef enum {
    BREEZE_SUCCESS = 0,                    /**< Operation completed successfully */
    BREEZE_ERROR_INVALID_PARAM = -1,       /**< Invalid parameter provided */
    BREEZE_ERROR_OUT_OF_MEMORY = -2,       /**< Insufficient memory available */
    BREEZE_ERROR_TIMEOUT = -3,             /**< Operation timed out */
    BREEZE_ERROR_NOT_SUPPORTED = -4,       /**< Feature not supported */
    BREEZE_ERROR_HARDWARE_FAULT = -5,      /**< Hardware fault detected */
    BREEZE_ERROR_BUFFER_OVERFLOW = -6,     /**< Buffer overflow detected */
    BREEZE_ERROR_BUFFER_UNDERFLOW = -7,    /**< Buffer underflow detected */
    BREEZE_ERROR_INVALID_STATE = -8,       /**< Invalid system state */
    BREEZE_ERROR_COMMUNICATION = -9,       /**< Communication error */
    BREEZE_ERROR_CHECKSUM = -10,           /**< Checksum verification failed */
    BREEZE_ERROR_NOT_INITIALIZED = -11,    /**< Module not initialized */
    BREEZE_ERROR_ALREADY_INITIALIZED = -12, /**< Module already initialized */
    BREEZE_ERROR_RESOURCE_BUSY = -13,      /**< Resource is busy */
    BREEZE_ERROR_PERMISSION_DENIED = -14,  /**< Permission denied */
    BREEZE_ERROR_FILE_NOT_FOUND = -15,     /**< File not found */
    BREEZE_ERROR_NOT_FOUND = -16,          /**< Item not found */
    BREEZE_ERROR_UNKNOWN = -99             /**< Unknown error */
} BreezeErrorCode;

/**
 * @brief Error callback function type
 * @param error_code The error code that occurred
 * @param module_name Name of the module where error occurred
 * @param message Optional error message
 */
typedef void (*BreezeErrorCallback)(BreezeErrorCode error_code, 
                                   const char* module_name, 
                                   const char* message);

/**
 * @brief Global error callback (optional)
 */
extern BreezeErrorCallback g_breeze_error_callback;

/**
 * @brief Set global error callback
 * @param callback Error callback function
 */
static inline void breeze_set_error_callback(BreezeErrorCallback callback) {
    g_breeze_error_callback = callback;
}

/**
 * @brief Report error to global callback if set
 * @param error_code The error code
 * @param module_name Name of the module
 * @param message Optional error message
 */
static inline void breeze_report_error(BreezeErrorCode error_code, 
                                      const char* module_name, 
                                      const char* message) {
    if (g_breeze_error_callback) {
        g_breeze_error_callback(error_code, module_name, message);
    }
}

/**
 * @brief Get error message string for error code
 * @param error_code The error code
 * @return String description of the error
 */
static inline const char* breeze_error_string(BreezeErrorCode error_code) {
    switch (error_code) {
        case BREEZE_SUCCESS: return "Success";
        case BREEZE_ERROR_INVALID_PARAM: return "Invalid parameter";
        case BREEZE_ERROR_OUT_OF_MEMORY: return "Out of memory";
        case BREEZE_ERROR_TIMEOUT: return "Timeout";
        case BREEZE_ERROR_NOT_SUPPORTED: return "Not supported";
        case BREEZE_ERROR_HARDWARE_FAULT: return "Hardware fault";
        case BREEZE_ERROR_BUFFER_OVERFLOW: return "Buffer overflow";
        case BREEZE_ERROR_BUFFER_UNDERFLOW: return "Buffer underflow";
        case BREEZE_ERROR_INVALID_STATE: return "Invalid state";
        case BREEZE_ERROR_COMMUNICATION: return "Communication error";
        case BREEZE_ERROR_CHECKSUM: return "Checksum error";
        case BREEZE_ERROR_NOT_INITIALIZED: return "Not initialized";
        case BREEZE_ERROR_ALREADY_INITIALIZED: return "Already initialized";
        case BREEZE_ERROR_RESOURCE_BUSY: return "Resource busy";
        case BREEZE_ERROR_PERMISSION_DENIED: return "Permission denied";
        case BREEZE_ERROR_FILE_NOT_FOUND: return "File not found";
        default: return "Unknown error";
    }
}

/**
 * @brief Check if error code indicates success
 * @param error_code The error code to check
 * @return 1 if success, 0 if error
 */
static inline int breeze_is_success(BreezeErrorCode error_code) {
    return error_code == BREEZE_SUCCESS;
}

/**
 * @brief Check if error code indicates failure
 * @param error_code The error code to check
 * @return 1 if error, 0 if success
 */
static inline int breeze_is_error(BreezeErrorCode error_code) {
    return error_code != BREEZE_SUCCESS;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_ERROR_CODES_H */