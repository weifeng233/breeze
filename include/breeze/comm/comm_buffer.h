/**
 * @file comm_buffer.h
 * @brief Buffer management utilities for communication protocols
 *
 * This header provides buffer management utilities for efficient data handling
 * in communication protocols within the Breeze Framework.
 */

#ifndef BREEZE_COMM_BUFFER_H
#define BREEZE_COMM_BUFFER_H

#include "../core/error_codes.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Communication buffer structure (forward declaration from comm_interface.h)
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
 * @brief Buffer management utilities
 */

/**
 * @brief Initialize communication buffer
 * @param buffer Pointer to buffer structure
 * @param data Pointer to buffer data
 * @param size Buffer size
 * @param is_circular Flag for circular buffer mode
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_init(BreezeCommBuffer* buffer, 
                                       uint8_t* data, 
                                       size_t size, 
                                       int is_circular);

/**
 * @brief Reset buffer to empty state
 * @param buffer Pointer to buffer structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_reset(BreezeCommBuffer* buffer);

/**
 * @brief Write data to buffer
 * @param buffer Pointer to buffer structure
 * @param data Pointer to data to write
 * @param length Length of data to write
 * @param written Pointer to store actual bytes written (optional)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_write(BreezeCommBuffer* buffer, 
                                        const uint8_t* data, 
                                        size_t length, 
                                        size_t* written);

/**
 * @brief Read data from buffer
 * @param buffer Pointer to buffer structure
 * @param data Pointer to buffer for read data
 * @param length Maximum length to read
 * @param read Pointer to store actual bytes read (optional)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_read(BreezeCommBuffer* buffer, 
                                       uint8_t* data, 
                                       size_t length, 
                                       size_t* read);

/**
 * @brief Peek data from buffer without removing it
 * @param buffer Pointer to buffer structure
 * @param data Pointer to buffer for peeked data
 * @param length Maximum length to peek
 * @param peeked Pointer to store actual bytes peeked (optional)
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_peek(const BreezeCommBuffer* buffer, 
                                       uint8_t* data, 
                                       size_t length, 
                                       size_t* peeked);

/**
 * @brief Get available space in buffer
 * @param buffer Pointer to buffer structure
 * @return Available space in bytes
 */
size_t breeze_comm_buffer_available_space(const BreezeCommBuffer* buffer);

/**
 * @brief Get used space in buffer
 * @param buffer Pointer to buffer structure
 * @return Used space in bytes
 */
size_t breeze_comm_buffer_used_space(const BreezeCommBuffer* buffer);

/**
 * @brief Check if buffer is empty
 * @param buffer Pointer to buffer structure
 * @return 1 if empty, 0 if not empty
 */
int breeze_comm_buffer_is_empty(const BreezeCommBuffer* buffer);

/**
 * @brief Check if buffer is full
 * @param buffer Pointer to buffer structure
 * @return 1 if full, 0 if not full
 */
int breeze_comm_buffer_is_full(const BreezeCommBuffer* buffer);

/**
 * @brief Skip bytes in buffer (advance read position)
 * @param buffer Pointer to buffer structure
 * @param bytes Number of bytes to skip
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_skip(BreezeCommBuffer* buffer, size_t bytes);

/**
 * @brief Find pattern in buffer
 * @param buffer Pointer to buffer structure
 * @param pattern Pointer to pattern to find
 * @param pattern_length Length of pattern
 * @param offset Pointer to store offset where pattern was found
 * @return BREEZE_SUCCESS if found, BREEZE_ERROR_NOT_FOUND if not found
 */
BreezeErrorCode breeze_comm_buffer_find_pattern(const BreezeCommBuffer* buffer, 
                                               const uint8_t* pattern, 
                                               size_t pattern_length, 
                                               size_t* offset);

/**
 * @brief Compact buffer (move unread data to beginning)
 * @param buffer Pointer to buffer structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_compact(BreezeCommBuffer* buffer);

/**
 * @brief Get buffer statistics
 * @param buffer Pointer to buffer structure
 * @param total_size Pointer to store total buffer size
 * @param used_size Pointer to store used buffer size
 * @param available_size Pointer to store available buffer size
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_get_stats(const BreezeCommBuffer* buffer, 
                                            size_t* total_size, 
                                            size_t* used_size, 
                                            size_t* available_size);

/**
 * @brief Buffer utility macros
 */
#define BREEZE_COMM_BUFFER_DECLARE(name, size) \
    static uint8_t name##_data[size]; \
    static BreezeCommBuffer name = {0}

#define BREEZE_COMM_BUFFER_INIT_STATIC(name, size, circular) \
    do { \
        breeze_comm_buffer_init(&name, name##_data, size, circular); \
    } while(0)

/**
 * @brief Inline buffer utility functions
 */

/**
 * @brief Get next write position in circular buffer
 * @param buffer Pointer to buffer structure
 * @return Next write position
 */
static inline size_t breeze_comm_buffer_next_write_pos(const BreezeCommBuffer* buffer) {
    if (!buffer || !buffer->is_circular) {
        return 0;
    }
    return (buffer->write_pos + 1) % buffer->size;
}

/**
 * @brief Get next read position in circular buffer
 * @param buffer Pointer to buffer structure
 * @return Next read position
 */
static inline size_t breeze_comm_buffer_next_read_pos(const BreezeCommBuffer* buffer) {
    if (!buffer || !buffer->is_circular) {
        return 0;
    }
    return (buffer->read_pos + 1) % buffer->size;
}

/**
 * @brief Check if buffer positions are valid
 * @param buffer Pointer to buffer structure
 * @return 1 if valid, 0 if invalid
 */
static inline int breeze_comm_buffer_is_valid(const BreezeCommBuffer* buffer) {
    if (!buffer || !buffer->data || buffer->size == 0) {
        return 0;
    }
    
    if (buffer->read_pos >= buffer->size || buffer->write_pos >= buffer->size) {
        return 0;
    }
    
    if (!buffer->is_circular && buffer->used > buffer->size) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief Calculate distance between two positions in circular buffer
 * @param buffer Pointer to buffer structure
 * @param start Start position
 * @param end End position
 * @return Distance between positions
 */
static inline size_t breeze_comm_buffer_distance(const BreezeCommBuffer* buffer, 
                                                size_t start, 
                                                size_t end) {
    if (!buffer || !buffer->is_circular) {
        return (end >= start) ? (end - start) : 0;
    }
    
    if (end >= start) {
        return end - start;
    } else {
        return (buffer->size - start) + end;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_COMM_BUFFER_H */