/**
 * @file comm_buffer.c
 * @brief Implementation of buffer management utilities for communication protocols
 *
 * This file provides the implementation of communication buffer management functions.
 */

#include "../../include/breeze/comm/comm_buffer.h"
#include <string.h>

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
                                       int is_circular) {
    if (!buffer || !data || size == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    buffer->data = data;
    buffer->size = size;
    buffer->used = 0;
    buffer->read_pos = 0;
    buffer->write_pos = 0;
    buffer->is_circular = is_circular;
    
    return BREEZE_SUCCESS;
}

/**
 * @brief Reset buffer to empty state
 * @param buffer Pointer to buffer structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_reset(BreezeCommBuffer* buffer) {
    if (!buffer) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    buffer->used = 0;
    buffer->read_pos = 0;
    buffer->write_pos = 0;
    
    return BREEZE_SUCCESS;
}

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
                                        size_t* written) {
    size_t available_space;
    size_t bytes_to_write;
    size_t first_chunk;
    size_t second_chunk;
    
    if (!buffer || !data || length == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    available_space = breeze_comm_buffer_available_space(buffer);
    bytes_to_write = (length <= available_space) ? length : available_space;
    
    if (bytes_to_write == 0) {
        if (written) *written = 0;
        return buffer->is_circular ? BREEZE_SUCCESS : BREEZE_ERROR_BUFFER_OVERFLOW;
    }
    
    if (buffer->is_circular) {
        /* Handle circular buffer wrap-around */
        first_chunk = (buffer->write_pos + bytes_to_write <= buffer->size) ? 
                      bytes_to_write : (buffer->size - buffer->write_pos);
        second_chunk = bytes_to_write - first_chunk;
        
        memcpy(&buffer->data[buffer->write_pos], data, first_chunk);
        if (second_chunk > 0) {
            memcpy(&buffer->data[0], &data[first_chunk], second_chunk);
        }
        
        buffer->write_pos = (buffer->write_pos + bytes_to_write) % buffer->size;
    } else {
        /* Linear buffer */
        memcpy(&buffer->data[buffer->write_pos], data, bytes_to_write);
        buffer->write_pos += bytes_to_write;
    }
    
    buffer->used += bytes_to_write;
    
    if (written) {
        *written = bytes_to_write;
    }
    
    return BREEZE_SUCCESS;
}

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
                                       size_t* read) {
    size_t bytes_to_read;
    size_t first_chunk;
    size_t second_chunk;
    
    if (!buffer || !data || length == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    bytes_to_read = (length <= buffer->used) ? length : buffer->used;
    
    if (bytes_to_read == 0) {
        if (read) *read = 0;
        return BREEZE_SUCCESS;
    }
    
    if (buffer->is_circular) {
        /* Handle circular buffer wrap-around */
        first_chunk = (buffer->read_pos + bytes_to_read <= buffer->size) ? 
                      bytes_to_read : (buffer->size - buffer->read_pos);
        second_chunk = bytes_to_read - first_chunk;
        
        memcpy(data, &buffer->data[buffer->read_pos], first_chunk);
        if (second_chunk > 0) {
            memcpy(&data[first_chunk], &buffer->data[0], second_chunk);
        }
        
        buffer->read_pos = (buffer->read_pos + bytes_to_read) % buffer->size;
    } else {
        /* Linear buffer */
        memcpy(data, &buffer->data[buffer->read_pos], bytes_to_read);
        buffer->read_pos += bytes_to_read;
    }
    
    buffer->used -= bytes_to_read;
    
    if (read) {
        *read = bytes_to_read;
    }
    
    return BREEZE_SUCCESS;
}

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
                                       size_t* peeked) {
    size_t bytes_to_peek;
    size_t first_chunk;
    size_t second_chunk;
    
    if (!buffer || !data || length == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    bytes_to_peek = (length <= buffer->used) ? length : buffer->used;
    
    if (bytes_to_peek == 0) {
        if (peeked) *peeked = 0;
        return BREEZE_SUCCESS;
    }
    
    if (buffer->is_circular) {
        /* Handle circular buffer wrap-around */
        first_chunk = (buffer->read_pos + bytes_to_peek <= buffer->size) ? 
                      bytes_to_peek : (buffer->size - buffer->read_pos);
        second_chunk = bytes_to_peek - first_chunk;
        
        memcpy(data, &buffer->data[buffer->read_pos], first_chunk);
        if (second_chunk > 0) {
            memcpy(&data[first_chunk], &buffer->data[0], second_chunk);
        }
    } else {
        /* Linear buffer */
        memcpy(data, &buffer->data[buffer->read_pos], bytes_to_peek);
    }
    
    if (peeked) {
        *peeked = bytes_to_peek;
    }
    
    return BREEZE_SUCCESS;
}

/**
 * @brief Get available space in buffer
 * @param buffer Pointer to buffer structure
 * @return Available space in bytes
 */
size_t breeze_comm_buffer_available_space(const BreezeCommBuffer* buffer) {
    if (!buffer || !breeze_comm_buffer_is_valid(buffer)) {
        return 0;
    }
    
    return buffer->size - buffer->used;
}

/**
 * @brief Get used space in buffer
 * @param buffer Pointer to buffer structure
 * @return Used space in bytes
 */
size_t breeze_comm_buffer_used_space(const BreezeCommBuffer* buffer) {
    if (!buffer || !breeze_comm_buffer_is_valid(buffer)) {
        return 0;
    }
    
    return buffer->used;
}

/**
 * @brief Check if buffer is empty
 * @param buffer Pointer to buffer structure
 * @return 1 if empty, 0 if not empty
 */
int breeze_comm_buffer_is_empty(const BreezeCommBuffer* buffer) {
    if (!buffer) {
        return 1;
    }
    
    return buffer->used == 0;
}

/**
 * @brief Check if buffer is full
 * @param buffer Pointer to buffer structure
 * @return 1 if full, 0 if not full
 */
int breeze_comm_buffer_is_full(const BreezeCommBuffer* buffer) {
    if (!buffer || !breeze_comm_buffer_is_valid(buffer)) {
        return 0;
    }
    
    return buffer->used == buffer->size;
}

/**
 * @brief Skip bytes in buffer (advance read position)
 * @param buffer Pointer to buffer structure
 * @param bytes Number of bytes to skip
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_skip(BreezeCommBuffer* buffer, size_t bytes) {
    if (!buffer) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    if (bytes > buffer->used) {
        bytes = buffer->used;
    }
    
    if (buffer->is_circular) {
        buffer->read_pos = (buffer->read_pos + bytes) % buffer->size;
    } else {
        buffer->read_pos += bytes;
    }
    
    buffer->used -= bytes;
    
    return BREEZE_SUCCESS;
}

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
                                               size_t* offset) {
    size_t i, j;
    size_t search_length;
    
    if (!buffer || !pattern || pattern_length == 0 || !offset) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    if (pattern_length > buffer->used) {
        return BREEZE_ERROR_NOT_FOUND;
    }
    
    search_length = buffer->used - pattern_length + 1;
    
    for (i = 0; i < search_length; i++) {
        int match = 1;
        
        for (j = 0; j < pattern_length; j++) {
            size_t buffer_pos;
            
            if (buffer->is_circular) {
                buffer_pos = (buffer->read_pos + i + j) % buffer->size;
            } else {
                buffer_pos = buffer->read_pos + i + j;
            }
            
            if (buffer->data[buffer_pos] != pattern[j]) {
                match = 0;
                break;
            }
        }
        
        if (match) {
            *offset = i;
            return BREEZE_SUCCESS;
        }
    }
    
    return BREEZE_ERROR_NOT_FOUND;
}

/**
 * @brief Compact buffer (move unread data to beginning)
 * @param buffer Pointer to buffer structure
 * @return BREEZE_SUCCESS if successful, error code otherwise
 */
BreezeErrorCode breeze_comm_buffer_compact(BreezeCommBuffer* buffer) {
    if (!buffer) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (!breeze_comm_buffer_is_valid(buffer)) {
        return BREEZE_ERROR_INVALID_STATE;
    }
    
    /* Only compact linear buffers */
    if (buffer->is_circular) {
        return BREEZE_ERROR_NOT_SUPPORTED;
    }
    
    if (buffer->used == 0 || buffer->read_pos == 0) {
        return BREEZE_SUCCESS;
    }
    
    memmove(&buffer->data[0], &buffer->data[buffer->read_pos], buffer->used);
    buffer->read_pos = 0;
    buffer->write_pos = buffer->used;
    
    return BREEZE_SUCCESS;
}

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
                                            size_t* available_size) {
    if (!buffer) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    if (total_size) {
        *total_size = buffer->size;
    }
    
    if (used_size) {
        *used_size = buffer->used;
    }
    
    if (available_size) {
        *available_size = buffer->size - buffer->used;
    }
    
    return BREEZE_SUCCESS;
}