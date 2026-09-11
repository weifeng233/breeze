/**
 * @file test_comm_interface.c
 * @brief Unit tests for communication interface abstractions
 *
 * This file contains unit tests for the communication interface structures
 * and functions in the Breeze Framework.
 */

#include "../../include/breeze/comm/comm_interface.h"
#include "../../include/breeze/debug/unit_test.h"
#include <stdlib.h>
#include <string.h>

/* Mock data for testing */
static uint8_t test_tx_buffer_data[256];
static uint8_t test_rx_buffer_data[256];
static BreezeCommInterface test_comm_interface;
static int mock_init_called = 0;
static int mock_deinit_called = 0;
static int mock_send_called = 0;
static int mock_receive_called = 0;

/* Mock implementation functions */
static BreezeErrorCode mock_comm_init(BreezeCommInterface* comm, const BreezeCommConfig* config) {
    mock_init_called = 1;
    if (!comm || !config) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    comm->config = *config;
    comm->is_initialized = 1;
    return BREEZE_SUCCESS;
}

static BreezeErrorCode mock_comm_deinit(BreezeCommInterface* comm) {
    mock_deinit_called = 1;
    if (!comm) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    comm->is_initialized = 0;
    return BREEZE_SUCCESS;
}

static BreezeErrorCode mock_comm_send(BreezeCommInterface* comm, const uint8_t* data, size_t length) {
    mock_send_called = 1;
    if (!comm || !data || length == 0) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    comm->stats.bytes_sent += length;
    comm->stats.packets_sent++;
    return BREEZE_SUCCESS;
}

static BreezeErrorCode mock_comm_receive(BreezeCommInterface* comm, uint8_t* data, size_t* length) {
    mock_receive_called = 1;
    if (!comm || !data || !length) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    
    /* Mock receiving some test data */
    const uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04};
    size_t test_data_len = sizeof(test_data);
    size_t copy_len = (*length < test_data_len) ? *length : test_data_len;
    
    memcpy(data, test_data, copy_len);
    *length = copy_len;
    
    comm->stats.bytes_received += copy_len;
    comm->stats.packets_received++;
    
    return BREEZE_SUCCESS;
}

static BreezeErrorCode mock_comm_get_stats(BreezeCommInterface* comm, BreezeCommStats* stats) {
    if (!comm || !stats) {
        return BREEZE_ERROR_INVALID_PARAM;
    }
    *stats = comm->stats;
    return BREEZE_SUCCESS;
}

/* Test setup function */
static void setup_test_comm_interface(void) {
    memset(&test_comm_interface, 0, sizeof(test_comm_interface));
    
    /* Setup mock operations */
    test_comm_interface.ops.init = mock_comm_init;
    test_comm_interface.ops.deinit = mock_comm_deinit;
    test_comm_interface.ops.send = mock_comm_send;
    test_comm_interface.ops.receive = mock_comm_receive;
    test_comm_interface.ops.get_stats = mock_comm_get_stats;
    
    /* Setup buffers */
    test_comm_interface.tx_buffer.data = test_tx_buffer_data;
    test_comm_interface.tx_buffer.size = sizeof(test_tx_buffer_data);
    test_comm_interface.rx_buffer.data = test_rx_buffer_data;
    test_comm_interface.rx_buffer.size = sizeof(test_rx_buffer_data);
    
    /* Reset mock flags */
    mock_init_called = 0;
    mock_deinit_called = 0;
    mock_send_called = 0;
    mock_receive_called = 0;
}

/* Test functions */

static void test_comm_interface_init_valid(void) {
    BreezeCommConfig config = {0};
    config.protocol = BREEZE_COMM_PROTOCOL_UART;
    config.mode = BREEZE_COMM_MODE_BLOCKING;
    config.baudrate = 115200;
    config.timeout_ms = 1000;
    
    setup_test_comm_interface();
    
    BreezeErrorCode result = breeze_comm_init(&test_comm_interface, &config);
    
    BREEZE_ASSERT_SUCCESS(result);
    BREEZE_ASSERT_EQUAL_INT(1, mock_init_called);
    BREEZE_ASSERT_EQUAL_INT(1, test_comm_interface.is_initialized);
    BREEZE_ASSERT_EQUAL_INT(BREEZE_COMM_PROTOCOL_UART, test_comm_interface.config.protocol);
    BREEZE_ASSERT_EQUAL_INT(115200, test_comm_interface.config.baudrate);
}

static void test_comm_interface_init_null_params(void) {
    BreezeCommConfig config = {0};
    
    setup_test_comm_interface();
    
    /* Test null comm interface */
    BreezeErrorCode result = breeze_comm_init(NULL, &config);
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_INVALID_PARAM, result);
    
    /* Test null config */
    result = breeze_comm_init(&test_comm_interface, NULL);
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_INVALID_PARAM, result);
}

static void test_comm_interface_init_already_initialized(void) {
    BreezeCommConfig config = {0};
    config.protocol = BREEZE_COMM_PROTOCOL_UART;
    
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1; /* Mark as already initialized */
    
    BreezeErrorCode result = breeze_comm_init(&test_comm_interface, &config);
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_ALREADY_INITIALIZED, result);
}

static void test_comm_interface_deinit_valid(void) {
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1;
    
    BreezeErrorCode result = breeze_comm_deinit(&test_comm_interface);
    
    BREEZE_ASSERT_SUCCESS(result);
    BREEZE_ASSERT_EQUAL_INT(1, mock_deinit_called);
    BREEZE_ASSERT_EQUAL_INT(0, test_comm_interface.is_initialized);
}

static void test_comm_interface_deinit_not_initialized(void) {
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 0;
    
    BreezeErrorCode result = breeze_comm_deinit(&test_comm_interface);
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_NOT_INITIALIZED, result);
}

static void test_comm_interface_send_valid(void) {
    const uint8_t test_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1;
    
    BreezeErrorCode result = breeze_comm_send(&test_comm_interface, test_data, sizeof(test_data));
    
    BREEZE_ASSERT_SUCCESS(result);
    BREEZE_ASSERT_EQUAL_INT(1, mock_send_called);
    BREEZE_ASSERT_EQUAL_INT(sizeof(test_data), test_comm_interface.stats.bytes_sent);
    BREEZE_ASSERT_EQUAL_INT(1, test_comm_interface.stats.packets_sent);
}

static void test_comm_interface_send_invalid_params(void) {
    const uint8_t test_data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1;
    
    /* Test null comm interface */
    BreezeErrorCode result = breeze_comm_send(NULL, test_data, sizeof(test_data));
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_INVALID_PARAM, result);
    
    /* Test null data */
    result = breeze_comm_send(&test_comm_interface, NULL, sizeof(test_data));
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_INVALID_PARAM, result);
    
    /* Test zero length */
    result = breeze_comm_send(&test_comm_interface, test_data, 0);
    BREEZE_ASSERT_ERROR_CODE(BREEZE_ERROR_INVALID_PARAM, result);
}

static void test_comm_interface_receive_valid(void) {
    uint8_t receive_buffer[16];
    size_t receive_length = sizeof(receive_buffer);
    
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1;
    
    BreezeErrorCode result = breeze_comm_receive(&test_comm_interface, receive_buffer, &receive_length);
    
    BREEZE_ASSERT_SUCCESS(result);
    BREEZE_ASSERT_EQUAL_INT(1, mock_receive_called);
    BREEZE_ASSERT_EQUAL_INT(4, receive_length); /* Mock returns 4 bytes */
    BREEZE_ASSERT_EQUAL_INT(0x01, receive_buffer[0]);
    BREEZE_ASSERT_EQUAL_INT(0x02, receive_buffer[1]);
    BREEZE_ASSERT_EQUAL_INT(0x03, receive_buffer[2]);
    BREEZE_ASSERT_EQUAL_INT(0x04, receive_buffer[3]);
}

static void test_comm_interface_get_stats(void) {
    BreezeCommStats stats;
    
    setup_test_comm_interface();
    test_comm_interface.is_initialized = 1;
    test_comm_interface.stats.bytes_sent = 100;
    test_comm_interface.stats.bytes_received = 50;
    test_comm_interface.stats.packets_sent = 10;
    test_comm_interface.stats.packets_received = 5;
    
    BreezeErrorCode result = breeze_comm_get_stats(&test_comm_interface, &stats);
    
    BREEZE_ASSERT_SUCCESS(result);
    BREEZE_ASSERT_EQUAL_INT(100, stats.bytes_sent);
    BREEZE_ASSERT_EQUAL_INT(50, stats.bytes_received);
    BREEZE_ASSERT_EQUAL_INT(10, stats.packets_sent);
    BREEZE_ASSERT_EQUAL_INT(5, stats.packets_received);
}

static void test_comm_protocol_enum_values(void) {
    BREEZE_ASSERT_EQUAL_INT(0, BREEZE_COMM_PROTOCOL_UART);
    BREEZE_ASSERT_EQUAL_INT(1, BREEZE_COMM_PROTOCOL_SPI);
    BREEZE_ASSERT_EQUAL_INT(2, BREEZE_COMM_PROTOCOL_I2C);
    BREEZE_ASSERT_EQUAL_INT(3, BREEZE_COMM_PROTOCOL_CAN);
    BREEZE_ASSERT_EQUAL_INT(4, BREEZE_COMM_PROTOCOL_ETHERNET);
    BREEZE_ASSERT_EQUAL_INT(5, BREEZE_COMM_PROTOCOL_USB);
    BREEZE_ASSERT_EQUAL_INT(99, BREEZE_COMM_PROTOCOL_CUSTOM);
}

static void test_comm_mode_enum_values(void) {
    BREEZE_ASSERT_EQUAL_INT(0, BREEZE_COMM_MODE_BLOCKING);
    BREEZE_ASSERT_EQUAL_INT(1, BREEZE_COMM_MODE_NON_BLOCKING);
    BREEZE_ASSERT_EQUAL_INT(2, BREEZE_COMM_MODE_INTERRUPT);
    BREEZE_ASSERT_EQUAL_INT(3, BREEZE_COMM_MODE_DMA);
}

/* Main test runner */
int main(void) {
    breeze_test_init();
    
    BREEZE_TEST_SUITE_BEGIN("Communication Interface Tests");
    
    BREEZE_RUN_TEST(test_comm_interface_init_valid);
    BREEZE_RUN_TEST(test_comm_interface_init_null_params);
    BREEZE_RUN_TEST(test_comm_interface_init_already_initialized);
    BREEZE_RUN_TEST(test_comm_interface_deinit_valid);
    BREEZE_RUN_TEST(test_comm_interface_deinit_not_initialized);
    BREEZE_RUN_TEST(test_comm_interface_send_valid);
    BREEZE_RUN_TEST(test_comm_interface_send_invalid_params);
    BREEZE_RUN_TEST(test_comm_interface_receive_valid);
    BREEZE_RUN_TEST(test_comm_interface_get_stats);
    BREEZE_RUN_TEST(test_comm_protocol_enum_values);
    BREEZE_RUN_TEST(test_comm_mode_enum_values);
    
    BREEZE_TEST_SUITE_END();
    
    breeze_test_summary();
    
    return (g_breeze_test_stats.tests_failed == 0) ? 0 : 1;
}