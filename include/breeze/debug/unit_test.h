/**
 * @file unit_test.h
 * @brief Lightweight unit testing framework for Breeze Framework
 *
 * This header provides a simple, lightweight testing framework with
 * assertion macros for validating Breeze Framework functionality.
 */

#ifndef BREEZE_UNIT_TEST_H
#define BREEZE_UNIT_TEST_H

#include "../core/error_codes.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default floating point tolerance for comparisons
 */
#ifndef BREEZE_TEST_FLOAT_TOLERANCE
#define BREEZE_TEST_FLOAT_TOLERANCE 1e-6f
#endif

/**
 * @brief Test statistics structure
 */
typedef struct {
    int tests_run;      /**< Total number of tests run */
    int tests_passed;   /**< Number of tests passed */
    int tests_failed;   /**< Number of tests failed */
    int assertions_run; /**< Total number of assertions */
    int assertions_passed; /**< Number of assertions passed */
    int assertions_failed; /**< Number of assertions failed */
} BreezeTestStats;

/**
 * @brief Global test statistics
 */
extern BreezeTestStats g_breeze_test_stats;

/**
 * @brief Test function type
 */
typedef void (*BreezeTestFunction)(void);

/**
 * @brief Initialize test framework
 */
static inline void breeze_test_init(void) {
    g_breeze_test_stats.tests_run = 0;
    g_breeze_test_stats.tests_passed = 0;
    g_breeze_test_stats.tests_failed = 0;
    g_breeze_test_stats.assertions_run = 0;
    g_breeze_test_stats.assertions_passed = 0;
    g_breeze_test_stats.assertions_failed = 0;
}

/**
 * @brief Print test results summary
 */
static inline void breeze_test_summary(void) {
    printf("\n=== Breeze Test Summary ===\n");
    printf("Tests: %d run, %d passed, %d failed\n", 
           g_breeze_test_stats.tests_run,
           g_breeze_test_stats.tests_passed,
           g_breeze_test_stats.tests_failed);
    printf("Assertions: %d run, %d passed, %d failed\n",
           g_breeze_test_stats.assertions_run,
           g_breeze_test_stats.assertions_passed,
           g_breeze_test_stats.assertions_failed);
    printf("Success rate: %.1f%%\n", 
           g_breeze_test_stats.tests_run > 0 ? 
           (100.0f * g_breeze_test_stats.tests_passed / g_breeze_test_stats.tests_run) : 0.0f);
    printf("==========================\n");
}

/**
 * @brief Internal assertion implementation
 */
static inline void breeze_assert_impl(int condition, const char* expression, 
                                     const char* file, int line, const char* message) {
    g_breeze_test_stats.assertions_run++;
    
    if (condition) {
        g_breeze_test_stats.assertions_passed++;
    } else {
        g_breeze_test_stats.assertions_failed++;
        printf("ASSERTION FAILED: %s:%d\n", file, line);
        printf("  Expression: %s\n", expression);
        if (message) {
            printf("  Message: %s\n", message);
        }
    }
}

/**
 * @brief Run a test function
 */
static inline void breeze_run_test(const char* test_name, BreezeTestFunction test_func) {
    int assertions_before = g_breeze_test_stats.assertions_failed;
    
    printf("Running test: %s... ", test_name);
    fflush(stdout);
    
    g_breeze_test_stats.tests_run++;
    
    if (test_func) {
        test_func();
    }
    
    if (g_breeze_test_stats.assertions_failed == assertions_before) {
        g_breeze_test_stats.tests_passed++;
        printf("PASSED\n");
    } else {
        g_breeze_test_stats.tests_failed++;
        printf("FAILED\n");
    }
}

/* Basic assertion macros */

/**
 * @brief Assert that condition is true
 */
#define BREEZE_ASSERT(condition) \
    breeze_assert_impl((condition), #condition, __FILE__, __LINE__, NULL)

/**
 * @brief Assert that condition is true with custom message
 */
#define BREEZE_ASSERT_MSG(condition, message) \
    breeze_assert_impl((condition), #condition, __FILE__, __LINE__, (message))

/**
 * @brief Assert that two integers are equal
 */
#define BREEZE_ASSERT_EQUAL_INT(expected, actual) \
    breeze_assert_impl((expected) == (actual), \
                      "Expected: " #expected ", Actual: " #actual, \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that two floats are equal within tolerance
 */
#define BREEZE_ASSERT_EQUAL_FLOAT(expected, actual, tolerance) \
    breeze_assert_impl(fabsf((expected) - (actual)) <= (tolerance), \
                      "Expected: " #expected ", Actual: " #actual, \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that two floats are equal within default tolerance
 */
#define BREEZE_ASSERT_EQUAL_FLOAT_DEFAULT(expected, actual) \
    BREEZE_ASSERT_EQUAL_FLOAT(expected, actual, BREEZE_TEST_FLOAT_TOLERANCE)

/**
 * @brief Assert that two doubles are equal within tolerance
 */
#define BREEZE_ASSERT_EQUAL_DOUBLE(expected, actual, tolerance) \
    breeze_assert_impl(fabs((expected) - (actual)) <= (tolerance), \
                      "Expected: " #expected ", Actual: " #actual, \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that pointer is not NULL
 */
#define BREEZE_ASSERT_NOT_NULL(ptr) \
    breeze_assert_impl((ptr) != NULL, #ptr " should not be NULL", \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that pointer is NULL
 */
#define BREEZE_ASSERT_NULL(ptr) \
    breeze_assert_impl((ptr) == NULL, #ptr " should be NULL", \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that error code indicates success
 */
#define BREEZE_ASSERT_SUCCESS(error_code) \
    breeze_assert_impl(breeze_is_success(error_code), \
                      "Expected success, got: " #error_code, \
                      __FILE__, __LINE__, breeze_error_string(error_code))

/**
 * @brief Assert that error code indicates failure
 */
#define BREEZE_ASSERT_ERROR(error_code) \
    breeze_assert_impl(breeze_is_error(error_code), \
                      "Expected error, got success", \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that specific error code is returned
 */
#define BREEZE_ASSERT_ERROR_CODE(expected_error, actual_error) \
    breeze_assert_impl((expected_error) == (actual_error), \
                      "Expected error: " #expected_error ", Actual: " #actual_error, \
                      __FILE__, __LINE__, breeze_error_string(actual_error))

/**
 * @brief Assert that two strings are equal
 */
#define BREEZE_ASSERT_EQUAL_STRING(expected, actual) \
    breeze_assert_impl(strcmp((expected), (actual)) == 0, \
                      "Expected: " #expected ", Actual: " #actual, \
                      __FILE__, __LINE__, NULL)

/**
 * @brief Assert that array elements are equal
 */
#define BREEZE_ASSERT_EQUAL_ARRAY_INT(expected, actual, size) \
    do { \
        int i; \
        int arrays_equal = 1; \
        for (i = 0; i < (size); i++) { \
            if ((expected)[i] != (actual)[i]) { \
                arrays_equal = 0; \
                break; \
            } \
        } \
        breeze_assert_impl(arrays_equal, \
                          "Arrays " #expected " and " #actual " should be equal", \
                          __FILE__, __LINE__, NULL); \
    } while(0)

/**
 * @brief Assert that float array elements are equal within tolerance
 */
#define BREEZE_ASSERT_EQUAL_ARRAY_FLOAT(expected, actual, size, tolerance) \
    do { \
        int i; \
        int arrays_equal = 1; \
        for (i = 0; i < (size); i++) { \
            if (fabsf((expected)[i] - (actual)[i]) > (tolerance)) { \
                arrays_equal = 0; \
                break; \
            } \
        } \
        breeze_assert_impl(arrays_equal, \
                          "Arrays " #expected " and " #actual " should be equal", \
                          __FILE__, __LINE__, NULL); \
    } while(0)

/**
 * @brief Test suite macros
 */
#define BREEZE_TEST_SUITE_BEGIN(suite_name) \
    printf("\n=== Test Suite: %s ===\n", suite_name);

#define BREEZE_TEST_SUITE_END() \
    printf("=== End Test Suite ===\n");

/**
 * @brief Run test macro
 */
#define BREEZE_RUN_TEST(test_func) \
    breeze_run_test(#test_func, test_func)

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_UNIT_TEST_H */