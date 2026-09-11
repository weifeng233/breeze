/**
 * @file breeze.h
 * @brief Breeze框架的主头文件
 *
 * 该头文件包含了Breeze框架的所有模块。
 * 
 * 框架扩展后包含以下模块：
 * - 核心模块：错误处理、配置管理
 * - 通信模块：UART、SPI、I2C等通信协议
 * - 传感器融合模块：卡尔曼滤波、GPS工具等
 * - RTOS工具模块：任务管理、同步原语等
 * - 电源管理模块：功耗监控、睡眠管理等
 * - 安全模块：加密、哈希、随机数生成等
 * - 存储模块：文件系统、闪存管理等
 * - 网络模块：TCP/UDP、HTTP、MQTT等
 * - 音频处理模块：滤波、编解码、分析等
 * - 机器学习模块：神经网络、分类、聚类等
 * - 调试模块：单元测试、日志、性能分析等
 */

#ifndef BREEZE_H
#define BREEZE_H

/* 核心模块 - Core Infrastructure */
#include "core/error_codes.h"
#include "core/config.h"

/* 通信协议模块 - Communication Protocols */
#include "comm/comm_interface.h"
#include "comm/comm_hal.h"
#include "comm/comm_buffer.h"
#include "comm/uart.h"
/* Future communication modules will be added here:
 * #include "comm/spi.h"
 * #include "comm/i2c.h"
 * #include "comm/can.h"
 */

/* 调试和测试模块 - Testing and Debugging */
#include "debug/unit_test.h"
/* Future debug modules will be added here:
 * #include "debug/logger.h"
 * #include "debug/profiler.h"
 */

/* 现有模块 - Existing Modules */

/* 滤波模块 */
#include "filter/complementary_filter.h"
#include "filter/kalman_filter.h"
#include "filter/low_pass_filter.h"
#include "filter/high_pass_filter.h"
#include "filter/median_filter.h"

/* 控制模块 */
#include "control/pid_controller.h"
#include "control/fuzzy_controller.h"
#include "control/adaptive_controller.h"
#include "control/state_feedback_controller.h"

/* 移动平台控制模块 */
#include "control/platform/mobile_platform_hal.h"
#include "control/platform/differential_drive.h"
#include "control/platform/ackermann_steering.h"
#include "control/platform/mecanum_drive.h"
#include "control/platform/omni_drive.h"
#include "control/platform/balance_controller.h"

/* 图像处理模块 */
#include "image/binary_threshold.h"
#include "image/otsu_threshold.h"
#include "image/sobel_operator.h"
#include "image/gaussian_blur.h"
#include "image/canny_edge.h"
#include "image/hough_transform.h"
#include "image/morphology.h"
#include "image/histogram.h"

/* 数学工具模块 */
#include "math/matrix.h"
#include "math/vector.h"
#include "math/quaternion.h"
#include "math/interpolation.h"

/* 未来模块占位符 - Future Module Placeholders */
/*
 * 以下模块将在后续任务中添加：
 * 
 * 传感器融合模块 - Sensor Fusion and Data Processing
 * #include "sensor/extended_kalman_filter.h"
 * #include "sensor/gps_utils.h"
 * #include "sensor/sensor_calibration.h"
 * 
 * RTOS工具模块 - RTOS Utilities and Abstractions  
 * #include "rtos/task_manager.h"
 * #include "rtos/synchronization.h"
 * #include "rtos/memory_pool.h"
 * 
 * 电源管理模块 - Power Management
 * #include "power/power_monitor.h"
 * #include "power/sleep_manager.h"
 * #include "power/frequency_scaling.h"
 * 
 * 安全和加密模块 - Security and Cryptography
 * #include "security/aes_crypto.h"
 * #include "security/hash_functions.h"
 * #include "security/random_generator.h"
 * 
 * 存储和文件系统模块 - Storage and File System
 * #include "storage/fat_filesystem.h"
 * #include "storage/flash_manager.h"
 * #include "storage/data_logger.h"
 * 
 * 网络协议栈模块 - Network Stack
 * #include "network/tcp_udp.h"
 * #include "network/http_client.h"
 * #include "network/mqtt_client.h"
 * 
 * 音频处理模块 - Audio Processing
 * #include "audio/audio_filters.h"
 * #include "audio/audio_codecs.h"
 * #include "audio/frequency_analysis.h"
 * 
 * 机器学习模块 - Machine Learning and AI
 * #include "ml/neural_network.h"
 * #include "ml/classifiers.h"
 * #include "ml/clustering.h"
 */

#endif /* BREEZE_H */
