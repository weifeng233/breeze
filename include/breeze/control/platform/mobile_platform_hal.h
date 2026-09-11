/**
 * @file mobile_platform_hal.h
 * @brief 移动平台控制的硬件抽象层（HAL）
 *
 * 该文件定义了用户需要实现的通用接口，以便使用移动平台控制算法。
 * 每种平台类型可能需要这些接口的特定实现。
 */

#ifndef BREEZE_MOBILE_PLATFORM_HAL_H
#define BREEZE_MOBILE_PLATFORM_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机控制函数类型
 *
 * 该函数类型用于设置电机的速度。
 * 实现应处理从归一化速度值到实际硬件控制信号的转换。
 *
 * @param motor_id 电机标识符（平台特定）
 * @param speed 归一化速度值（-1.0到1.0）
 */
typedef void (*BreezeMotorControlFunc)(int motor_id, float speed);

/**
 * @brief IMU数据结构
 *
 * 包含来自IMU传感器的陀螺仪和加速度计数据。
 */
typedef struct {
    float gyro_x;    /* 陀螺仪X轴角速度（弧度/秒） */
    float gyro_y;    /* 陀螺仪Y轴角速度（弧度/秒） */
    float gyro_z;    /* 陀螺仪Z轴角速度（弧度/秒） */
    float accel_x;   /* 加速度计X轴加速度（米/秒²） */
    float accel_y;   /* 加速度计Y轴加速度（米/秒²） */
    float accel_z;   /* 加速度计Z轴加速度（米/秒²） */
} BreezeIMUData;

/**
 * @brief IMU数据获取函数类型
 *
 * 该函数类型用于获取最新的IMU传感器数据。
 *
 * @param imu_data 指向要填充的IMU数据结构的指针
 * @return 成功返回1，失败返回0
 */
typedef int (*BreezeIMUDataFunc)(BreezeIMUData* imu_data);

/**
 * @brief 编码器数据获取函数类型
 *
 * 该函数类型用于获取最新的编码器计数或速度。
 *
 * @param encoder_id 编码器标识符（平台特定）
 * @param reset 读取后是否重置编码器计数的标志（1表示重置，0表示不重置）
 * @return 编码器计数或速度值（平台特定）
 */
typedef float (*BreezeEncoderFunc)(int encoder_id, int reset);

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_MOBILE_PLATFORM_HAL_H */
