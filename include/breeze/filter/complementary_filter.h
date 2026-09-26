/**
 * @file complementary_filter.h
 * @brief 用于从陀螺仪和加速度计数据估计姿态的互补滤波器
 *
 * 该实现提供了一个简单的互补滤波器，融合陀螺仪和加速度计数据
 * 来估计姿态（横滚角和俯仰角）。
 */

#ifndef BREEZE_COMPLEMENTARY_FILTER_H
#define BREEZE_COMPLEMENTARY_FILTER_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 互补滤波器状态结构体
 */
typedef struct {
    float roll;              /* 横滚角（弧度） */
    float pitch;             /* 俯仰角（弧度） */
    float alpha;             /* 滤波系数（0.0到1.0） */
    float dt;                /* 时间步长（秒） */
} BreezeComplementaryFilter;

/**
 * @brief 初始化互补滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）- 值越高越信任陀螺仪数据
 * @param dt 时间步长（秒）
 */
static inline void BreezeComplementaryFilter_Init(BreezeComplementaryFilter* filter, float alpha, float dt) {
    if (filter) {
        filter->roll = 0.0f;
        filter->pitch = 0.0f;
        filter->alpha = alpha;
        filter->dt = dt;
    }
}

/**
 * @brief 使用新的传感器数据更新滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param gyro_x 陀螺仪X轴角速度（弧度/秒）
 * @param gyro_y 陀螺仪Y轴角速度（弧度/秒）
 * @param gyro_z 陀螺仪Z轴角速度（弧度/秒）。本滤波器只估计横滚与俯仰
 *               （结构体里没有 yaw 字段），因此该参数不参与运算，
 *               保留它只是为了与三轴传感器的调用形式对称。
 * @param accel_x 加速度计X轴读数
 * @param accel_y 加速度计Y轴读数
 * @param accel_z 加速度计Z轴读数
 */
static inline void BreezeComplementaryFilter_Update(
    BreezeComplementaryFilter* filter,
    float gyro_x, float gyro_y, float gyro_z,
    float accel_x, float accel_y, float accel_z
) {
    float accel_roll, accel_pitch;
    float gyro_roll, gyro_pitch;

    if (!filter) return;

    (void)gyro_z;   /* 见参数说明：偏航不由互补滤波器估计 */

    /* 根据加速度计（重力向量）计算横滚角和俯仰角 */
    accel_roll = atan2f(accel_y, accel_z);
    accel_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z));

    /* 积分陀螺仪数据 */
    gyro_roll = filter->roll + gyro_x * filter->dt;
    gyro_pitch = filter->pitch + gyro_y * filter->dt;

    /* 互补滤波 - 结合加速度计和陀螺仪的估计值 */
    filter->roll = filter->alpha * gyro_roll + (1.0f - filter->alpha) * accel_roll;
    filter->pitch = filter->alpha * gyro_pitch + (1.0f - filter->alpha) * accel_pitch;
}

/**
 * @brief 获取当前横滚角
 *
 * @param filter 指向滤波器结构体的指针
 * @return 横滚角（弧度）
 */
static inline float BreezeComplementaryFilter_GetRoll(const BreezeComplementaryFilter* filter) {
    return filter ? filter->roll : 0.0f;
}

/**
 * @brief 获取当前俯仰角
 *
 * @param filter 指向滤波器结构体的指针
 * @return 俯仰角（弧度）
 */
static inline float BreezeComplementaryFilter_GetPitch(const BreezeComplementaryFilter* filter) {
    return filter ? filter->pitch : 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_COMPLEMENTARY_FILTER_H */
