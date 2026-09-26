/**
 * @file quaternion.h
 * @brief 四元数运算实现
 *
 * 该实现提供了四元数的基本运算功能，用于表示和操作3D旋转。
 * 包括四元数乘法、共轭、归一化以及与欧拉角的转换等操作。
 */

#ifndef BREEZE_QUATERNION_H
#define BREEZE_QUATERNION_H

#include <math.h>
#include "vector.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 四元数结构体
 */
typedef struct {
    float w;  /* 实部 */
    float x;  /* 虚部i */
    float y;  /* 虚部j */
    float z;  /* 虚部k */
} BreezeQuaternion;

/**
 * @brief 初始化四元数
 *
 * @param quaternion 指向四元数结构体的指针
 * @param w 实部
 * @param x 虚部i
 * @param y 虚部j
 * @param z 虚部k
 */
static inline void BreezeQuaternion_Init(BreezeQuaternion* quaternion, float w, float x, float y, float z) {
    if (!quaternion) return;
    
    quaternion->w = w;
    quaternion->x = x;
    quaternion->y = y;
    quaternion->z = z;
}

/**
 * @brief 设置四元数为单位四元数
 *
 * @param quaternion 指向四元数结构体的指针
 */
static inline void BreezeQuaternion_SetIdentity(BreezeQuaternion* quaternion) {
    if (!quaternion) return;
    
    quaternion->w = 1.0f;
    quaternion->x = 0.0f;
    quaternion->y = 0.0f;
    quaternion->z = 0.0f;
}

/**
 * @brief 从欧拉角（ZYX顺序，即航向-俯仰-横滚）创建四元数
 *
 * @param quaternion 指向四元数结构体的指针
 * @param roll 横滚角（弧度）
 * @param pitch 俯仰角（弧度）
 * @param yaw 航向角（弧度）
 */
static inline void BreezeQuaternion_FromEulerZYX(BreezeQuaternion* quaternion, float roll, float pitch, float yaw) {
    float cr, cp, cy, sr, sp, sy;
    float cpcy, spsy, cpsy, spcy;
    
    if (!quaternion) return;
    
    /* 计算一半角的正弦和余弦值 */
    cr = cosf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    cy = cosf(yaw * 0.5f);
    
    sr = sinf(roll * 0.5f);
    sp = sinf(pitch * 0.5f);
    sy = sinf(yaw * 0.5f);
    
    /* 计算四元数分量 */
    cpcy = cp * cy;
    spsy = sp * sy;
    cpsy = cp * sy;
    spcy = sp * cy;
    
    quaternion->w = cr * cpcy + sr * spsy;
    quaternion->x = sr * cpcy - cr * spsy;
    quaternion->y = cr * spcy + sr * cpsy;
    quaternion->z = cr * cpsy - sr * spcy;
}

/**
 * @brief 从轴角表示创建四元数
 *
 * @param quaternion 指向四元数结构体的指针
 * @param axis 旋转轴（单位向量）
 * @param angle 旋转角度（弧度）
 */
static inline void BreezeQuaternion_FromAxisAngle(BreezeQuaternion* quaternion, const BreezeVector3D* axis, float angle) {
    float half_angle;
    float sin_half_angle;
    BreezeVector3D normalized_axis;
    
    if (!quaternion || !axis) return;
    
    /* 归一化旋转轴 */
    if (!BreezeVector3D_Normalize(&normalized_axis, axis)) {
        BreezeQuaternion_SetIdentity(quaternion);
        return;
    }
    
    half_angle = angle * 0.5f;
    sin_half_angle = sinf(half_angle);
    
    quaternion->w = cosf(half_angle);
    quaternion->x = normalized_axis.x * sin_half_angle;
    quaternion->y = normalized_axis.y * sin_half_angle;
    quaternion->z = normalized_axis.z * sin_half_angle;
}

/**
 * @brief 将四元数转换为欧拉角（ZYX顺序）
 *
 * @param quaternion 输入四元数
 * @param roll 输出横滚角（弧度）
 * @param pitch 输出俯仰角（弧度）
 * @param yaw 输出航向角（弧度）
 */
static inline void BreezeQuaternion_ToEulerZYX(const BreezeQuaternion* quaternion, float* roll, float* pitch, float* yaw) {
    float sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;
    
    if (!quaternion || !roll || !pitch || !yaw) return;
    
    /* 计算俯仰角（pitch） */
    sinp = 2.0f * (quaternion->w * quaternion->y - quaternion->z * quaternion->x);
    
    /* 检查是否在万向节锁的情况（pitch = ±90度） */
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sinp);  /* 使用正确的符号 */
    } else {
        *pitch = asinf(sinp);
    }
    
    /* 计算横滚角（roll）和航向角（yaw） */
    if (fabsf(sinp) < 0.99999f) {  /* 避免万向节锁 */
        sinr_cosp = 2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
        cosr_cosp = 1.0f - 2.0f * (quaternion->x * quaternion->x + quaternion->y * quaternion->y);
        *roll = atan2f(sinr_cosp, cosr_cosp);
        
        siny_cosp = 2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y);
        cosy_cosp = 1.0f - 2.0f * (quaternion->y * quaternion->y + quaternion->z * quaternion->z);
        *yaw = atan2f(siny_cosp, cosy_cosp);
    } else {
        /* 万向节锁情况下，roll和yaw不能唯一确定，约定roll=0 */
        *roll = 0.0f;
        siny_cosp = 2.0f * (quaternion->x * quaternion->z - quaternion->w * quaternion->y);
        cosy_cosp = 1.0f - 2.0f * (quaternion->x * quaternion->x + quaternion->z * quaternion->z);
        *yaw = atan2f(siny_cosp, cosy_cosp);
    }
}

/**
 * @brief 计算四元数的模
 *
 * @param quaternion 输入四元数
 * @return 四元数的模
 */
static inline float BreezeQuaternion_Magnitude(const BreezeQuaternion* quaternion) {
    if (!quaternion) return 0.0f;
    
    return sqrtf(quaternion->w * quaternion->w + 
                quaternion->x * quaternion->x + 
                quaternion->y * quaternion->y + 
                quaternion->z * quaternion->z);
}

/**
 * @brief 归一化四元数
 *
 * @param result 结果四元数
 * @param quaternion 输入四元数
 * @return 成功返回1，失败返回0
 */
static inline int BreezeQuaternion_Normalize(BreezeQuaternion* result, const BreezeQuaternion* quaternion) {
    float magnitude;
    
    if (!result || !quaternion) return 0;
    
    magnitude = BreezeQuaternion_Magnitude(quaternion);
    
    if (magnitude < 1e-6f) return 0;  /* 四元数模接近零 */
    
    result->w = quaternion->w / magnitude;
    result->x = quaternion->x / magnitude;
    result->y = quaternion->y / magnitude;
    result->z = quaternion->z / magnitude;
    
    return 1;
}

/**
 * @brief 计算四元数的共轭
 *
 * @param result 结果四元数
 * @param quaternion 输入四元数
 */
static inline void BreezeQuaternion_Conjugate(BreezeQuaternion* result, const BreezeQuaternion* quaternion) {
    if (!result || !quaternion) return;
    
    result->w = quaternion->w;
    result->x = -quaternion->x;
    result->y = -quaternion->y;
    result->z = -quaternion->z;
}

/**
 * @brief 计算四元数的逆
 *
 * @param result 结果四元数
 * @param quaternion 输入四元数
 * @return 成功返回1，失败返回0
 */
static inline int BreezeQuaternion_Inverse(BreezeQuaternion* result, const BreezeQuaternion* quaternion) {
    float magnitude_squared;
    
    if (!result || !quaternion) return 0;
    
    magnitude_squared = quaternion->w * quaternion->w + 
                        quaternion->x * quaternion->x + 
                        quaternion->y * quaternion->y + 
                        quaternion->z * quaternion->z;
    
    if (magnitude_squared < 1e-6f) return 0;  /* 四元数模接近零 */
    
    /* 逆四元数 = 共轭四元数 / 模的平方 */
    result->w = quaternion->w / magnitude_squared;
    result->x = -quaternion->x / magnitude_squared;
    result->y = -quaternion->y / magnitude_squared;
    result->z = -quaternion->z / magnitude_squared;
    
    return 1;
}

/**
 * @brief 四元数乘法
 *
 * @param result 结果四元数
 * @param a 输入四元数A
 * @param b 输入四元数B
 */
static inline void BreezeQuaternion_Multiply(BreezeQuaternion* result, const BreezeQuaternion* a, const BreezeQuaternion* b) {
    BreezeQuaternion temp;
    
    if (!result || !a || !b) return;
    
    /* 使用临时变量以支持原地操作 */
    temp.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
    temp.x = a->w * b->x + a->x * b->w + a->y * b->z - a->z * b->y;
    temp.y = a->w * b->y - a->x * b->z + a->y * b->w + a->z * b->x;
    temp.z = a->w * b->z + a->x * b->y - a->y * b->x + a->z * b->w;
    
    *result = temp;
}

/**
 * @brief 使用四元数旋转向量
 *
 * @param result 结果向量
 * @param quaternion 旋转四元数（必须是单位四元数）
 * @param vector 输入向量
 */
static inline void BreezeQuaternion_RotateVector(BreezeVector3D* result, const BreezeQuaternion* quaternion, const BreezeVector3D* vector) {
    BreezeQuaternion vector_quaternion;
    BreezeQuaternion temp;
    BreezeQuaternion conjugate;
    BreezeQuaternion rotated;
    
    if (!result || !quaternion || !vector) return;
    
    /* 将向量转换为四元数（实部为0） */
    vector_quaternion.w = 0.0f;
    vector_quaternion.x = vector->x;
    vector_quaternion.y = vector->y;
    vector_quaternion.z = vector->z;
    
    /* 计算共轭四元数 */
    BreezeQuaternion_Conjugate(&conjugate, quaternion);
    
    /* 旋转公式：q * v * q^(-1)，其中q是单位四元数，q^(-1) = q* */
    BreezeQuaternion_Multiply(&temp, quaternion, &vector_quaternion);
    BreezeQuaternion_Multiply(&rotated, &temp, &conjugate);
    
    /* 提取旋转后的向量 */
    result->x = rotated.x;
    result->y = rotated.y;
    result->z = rotated.z;
}

/**
 * @brief 四元数球面线性插值（Slerp）
 *
 * @param result 结果四元数
 * @param a 起始四元数
 * @param b 结束四元数
 * @param t 插值参数（0.0到1.0）
 */
static inline void BreezeQuaternion_Slerp(BreezeQuaternion* result, const BreezeQuaternion* a, const BreezeQuaternion* b, float t) {
    float cos_half_theta;
    float half_theta;
    float sin_half_theta;
    float ratio_a, ratio_b;
    BreezeQuaternion q_b;
    
    if (!result || !a || !b) return;
    
    /* 限制t的范围 */
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    
    /* 计算四元数之间的夹角余弦 */
    cos_half_theta = a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z;
    
    /* 如果点积为负，使用-b而不是b，这样可以保证走最短路径 */
    if (cos_half_theta < 0.0f) {
        q_b.w = -b->w;
        q_b.x = -b->x;
        q_b.y = -b->y;
        q_b.z = -b->z;
        cos_half_theta = -cos_half_theta;
    } else {
        q_b = *b;
    }
    
    /* 如果四元数非常接近，使用线性插值 */
    if (cos_half_theta > 0.9999f) {
        ratio_a = 1.0f - t;
        ratio_b = t;
    } else {
        /* 否则使用球面线性插值 */
        half_theta = acosf(cos_half_theta);
        sin_half_theta = sinf(half_theta);
        
        ratio_a = sinf((1.0f - t) * half_theta) / sin_half_theta;
        ratio_b = sinf(t * half_theta) / sin_half_theta;
    }
    
    /* 计算插值结果 */
    result->w = ratio_a * a->w + ratio_b * q_b.w;
    result->x = ratio_a * a->x + ratio_b * q_b.x;
    result->y = ratio_a * a->y + ratio_b * q_b.y;
    result->z = ratio_a * a->z + ratio_b * q_b.z;
    
    /* 归一化结果 */
    BreezeQuaternion_Normalize(result, result);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_QUATERNION_H */
