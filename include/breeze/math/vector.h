/**
 * @file vector.h
 * @brief 向量运算实现
 *
 * 该实现提供了基本的向量运算功能，支持2D、3D和4D向量。
 * 包括向量加减乘、点积、叉积等操作。
 */

#ifndef BREEZE_VECTOR_H
#define BREEZE_VECTOR_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 2D向量结构体
 */
typedef struct {
    float x;
    float y;
} BreezeVector2D;

/**
 * @brief 3D向量结构体
 */
typedef struct {
    float x;
    float y;
    float z;
} BreezeVector3D;

/**
 * @brief 4D向量结构体
 */
typedef struct {
    float x;
    float y;
    float z;
    float w;
} BreezeVector4D;

/**
 * @brief 初始化2D向量
 *
 * @param vector 指向向量结构体的指针
 * @param x X分量
 * @param y Y分量
 */
static void BreezeVector2D_Init(BreezeVector2D* vector, float x, float y) {
    if (!vector) return;
    
    vector->x = x;
    vector->y = y;
}

/**
 * @brief 初始化3D向量
 *
 * @param vector 指向向量结构体的指针
 * @param x X分量
 * @param y Y分量
 * @param z Z分量
 */
static void BreezeVector3D_Init(BreezeVector3D* vector, float x, float y, float z) {
    if (!vector) return;
    
    vector->x = x;
    vector->y = y;
    vector->z = z;
}

/**
 * @brief 初始化4D向量
 *
 * @param vector 指向向量结构体的指针
 * @param x X分量
 * @param y Y分量
 * @param z Z分量
 * @param w W分量
 */
static void BreezeVector4D_Init(BreezeVector4D* vector, float x, float y, float z, float w) {
    if (!vector) return;
    
    vector->x = x;
    vector->y = y;
    vector->z = z;
    vector->w = w;
}

/**
 * @brief 2D向量加法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector2D_Add(BreezeVector2D* result, const BreezeVector2D* a, const BreezeVector2D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x + b->x;
    result->y = a->y + b->y;
}

/**
 * @brief 3D向量加法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector3D_Add(BreezeVector3D* result, const BreezeVector3D* a, const BreezeVector3D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
}

/**
 * @brief 4D向量加法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector4D_Add(BreezeVector4D* result, const BreezeVector4D* a, const BreezeVector4D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x + b->x;
    result->y = a->y + b->y;
    result->z = a->z + b->z;
    result->w = a->w + b->w;
}

/**
 * @brief 2D向量减法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector2D_Subtract(BreezeVector2D* result, const BreezeVector2D* a, const BreezeVector2D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x - b->x;
    result->y = a->y - b->y;
}

/**
 * @brief 3D向量减法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector3D_Subtract(BreezeVector3D* result, const BreezeVector3D* a, const BreezeVector3D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
}

/**
 * @brief 4D向量减法
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector4D_Subtract(BreezeVector4D* result, const BreezeVector4D* a, const BreezeVector4D* b) {
    if (!result || !a || !b) return;
    
    result->x = a->x - b->x;
    result->y = a->y - b->y;
    result->z = a->z - b->z;
    result->w = a->w - b->w;
}

/**
 * @brief 2D向量标量乘法
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @param scalar 标量值
 */
static void BreezeVector2D_ScalarMultiply(BreezeVector2D* result, const BreezeVector2D* vector, float scalar) {
    if (!result || !vector) return;
    
    result->x = vector->x * scalar;
    result->y = vector->y * scalar;
}

/**
 * @brief 3D向量标量乘法
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @param scalar 标量值
 */
static void BreezeVector3D_ScalarMultiply(BreezeVector3D* result, const BreezeVector3D* vector, float scalar) {
    if (!result || !vector) return;
    
    result->x = vector->x * scalar;
    result->y = vector->y * scalar;
    result->z = vector->z * scalar;
}

/**
 * @brief 4D向量标量乘法
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @param scalar 标量值
 */
static void BreezeVector4D_ScalarMultiply(BreezeVector4D* result, const BreezeVector4D* vector, float scalar) {
    if (!result || !vector) return;
    
    result->x = vector->x * scalar;
    result->y = vector->y * scalar;
    result->z = vector->z * scalar;
    result->w = vector->w * scalar;
}

/**
 * @brief 计算2D向量的点积
 *
 * @param a 输入向量A
 * @param b 输入向量B
 * @return 点积结果
 */
static float BreezeVector2D_DotProduct(const BreezeVector2D* a, const BreezeVector2D* b) {
    if (!a || !b) return 0.0f;
    
    return a->x * b->x + a->y * b->y;
}

/**
 * @brief 计算3D向量的点积
 *
 * @param a 输入向量A
 * @param b 输入向量B
 * @return 点积结果
 */
static float BreezeVector3D_DotProduct(const BreezeVector3D* a, const BreezeVector3D* b) {
    if (!a || !b) return 0.0f;
    
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

/**
 * @brief 计算4D向量的点积
 *
 * @param a 输入向量A
 * @param b 输入向量B
 * @return 点积结果
 */
static float BreezeVector4D_DotProduct(const BreezeVector4D* a, const BreezeVector4D* b) {
    if (!a || !b) return 0.0f;
    
    return a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w;
}

/**
 * @brief 计算3D向量的叉积
 *
 * @param result 结果向量
 * @param a 输入向量A
 * @param b 输入向量B
 */
static void BreezeVector3D_CrossProduct(BreezeVector3D* result, const BreezeVector3D* a, const BreezeVector3D* b) {
    BreezeVector3D temp;
    
    if (!result || !a || !b) return;
    
    /* 使用临时变量以支持原地操作 */
    temp.x = a->y * b->z - a->z * b->y;
    temp.y = a->z * b->x - a->x * b->z;
    temp.z = a->x * b->y - a->y * b->x;
    
    result->x = temp.x;
    result->y = temp.y;
    result->z = temp.z;
}

/**
 * @brief 计算2D向量的长度
 *
 * @param vector 输入向量
 * @return 向量长度
 */
static float BreezeVector2D_Length(const BreezeVector2D* vector) {
    if (!vector) return 0.0f;
    
    return sqrtf(vector->x * vector->x + vector->y * vector->y);
}

/**
 * @brief 计算3D向量的长度
 *
 * @param vector 输入向量
 * @return 向量长度
 */
static float BreezeVector3D_Length(const BreezeVector3D* vector) {
    if (!vector) return 0.0f;
    
    return sqrtf(vector->x * vector->x + vector->y * vector->y + vector->z * vector->z);
}

/**
 * @brief 计算4D向量的长度
 *
 * @param vector 输入向量
 * @return 向量长度
 */
static float BreezeVector4D_Length(const BreezeVector4D* vector) {
    if (!vector) return 0.0f;
    
    return sqrtf(vector->x * vector->x + vector->y * vector->y + 
                vector->z * vector->z + vector->w * vector->w);
}

/**
 * @brief 归一化2D向量
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @return 成功返回1，失败返回0
 */
static int BreezeVector2D_Normalize(BreezeVector2D* result, const BreezeVector2D* vector) {
    float length;
    
    if (!result || !vector) return 0;
    
    length = BreezeVector2D_Length(vector);
    
    if (length < 1e-6f) return 0;  /* 向量长度接近零 */
    
    result->x = vector->x / length;
    result->y = vector->y / length;
    
    return 1;
}

/**
 * @brief 归一化3D向量
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @return 成功返回1，失败返回0
 */
static int BreezeVector3D_Normalize(BreezeVector3D* result, const BreezeVector3D* vector) {
    float length;
    
    if (!result || !vector) return 0;
    
    length = BreezeVector3D_Length(vector);
    
    if (length < 1e-6f) return 0;  /* 向量长度接近零 */
    
    result->x = vector->x / length;
    result->y = vector->y / length;
    result->z = vector->z / length;
    
    return 1;
}

/**
 * @brief 归一化4D向量
 *
 * @param result 结果向量
 * @param vector 输入向量
 * @return 成功返回1，失败返回0
 */
static int BreezeVector4D_Normalize(BreezeVector4D* result, const BreezeVector4D* vector) {
    float length;
    
    if (!result || !vector) return 0;
    
    length = BreezeVector4D_Length(vector);
    
    if (length < 1e-6f) return 0;  /* 向量长度接近零 */
    
    result->x = vector->x / length;
    result->y = vector->y / length;
    result->z = vector->z / length;
    result->w = vector->w / length;
    
    return 1;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_VECTOR_H */
