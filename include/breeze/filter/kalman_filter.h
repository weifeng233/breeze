/**
 * @file kalman_filter.h
 * @brief 卡尔曼滤波器实现
 *
 * 该实现提供了一个简单的一维卡尔曼滤波器，用于状态估计和传感器融合。
 * 卡尔曼滤波器是一种递归的状态估计算法，可以从含有噪声的测量中估计动态系统的状态。
 */

#ifndef BREEZE_KALMAN_FILTER_H
#define BREEZE_KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一维卡尔曼滤波器状态结构体
 */
typedef struct {
    float x;           /* 状态估计值 */
    float p;           /* 估计误差协方差 */
    float q;           /* 过程噪声协方差 */
    float r;           /* 测量噪声协方差 */
    float k;           /* 卡尔曼增益 */
    float a;           /* 状态转移系数 */
    float h;           /* 测量系数 */
} BreezeKalmanFilter1D;

/**
 * @brief 初始化一维卡尔曼滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param q 过程噪声协方差（系统动态不确定性）
 * @param r 测量噪声协方差（传感器噪声）
 * @param p_init 初始估计误差协方差
 * @param x_init 初始状态估计值
 */
static inline void BreezeKalmanFilter1D_Init(
    BreezeKalmanFilter1D* filter,
    float q, float r,
    float p_init, float x_init
) {
    if (filter) {
        filter->x = x_init;
        filter->p = p_init;
        filter->q = q;
        filter->r = r;
        filter->k = 0.0f;
        filter->a = 1.0f;  /* 默认为1，表示状态保持不变 */
        filter->h = 1.0f;  /* 默认为1，表示测量值直接对应状态 */
    }
}

/**
 * @brief 设置状态转移系数
 *
 * @param filter 指向滤波器结构体的指针
 * @param a 状态转移系数
 */
static inline void BreezeKalmanFilter1D_SetStateTransition(BreezeKalmanFilter1D* filter, float a) {
    if (filter) {
        filter->a = a;
    }
}

/**
 * @brief 设置测量系数
 *
 * @param filter 指向滤波器结构体的指针
 * @param h 测量系数
 */
static inline void BreezeKalmanFilter1D_SetMeasurementCoefficient(BreezeKalmanFilter1D* filter, float h) {
    if (filter) {
        filter->h = h;
    }
}

/**
 * @brief 更新卡尔曼滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param measurement 新的测量值
 * @return 更新后的状态估计值
 */
static inline float BreezeKalmanFilter1D_Update(BreezeKalmanFilter1D* filter, float measurement) {
    if (!filter) return 0.0f;

    /* 预测步骤 */
    filter->x = filter->a * filter->x;
    filter->p = filter->a * filter->a * filter->p + filter->q;

    /* 更新步骤 */
    filter->k = filter->p * filter->h / (filter->h * filter->p * filter->h + filter->r);
    filter->x = filter->x + filter->k * (measurement - filter->h * filter->x);
    filter->p = (1.0f - filter->k * filter->h) * filter->p;

    return filter->x;
}

/**
 * @brief 获取当前状态估计值
 *
 * @param filter 指向滤波器结构体的指针
 * @return 当前状态估计值
 */
static inline float BreezeKalmanFilter1D_GetState(const BreezeKalmanFilter1D* filter) {
    return filter ? filter->x : 0.0f;
}

/**
 * @brief 获取当前估计误差协方差
 *
 * @param filter 指向滤波器结构体的指针
 * @return 当前估计误差协方差
 */
static inline float BreezeKalmanFilter1D_GetCovariance(const BreezeKalmanFilter1D* filter) {
    return filter ? filter->p : 0.0f;
}

/**
 * @brief 获取当前卡尔曼增益
 *
 * @param filter 指向滤波器结构体的指针
 * @return 当前卡尔曼增益
 */
static inline float BreezeKalmanFilter1D_GetGain(const BreezeKalmanFilter1D* filter) {
    return filter ? filter->k : 0.0f;
}

/**
 * @brief 多维卡尔曼滤波器状态结构体（最大支持4维）
 */
typedef struct {
    float x[4];        /* 状态向量 */
    float p[4][4];     /* 状态协方差矩阵 */
    float q[4][4];     /* 过程噪声协方差矩阵 */
    float r[4][4];     /* 测量噪声协方差矩阵 */
    float k[4][4];     /* 卡尔曼增益矩阵 */
    float a[4][4];     /* 状态转移矩阵 */
    float h[4][4];     /* 测量矩阵 */
    int dim;           /* 状态向量维度 */
    int meas_dim;      /* 测量向量维度 */
} BreezeKalmanFilterND;

/* 注意：多维卡尔曼滤波器的实现需要矩阵运算支持，将在后续的数学工具模块中添加 */

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_KALMAN_FILTER_H */
