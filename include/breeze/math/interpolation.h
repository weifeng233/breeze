/**
 * @file interpolation.h
 * @brief 插值算法实现
 *
 * 该实现提供了各种插值算法，包括线性插值、三次样条插值和贝塞尔曲线插值。
 * 插值算法用于在已知数据点之间估计中间值。
 */

#ifndef BREEZE_INTERPOLATION_H
#define BREEZE_INTERPOLATION_H

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "vector.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 线性插值
 *
 * @param x0 起始点x坐标
 * @param y0 起始点y坐标
 * @param x1 结束点x坐标
 * @param y1 结束点y坐标
 * @param x 插值点x坐标
 * @return 插值点y坐标
 */
static inline float BreezeInterpolation_Linear(float x0, float y0, float x1, float y1, float x) {
    if (fabsf(x1 - x0) < 1e-6f) return (y0 + y1) * 0.5f;
    
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

/**
 * @brief 余弦插值
 *
 * @param y0 起始点y坐标
 * @param y1 结束点y坐标
 * @param mu 插值参数（0.0到1.0）
 * @return 插值点y坐标
 */
static inline float BreezeInterpolation_Cosine(float y0, float y1, float mu) {
    float mu2;
    
    mu2 = (1.0f - cosf(mu * M_PI)) * 0.5f;
    return y0 * (1.0f - mu2) + y1 * mu2;
}

/**
 * @brief 三次Hermite插值
 *
 * @param y0 起始点前一个点的y坐标
 * @param y1 起始点y坐标
 * @param y2 结束点y坐标
 * @param y3 结束点后一个点的y坐标
 * @param mu 插值参数（0.0到1.0）
 * @param tension 张力参数（0.0为自然曲线）
 * @param bias 偏置参数（0.0为无偏置）
 * @return 插值点y坐标
 */
static inline float BreezeInterpolation_CubicHermite(
    float y0, float y1, float y2, float y3,
    float mu, float tension, float bias
) {
    float m0, m1;
    float mu2, mu3;
    float a0, a1, a2, a3;
    
    mu2 = mu * mu;
    mu3 = mu2 * mu;
    
    /* 计算切线 */
    m0 = (y2 - y0) * (1.0f + bias) * (1.0f - tension) * 0.5f;
    m0 += (y3 - y1) * (1.0f - bias) * (1.0f - tension) * 0.5f;
    
    m1 = (y3 - y1) * (1.0f + bias) * (1.0f - tension) * 0.5f;
    m1 += (y2 - y0) * (1.0f - bias) * (1.0f - tension) * 0.5f;
    
    /* 计算多项式系数 */
    a0 = 2.0f * mu3 - 3.0f * mu2 + 1.0f;
    a1 = mu3 - 2.0f * mu2 + mu;
    a2 = mu3 - mu2;
    a3 = -2.0f * mu3 + 3.0f * mu2;
    
    /* 计算插值结果 */
    return a0 * y1 + a1 * m0 + a2 * m1 + a3 * y2;
}

/**
 * @brief 三次样条插值结构体
 */
typedef struct {
    float* x;          /* x坐标数组 */
    float* y;          /* y坐标数组 */
    float* a;          /* 样条系数a */
    float* b;          /* 样条系数b */
    float* c;          /* 样条系数c */
    float* d;          /* 样条系数d */
    int n;             /* 数据点数量 */
    int allocated;     /* 是否分配了内存 */
} BreezeSplineInterpolation;

/*
 * 前置声明：Init 在失败路径上要调用 Free，而 Free 的定义在本文件后面。
 * 缺了它，C 会把 Init 里那次调用当成隐式声明（返回 int），随后 Free 的真
 * 定义就变成 "static declaration follows non-static declaration" —— 硬错误。
 */
static inline void BreezeSplineInterpolation_Free(BreezeSplineInterpolation* spline);

/**
 * @brief 初始化三次样条插值
 *
 * @param spline 指向样条插值结构体的指针
 * @param x x坐标数组
 * @param y y坐标数组
 * @param n 数据点数量
 * @param allocate 是否分配内存（1表示分配，0表示使用传入的数组）
 * @return 成功返回1，失败返回0
 */
static inline int BreezeSplineInterpolation_Init(
    BreezeSplineInterpolation* spline,
    float* x, float* y, int n,
    int allocate
) {
    int i;
    float* h;
    float* alpha;
    float* l;
    float* mu;
    float* z;
    
    if (!spline || !x || !y || n < 2) return 0;
    
    spline->n = n;
    spline->allocated = allocate;
    
    if (allocate) {
        /* 分配内存 */
        spline->x = (float*)malloc(n * sizeof(float));
        spline->y = (float*)malloc(n * sizeof(float));
        
        if (!spline->x || !spline->y) {
            if (spline->x) free(spline->x);
            if (spline->y) free(spline->y);
            return 0;
        }
        
        /* 复制数据 */
        for (i = 0; i < n; i++) {
            spline->x[i] = x[i];
            spline->y[i] = y[i];
        }
    } else {
        /* 直接使用传入的数组 */
        spline->x = x;
        spline->y = y;
    }
    
    /* 分配系数数组 */
    spline->a = (float*)malloc(n * sizeof(float));
    spline->b = (float*)malloc(n * sizeof(float));
    spline->c = (float*)malloc(n * sizeof(float));
    spline->d = (float*)malloc(n * sizeof(float));
    
    if (!spline->a || !spline->b || !spline->c || !spline->d) {
        BreezeSplineInterpolation_Free(spline);
        return 0;
    }
    
    /* 分配临时数组 */
    h = (float*)malloc((n - 1) * sizeof(float));
    alpha = (float*)malloc((n - 1) * sizeof(float));
    l = (float*)malloc(n * sizeof(float));
    mu = (float*)malloc(n * sizeof(float));
    z = (float*)malloc(n * sizeof(float));
    
    if (!h || !alpha || !l || !mu || !z) {
        if (h) free(h);
        if (alpha) free(alpha);
        if (l) free(l);
        if (mu) free(mu);
        if (z) free(z);
        BreezeSplineInterpolation_Free(spline);
        return 0;
    }
    
    /* 计算h和alpha */
    for (i = 0; i < n - 1; i++) {
        h[i] = x[i + 1] - x[i];
        if (h[i] < 1e-6f) {
            free(h);
            free(alpha);
            free(l);
            free(mu);
            free(z);
            BreezeSplineInterpolation_Free(spline);
            return 0;  /* x值必须严格递增 */
        }
    }
    
    for (i = 1; i < n - 1; i++) {
        alpha[i] = 3.0f * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1]);
    }
    
    /* 三对角矩阵算法 */
    l[0] = 1.0f;
    mu[0] = 0.0f;
    z[0] = 0.0f;
    
    for (i = 1; i < n - 1; i++) {
        l[i] = 2.0f * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    
    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    spline->c[n - 1] = 0.0f;
    
    /* 回代求解 */
    for (i = n - 2; i >= 0; i--) {
        spline->c[i] = z[i] - mu[i] * spline->c[i + 1];
        spline->b[i] = (y[i + 1] - y[i]) / h[i] - h[i] * (spline->c[i + 1] + 2.0f * spline->c[i]) / 3.0f;
        spline->d[i] = (spline->c[i + 1] - spline->c[i]) / (3.0f * h[i]);
    }
    
    /* 设置a系数 */
    for (i = 0; i < n - 1; i++) {
        spline->a[i] = y[i];
    }
    
    /* 释放临时数组 */
    free(h);
    free(alpha);
    free(l);
    free(mu);
    free(z);
    
    return 1;
}

/**
 * @brief 释放三次样条插值资源
 *
 * @param spline 指向样条插值结构体的指针
 */
static inline void BreezeSplineInterpolation_Free(BreezeSplineInterpolation* spline) {
    if (!spline) return;
    
    if (spline->allocated) {
        if (spline->x) free(spline->x);
        if (spline->y) free(spline->y);
    }
    
    if (spline->a) free(spline->a);
    if (spline->b) free(spline->b);
    if (spline->c) free(spline->c);
    if (spline->d) free(spline->d);
    
    spline->x = NULL;
    spline->y = NULL;
    spline->a = NULL;
    spline->b = NULL;
    spline->c = NULL;
    spline->d = NULL;
    spline->n = 0;
    spline->allocated = 0;
}

/**
 * @brief 使用三次样条插值计算y值
 *
 * @param spline 指向样条插值结构体的指针
 * @param x 插值点x坐标
 * @return 插值点y坐标
 */
static inline float BreezeSplineInterpolation_Evaluate(const BreezeSplineInterpolation* spline, float x) {
    int i;
    float dx;
    
    if (!spline || !spline->x || !spline->y || !spline->a || !spline->b || !spline->c || !spline->d || spline->n < 2) {
        return 0.0f;
    }
    
    /* 处理边界情况 */
    if (x <= spline->x[0]) return spline->y[0];
    if (x >= spline->x[spline->n - 1]) return spline->y[spline->n - 1];
    
    /* 查找x所在的区间 */
    for (i = 0; i < spline->n - 1; i++) {
        if (x < spline->x[i + 1]) break;
    }
    
    /* 计算插值 */
    dx = x - spline->x[i];
    return spline->a[i] + spline->b[i] * dx + spline->c[i] * dx * dx + spline->d[i] * dx * dx * dx;
}

/**
 * @brief 贝塞尔曲线点结构体
 */
typedef struct {
    float x;
    float y;
} BreezeBezierPoint;

/**
 * @brief 计算二次贝塞尔曲线点
 *
 * @param p0 起始点
 * @param p1 控制点
 * @param p2 结束点
 * @param t 参数（0.0到1.0）
 * @param result 结果点
 */
static inline void BreezeBezierCurve_Quadratic(
    const BreezeBezierPoint* p0,
    const BreezeBezierPoint* p1,
    const BreezeBezierPoint* p2,
    float t,
    BreezeBezierPoint* result
) {
    float t1 = 1.0f - t;
    float t1_squared = t1 * t1;
    float t_squared = t * t;
    float t1_t_2 = 2.0f * t1 * t;
    
    if (!p0 || !p1 || !p2 || !result) return;
    
    result->x = t1_squared * p0->x + t1_t_2 * p1->x + t_squared * p2->x;
    result->y = t1_squared * p0->y + t1_t_2 * p1->y + t_squared * p2->y;
}

/**
 * @brief 计算三次贝塞尔曲线点
 *
 * @param p0 起始点
 * @param p1 第一控制点
 * @param p2 第二控制点
 * @param p3 结束点
 * @param t 参数（0.0到1.0）
 * @param result 结果点
 */
static inline void BreezeBezierCurve_Cubic(
    const BreezeBezierPoint* p0,
    const BreezeBezierPoint* p1,
    const BreezeBezierPoint* p2,
    const BreezeBezierPoint* p3,
    float t,
    BreezeBezierPoint* result
) {
    float t1 = 1.0f - t;
    float t1_squared = t1 * t1;
    float t1_cubed = t1_squared * t1;
    float t_squared = t * t;
    float t_cubed = t_squared * t;
    float t1_squared_t = 3.0f * t1_squared * t;
    float t1_t_squared = 3.0f * t1 * t_squared;
    
    if (!p0 || !p1 || !p2 || !p3 || !result) return;
    
    result->x = t1_cubed * p0->x + t1_squared_t * p1->x + t1_t_squared * p2->x + t_cubed * p3->x;
    result->y = t1_cubed * p0->y + t1_squared_t * p1->y + t1_t_squared * p2->y + t_cubed * p3->y;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_INTERPOLATION_H */
