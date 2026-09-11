/**
 * @file sobel_operator.h
 * @brief 用于边缘检测的Sobel算子
 *
 * 该实现提供了用于灰度图像边缘检测的Sobel算子。
 */

#ifndef BREEZE_SOBEL_OPERATOR_H
#define BREEZE_SOBEL_OPERATOR_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 应用Sobel算子进行边缘检测
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（边缘幅度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeSobelOperator(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    int stride_bytes
) {
    int x, y;
    int stride;

    if (!src || !dst || width <= 0 || height <= 0) return;

    stride = stride_bytes > 0 ? stride_bytes : width;

    /* 清空目标图像 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            dst[y * stride + x] = 0;
        }
    }

    /* 应用Sobel算子（跳过边界像素） */
    for (y = 1; y < height - 1; y++) {
        for (x = 1; x < width - 1; x++) {
            /* 3x3邻域的像素索引 */
            int p00 = (y-1) * stride + (x-1);
            int p01 = (y-1) * stride + (x);
            int p02 = (y-1) * stride + (x+1);
            int p10 = (y)   * stride + (x-1);
            int p12 = (y)   * stride + (x+1);
            int p20 = (y+1) * stride + (x-1);
            int p21 = (y+1) * stride + (x);
            int p22 = (y+1) * stride + (x+1);

            /* Sobel X方向梯度: -1 0 1
             *                   -2 0 2
             *                   -1 0 1
             */
            int gx = -src[p00] - 2*src[p10] - src[p20] + src[p02] + 2*src[p12] + src[p22];

            /* Sobel Y方向梯度: -1 -2 -1
             *                    0  0  0
             *                    1  2  1
             */
            int gy = -src[p00] - 2*src[p01] - src[p02] + src[p20] + 2*src[p21] + src[p22];

            /* 计算梯度幅度（不使用平方根的近似计算） */
            int magnitude = (abs(gx) + abs(gy)) / 2;

            /* 限制在0-255范围内 */
            if (magnitude > 255) magnitude = 255;
            if (magnitude < 0) magnitude = 0;

            dst[y * stride + x] = (unsigned char)magnitude;
        }
    }
}

/**
 * @brief 应用带有梯度方向的Sobel算子
 *
 * @param src 源图像数据（灰度）
 * @param magnitude 边缘幅度的目标图像
 * @param direction 边缘方向的目标图像（弧度 * 128/PI）
 * @param width 图像宽度
 * @param height 图像高度
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeSobelOperatorWithDirection(
    const unsigned char* src,
    unsigned char* magnitude,
    unsigned char* direction,
    int width, int height,
    int stride_bytes
) {
    int x, y;
    int stride;

    if (!src || !magnitude || !direction || width <= 0 || height <= 0) return;

    stride = stride_bytes > 0 ? stride_bytes : width;

    /* 清空目标图像 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            magnitude[y * stride + x] = 0;
            direction[y * stride + x] = 0;
        }
    }

    /* 应用Sobel算子（跳过边界像素） */
    for (y = 1; y < height - 1; y++) {
        for (x = 1; x < width - 1; x++) {
            /* 3x3邻域的像素索引 */
            int p00 = (y-1) * stride + (x-1);
            int p01 = (y-1) * stride + (x);
            int p02 = (y-1) * stride + (x+1);
            int p10 = (y)   * stride + (x-1);
            int p12 = (y)   * stride + (x+1);
            int p20 = (y+1) * stride + (x-1);
            int p21 = (y+1) * stride + (x);
            int p22 = (y+1) * stride + (x+1);

            /* Sobel X方向梯度 */
            int gx = -src[p00] - 2*src[p10] - src[p20] + src[p02] + 2*src[p12] + src[p22];

            /* Sobel Y方向梯度 */
            int gy = -src[p00] - 2*src[p01] - src[p02] + src[p20] + 2*src[p21] + src[p22];

            /* 计算梯度幅度 */
            int mag = (abs(gx) + abs(gy)) / 2;

            /* 计算梯度方向（缩放到0-255范围） */
            unsigned char dir = 0;
            if (gx != 0 || gy != 0) {
                float angle = atan2f((float)gy, (float)gx);
                /* 将角度从[-PI, PI]缩放到[0, 255] */
                dir = (unsigned char)((angle + 3.14159265f) * 128.0f / 3.14159265f);
            }

            /* 将幅度限制在0-255范围内 */
            if (mag > 255) mag = 255;
            if (mag < 0) mag = 0;

            magnitude[y * stride + x] = (unsigned char)mag;
            direction[y * stride + x] = dir;
        }
    }
}

/**
 * @brief 应用带有阈值的Sobel算子
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（二值边缘）
 * @param width 图像宽度
 * @param height 图像高度
 * @param threshold 边缘检测的阈值
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeSobelOperatorThreshold(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    unsigned char threshold,
    int stride_bytes
) {
    int x, y;
    int stride;
    unsigned char* temp_buffer;

    if (!src || !dst || width <= 0 || height <= 0) return;

    stride = stride_bytes > 0 ? stride_bytes : width;

    /* 为边缘幅度分配临时缓冲区 */
    temp_buffer = (unsigned char*)malloc(stride * height);
    if (!temp_buffer) return;

    /* 应用Sobel算子 */
    BreezeSobelOperator(src, temp_buffer, width, height, stride_bytes);

    /* 应用阈值 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            dst[idx] = (temp_buffer[idx] > threshold) ? 255 : 0;
        }
    }

    /* 释放临时缓冲区 */
    free(temp_buffer);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_SOBEL_OPERATOR_H */
