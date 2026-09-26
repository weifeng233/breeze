/**
 * @file canny_edge.h
 * @brief Canny边缘检测算法实现
 *
 * 该实现提供了Canny边缘检测算法，这是一种多阶段的边缘检测算法，
 * 能够检测出图像中的边缘，同时抑制噪声，提供良好的边缘定位。
 */

#ifndef BREEZE_CANNY_EDGE_H
#define BREEZE_CANNY_EDGE_H

#include <stdlib.h>
#include <math.h>
#include "gaussian_blur.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算图像的梯度幅度和方向
 *
 * @param src 源图像数据（灰度）
 * @param magnitude 梯度幅度输出
 * @param direction 梯度方向输出（0-3，表示0°、45°、90°、135°）
 * @param width 图像宽度
 * @param height 图像高度
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeCannyGradient(
    const unsigned char* src,
    float* magnitude,
    unsigned char* direction,
    int width, int height,
    int stride_bytes
) {
    int x, y;
    int stride;
    
    if (!src || !magnitude || !direction || width <= 0 || height <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 计算梯度（跳过边界像素） */
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
            float gx = -src[p00] - 2*src[p10] - src[p20] + src[p02] + 2*src[p12] + src[p22];
            
            /* Sobel Y方向梯度 */
            float gy = -src[p00] - 2*src[p01] - src[p02] + src[p20] + 2*src[p21] + src[p22];
            
            /* 计算梯度幅度 */
            magnitude[y * width + x] = sqrtf(gx * gx + gy * gy);
            
            /* 计算梯度方向（量化为4个方向：0°、45°、90°、135°） */
            float angle = atan2f(gy, gx) * 180.0f / 3.14159265f;
            if (angle < 0) angle += 180.0f;
            
            /* 量化方向 */
            if ((angle >= 0 && angle < 22.5) || (angle >= 157.5 && angle <= 180)) {
                direction[y * width + x] = 0; /* 0度（水平） */
            } else if (angle >= 22.5 && angle < 67.5) {
                direction[y * width + x] = 1; /* 45度 */
            } else if (angle >= 67.5 && angle < 112.5) {
                direction[y * width + x] = 2; /* 90度（垂直） */
            } else {
                direction[y * width + x] = 3; /* 135度 */
            }
        }
    }
}

/**
 * @brief 非极大值抑制
 *
 * @param magnitude 梯度幅度
 * @param direction 梯度方向（0-3）
 * @param result 非极大值抑制结果
 * @param width 图像宽度
 * @param height 图像高度
 */
static inline void BreezeCannyNonMaxSuppression(
    const float* magnitude,
    const unsigned char* direction,
    float* result,
    int width, int height
) {
    int x, y;
    
    if (!magnitude || !direction || !result || width <= 0 || height <= 0) return;
    
    /* 初始化结果 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            result[y * width + x] = 0;
        }
    }
    
    /* 非极大值抑制（跳过边界像素） */
    for (y = 1; y < height - 1; y++) {
        for (x = 1; x < width - 1; x++) {
            int idx = y * width + x;
            float mag = magnitude[idx];
            float mag1 = 0, mag2 = 0;
            
            /* 根据梯度方向选择要比较的像素 */
            switch (direction[idx]) {
                case 0: /* 0度（水平） */
                    mag1 = magnitude[idx - 1];
                    mag2 = magnitude[idx + 1];
                    break;
                case 1: /* 45度 */
                    mag1 = magnitude[(y - 1) * width + (x + 1)];
                    mag2 = magnitude[(y + 1) * width + (x - 1)];
                    break;
                case 2: /* 90度（垂直） */
                    mag1 = magnitude[(y - 1) * width + x];
                    mag2 = magnitude[(y + 1) * width + x];
                    break;
                case 3: /* 135度 */
                    mag1 = magnitude[(y - 1) * width + (x - 1)];
                    mag2 = magnitude[(y + 1) * width + (x + 1)];
                    break;
            }
            
            /* 如果当前像素是局部最大值，则保留 */
            if (mag >= mag1 && mag >= mag2) {
                result[idx] = mag;
            }
        }
    }
}

/**
 * @brief 双阈值处理和滞后边缘跟踪
 *
 * @param nms 非极大值抑制结果
 * @param edges 边缘检测结果（二值图像）
 * @param width 图像宽度
 * @param height 图像高度
 * @param low_threshold 低阈值
 * @param high_threshold 高阈值
 */
static inline void BreezeCannyHysteresis(
    const float* nms,
    unsigned char* edges,
    int width, int height,
    float low_threshold,
    float high_threshold
) {
    int x, y, i, j;
    unsigned char* strong_edges;
    
    if (!nms || !edges || width <= 0 || height <= 0) return;
    
    /* 分配临时缓冲区用于存储强边缘 */
    strong_edges = (unsigned char*)calloc(width * height, sizeof(unsigned char));
    if (!strong_edges) return;
    
    /* 初始化边缘图像 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * width + x;
            float mag = nms[idx];
            
            if (mag >= high_threshold) {
                /* 强边缘 */
                edges[idx] = 255;
                strong_edges[idx] = 1;
            } else if (mag >= low_threshold) {
                /* 弱边缘 */
                edges[idx] = 128;
            } else {
                /* 非边缘 */
                edges[idx] = 0;
            }
        }
    }
    
    /* 滞后边缘跟踪 */
    for (y = 1; y < height - 1; y++) {
        for (x = 1; x < width - 1; x++) {
            int idx = y * width + x;
            
            /* 如果是强边缘，则检查8邻域中的弱边缘 */
            if (strong_edges[idx]) {
                for (j = -1; j <= 1; j++) {
                    for (i = -1; i <= 1; i++) {
                        if (i == 0 && j == 0) continue;
                        
                        int neighbor_idx = (y + j) * width + (x + i);
                        if (edges[neighbor_idx] == 128) {
                            /* 将弱边缘转换为强边缘 */
                            edges[neighbor_idx] = 255;
                            strong_edges[neighbor_idx] = 1;
                            /* 注意：这里需要重新检查这个新的强边缘的邻域 */
                            /* 为简化实现，我们依赖于下一次迭代 */
                        }
                    }
                }
            }
        }
    }
    
    /* 最终处理：将所有弱边缘设置为非边缘 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * width + x;
            if (edges[idx] == 128) {
                edges[idx] = 0;
            }
        }
    }
    
    free(strong_edges);
}

/**
 * @brief 应用Canny边缘检测算法
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（边缘）
 * @param width 图像宽度
 * @param height 图像高度
 * @param sigma 高斯滤波的标准差
 * @param low_threshold 低阈值
 * @param high_threshold 高阈值
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeCannyEdgeDetection(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    float sigma,
    float low_threshold,
    float high_threshold,
    int stride_bytes
) {
    int stride;
    unsigned char* blurred;
    float* magnitude;
    unsigned char* direction;
    float* nms_result;
    
    if (!src || !dst || width <= 0 || height <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 分配内存 */
    blurred = (unsigned char*)malloc(stride * height);
    magnitude = (float*)malloc(width * height * sizeof(float));
    direction = (unsigned char*)malloc(width * height);
    nms_result = (float*)malloc(width * height * sizeof(float));
    
    if (!blurred || !magnitude || !direction || !nms_result) {
        if (blurred) free(blurred);
        if (magnitude) free(magnitude);
        if (direction) free(direction);
        if (nms_result) free(nms_result);
        return;
    }
    
    /* 步骤1：高斯平滑 */
    BreezeGaussianBlur(src, blurred, width, height, sigma, 0, stride_bytes);
    
    /* 步骤2：计算梯度幅度和方向 */
    BreezeCannyGradient(blurred, magnitude, direction, width, height, stride_bytes);
    
    /* 步骤3：非极大值抑制 */
    BreezeCannyNonMaxSuppression(magnitude, direction, nms_result, width, height);
    
    /* 步骤4：双阈值处理和滞后边缘跟踪 */
    BreezeCannyHysteresis(nms_result, dst, width, height, low_threshold, high_threshold);
    
    /* 释放内存 */
    free(blurred);
    free(magnitude);
    free(direction);
    free(nms_result);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_CANNY_EDGE_H */
