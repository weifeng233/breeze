/**
 * @file gaussian_blur.h
 * @brief 高斯模糊滤波器实现
 *
 * 该实现提供了用于图像平滑和降噪的高斯模糊滤波器。
 * 高斯滤波器是一种线性平滑滤波器，适用于抑制服从正态分布的噪声。
 */

#ifndef BREEZE_GAUSSIAN_BLUR_H
#define BREEZE_GAUSSIAN_BLUR_H

#include <stdlib.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建一维高斯核
 *
 * @param kernel 输出的高斯核数组
 * @param size 核的大小（必须是奇数）
 * @param sigma 高斯函数的标准差
 * @return 返回1表示成功，0表示失败
 */
static inline int BreezeGaussianKernel1D(
    float* kernel,
    int size,
    float sigma
) {
    int i;
    float sum = 0.0f;
    int half_size;
    
    if (!kernel || size <= 0 || !(size % 2) || sigma <= 0) return 0;
    
    half_size = size / 2;
    
    /* 计算高斯核 */
    for (i = 0; i < size; i++) {
        float x = (float)(i - half_size);
        kernel[i] = expf(-(x * x) / (2.0f * sigma * sigma));
        sum += kernel[i];
    }
    
    /* 归一化 */
    if (sum != 0) {
        for (i = 0; i < size; i++) {
            kernel[i] /= sum;
        }
    }
    
    return 1;
}

/**
 * @brief 对图像应用一维高斯滤波（水平方向）
 *
 * @param src 源图像数据
 * @param temp 临时缓冲区（与src大小相同）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 高斯核
 * @param kernel_size 核的大小
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeGaussianBlur1D_Horizontal(
    const unsigned char* src,
    unsigned char* temp,
    int width, int height,
    const float* kernel,
    int kernel_size,
    int stride_bytes
) {
    int x, y, i;
    int stride;
    int half_kernel;
    
    if (!src || !temp || width <= 0 || height <= 0 || !kernel || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    half_kernel = kernel_size / 2;
    
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            float sum = 0.0f;
            
            for (i = -half_kernel; i <= half_kernel; i++) {
                int src_x = x + i;
                
                /* 边界处理 */
                if (src_x < 0) src_x = 0;
                if (src_x >= width) src_x = width - 1;
                
                sum += (float)src[y * stride + src_x] * kernel[i + half_kernel];
            }
            
            /* 四舍五入并限制在0-255范围内 */
            int value = (int)(sum + 0.5f);
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            
            temp[y * stride + x] = (unsigned char)value;
        }
    }
}

/**
 * @brief 对图像应用一维高斯滤波（垂直方向）
 *
 * @param temp 临时图像数据（水平滤波的结果）
 * @param dst 目标图像数据
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 高斯核
 * @param kernel_size 核的大小
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeGaussianBlur1D_Vertical(
    const unsigned char* temp,
    unsigned char* dst,
    int width, int height,
    const float* kernel,
    int kernel_size,
    int stride_bytes
) {
    int x, y, i;
    int stride;
    int half_kernel;
    
    if (!temp || !dst || width <= 0 || height <= 0 || !kernel || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    half_kernel = kernel_size / 2;
    
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            float sum = 0.0f;
            
            for (i = -half_kernel; i <= half_kernel; i++) {
                int src_y = y + i;
                
                /* 边界处理 */
                if (src_y < 0) src_y = 0;
                if (src_y >= height) src_y = height - 1;
                
                sum += (float)temp[src_y * stride + x] * kernel[i + half_kernel];
            }
            
            /* 四舍五入并限制在0-255范围内 */
            int value = (int)(sum + 0.5f);
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            
            dst[y * stride + x] = (unsigned char)value;
        }
    }
}

/**
 * @brief 对图像应用高斯模糊
 *
 * @param src 源图像数据
 * @param dst 目标图像数据
 * @param width 图像宽度
 * @param height 图像高度
 * @param sigma 高斯函数的标准差
 * @param kernel_size 核的大小（如果为0，则自动计算）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeGaussianBlur(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    float sigma,
    int kernel_size,
    int stride_bytes
) {
    int stride;
    float* kernel;
    unsigned char* temp;
    
    if (!src || !dst || width <= 0 || height <= 0 || sigma <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 如果未指定核大小，则根据sigma自动计算 */
    if (kernel_size <= 0) {
        /* 核大小通常设置为sigma的6倍左右，并确保是奇数 */
        kernel_size = (int)(sigma * 6.0f + 0.5f);
        if (kernel_size % 2 == 0) kernel_size++;
        if (kernel_size < 3) kernel_size = 3;
    } else if (kernel_size % 2 == 0) {
        /* 确保核大小是奇数 */
        kernel_size++;
    }
    
    /* 分配内存 */
    kernel = (float*)malloc(kernel_size * sizeof(float));
    temp = (unsigned char*)malloc(stride * height);
    
    if (!kernel || !temp) {
        if (kernel) free(kernel);
        if (temp) free(temp);
        return;
    }
    
    /* 创建高斯核 */
    BreezeGaussianKernel1D(kernel, kernel_size, sigma);
    
    /* 应用可分离的高斯滤波（先水平后垂直） */
    BreezeGaussianBlur1D_Horizontal(src, temp, width, height, kernel, kernel_size, stride_bytes);
    BreezeGaussianBlur1D_Vertical(temp, dst, width, height, kernel, kernel_size, stride_bytes);
    
    /* 释放内存 */
    free(kernel);
    free(temp);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_GAUSSIAN_BLUR_H */
