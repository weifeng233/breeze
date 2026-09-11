/**
 * @file morphology.h
 * @brief 图像形态学操作实现
 *
 * 该实现提供了基本的图像形态学操作，包括膨胀、腐蚀、开运算和闭运算。
 * 形态学操作主要用于二值图像处理，可以用于去除噪声、连接断开的区域等。
 */

#ifndef BREEZE_MORPHOLOGY_H
#define BREEZE_MORPHOLOGY_H

#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建结构元素
 *
 * @param kernel 结构元素数组
 * @param size 结构元素大小（必须是奇数）
 * @param shape 结构元素形状（0=矩形，1=十字形，2=圆形）
 */
static void BreezeMorphologyCreateKernel(
    unsigned char* kernel,
    int size,
    int shape
) {
    int i, j;
    int half_size;
    
    if (!kernel || size <= 0 || !(size % 2)) return;
    
    half_size = size / 2;
    
    /* 初始化为0 */
    memset(kernel, 0, size * size);
    
    switch (shape) {
        case 0: /* 矩形 */
            for (i = 0; i < size; i++) {
                for (j = 0; j < size; j++) {
                    kernel[i * size + j] = 1;
                }
            }
            break;
            
        case 1: /* 十字形 */
            for (i = 0; i < size; i++) {
                kernel[i * size + half_size] = 1;  /* 垂直线 */
                kernel[half_size * size + i] = 1;  /* 水平线 */
            }
            break;
            
        case 2: /* 圆形 */
            for (i = 0; i < size; i++) {
                for (j = 0; j < size; j++) {
                    float dx = (float)(j - half_size);
                    float dy = (float)(i - half_size);
                    float distance = sqrtf(dx * dx + dy * dy);
                    
                    if (distance <= half_size) {
                        kernel[i * size + j] = 1;
                    }
                }
            }
            break;
            
        default:
            /* 默认为矩形 */
            for (i = 0; i < size; i++) {
                for (j = 0; j < size; j++) {
                    kernel[i * size + j] = 1;
                }
            }
            break;
    }
}

/**
 * @brief 对二值图像应用膨胀操作
 *
 * @param src 源图像数据（二值）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_size 结构元素大小（必须是奇数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMorphologyDilate(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    const unsigned char* kernel,
    int kernel_size,
    int stride_bytes
) {
    int x, y, i, j;
    int stride;
    int half_kernel;
    
    if (!src || !dst || !kernel || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    half_kernel = kernel_size / 2;
    
    /* 初始化目标图像为0 */
    memset(dst, 0, stride * height);
    
    /* 应用膨胀操作 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            /* 如果源像素为0，则跳过 */
            if (src[y * stride + x] == 0) continue;
            
            /* 应用结构元素 */
            for (j = -half_kernel; j <= half_kernel; j++) {
                for (i = -half_kernel; i <= half_kernel; i++) {
                    int kernel_idx = (j + half_kernel) * kernel_size + (i + half_kernel);
                    
                    /* 如果结构元素在此位置为0，则跳过 */
                    if (kernel[kernel_idx] == 0) continue;
                    
                    int dst_x = x + i;
                    int dst_y = y + j;
                    
                    /* 检查边界 */
                    if (dst_x >= 0 && dst_x < width && dst_y >= 0 && dst_y < height) {
                        dst[dst_y * stride + dst_x] = 255;
                    }
                }
            }
        }
    }
}

/**
 * @brief 对二值图像应用腐蚀操作
 *
 * @param src 源图像数据（二值）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_size 结构元素大小（必须是奇数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMorphologyErode(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    const unsigned char* kernel,
    int kernel_size,
    int stride_bytes
) {
    int x, y, i, j;
    int stride;
    int half_kernel;
    
    if (!src || !dst || !kernel || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    half_kernel = kernel_size / 2;
    
    /* 初始化目标图像为0 */
    memset(dst, 0, stride * height);
    
    /* 应用腐蚀操作 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int match = 1;
            
            /* 检查结构元素是否完全匹配 */
            for (j = -half_kernel; j <= half_kernel && match; j++) {
                for (i = -half_kernel; i <= half_kernel && match; i++) {
                    int kernel_idx = (j + half_kernel) * kernel_size + (i + half_kernel);
                    
                    /* 如果结构元素在此位置为0，则跳过 */
                    if (kernel[kernel_idx] == 0) continue;
                    
                    int src_x = x + i;
                    int src_y = y + j;
                    
                    /* 检查边界 */
                    if (src_x < 0 || src_x >= width || src_y < 0 || src_y >= height ||
                        src[src_y * stride + src_x] == 0) {
                        match = 0;
                    }
                }
            }
            
            /* 如果结构元素完全匹配，则设置目标像素为255 */
            if (match) {
                dst[y * stride + x] = 255;
            }
        }
    }
}

/**
 * @brief 对二值图像应用开运算（先腐蚀后膨胀）
 *
 * @param src 源图像数据（二值）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_size 结构元素大小（必须是奇数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMorphologyOpen(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    const unsigned char* kernel,
    int kernel_size,
    int stride_bytes
) {
    int stride;
    unsigned char* temp;
    
    if (!src || !dst || !kernel || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 分配临时缓冲区 */
    temp = (unsigned char*)malloc(stride * height);
    if (!temp) return;
    
    /* 先腐蚀 */
    BreezeMorphologyErode(src, temp, width, height, kernel, kernel_size, stride_bytes);
    
    /* 后膨胀 */
    BreezeMorphologyDilate(temp, dst, width, height, kernel, kernel_size, stride_bytes);
    
    free(temp);
}

/**
 * @brief 对二值图像应用闭运算（先膨胀后腐蚀）
 *
 * @param src 源图像数据（二值）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_size 结构元素大小（必须是奇数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMorphologyClose(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    const unsigned char* kernel,
    int kernel_size,
    int stride_bytes
) {
    int stride;
    unsigned char* temp;
    
    if (!src || !dst || !kernel || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 分配临时缓冲区 */
    temp = (unsigned char*)malloc(stride * height);
    if (!temp) return;
    
    /* 先膨胀 */
    BreezeMorphologyDilate(src, temp, width, height, kernel, kernel_size, stride_bytes);
    
    /* 后腐蚀 */
    BreezeMorphologyErode(temp, dst, width, height, kernel, kernel_size, stride_bytes);
    
    free(temp);
}

/**
 * @brief 对二值图像应用形态学梯度（膨胀减腐蚀）
 *
 * @param src 源图像数据（二值）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel 结构元素
 * @param kernel_size 结构元素大小（必须是奇数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMorphologyGradient(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    const unsigned char* kernel,
    int kernel_size,
    int stride_bytes
) {
    int x, y;
    int stride;
    unsigned char* dilated;
    unsigned char* eroded;
    
    if (!src || !dst || !kernel || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 分配临时缓冲区 */
    dilated = (unsigned char*)malloc(stride * height);
    eroded = (unsigned char*)malloc(stride * height);
    
    if (!dilated || !eroded) {
        if (dilated) free(dilated);
        if (eroded) free(eroded);
        return;
    }
    
    /* 膨胀和腐蚀 */
    BreezeMorphologyDilate(src, dilated, width, height, kernel, kernel_size, stride_bytes);
    BreezeMorphologyErode(src, eroded, width, height, kernel, kernel_size, stride_bytes);
    
    /* 计算梯度（膨胀减腐蚀） */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            int diff = dilated[idx] - eroded[idx];
            
            /* 确保结果在0-255范围内 */
            if (diff < 0) diff = 0;
            if (diff > 255) diff = 255;
            
            dst[idx] = (unsigned char)diff;
        }
    }
    
    free(dilated);
    free(eroded);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_MORPHOLOGY_H */
