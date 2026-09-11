/**
 * @file histogram.h
 * @brief 图像直方图处理实现
 *
 * 该实现提供了图像直方图计算和直方图均衡化功能，
 * 用于增强图像对比度和改善图像的视觉效果。
 */

#ifndef BREEZE_HISTOGRAM_H
#define BREEZE_HISTOGRAM_H

#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 计算灰度图像的直方图
 *
 * @param src 源图像数据（灰度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param histogram 输出直方图（256个元素的数组）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeHistogramCompute(
    const unsigned char* src,
    int width, int height,
    int* histogram,
    int stride_bytes
) {
    int x, y;
    int stride;
    
    if (!src || !histogram || width <= 0 || height <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 清空直方图 */
    memset(histogram, 0, 256 * sizeof(int));
    
    /* 计算直方图 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            histogram[src[y * stride + x]]++;
        }
    }
}

/**
 * @brief 计算灰度图像的累积直方图
 *
 * @param histogram 输入直方图（256个元素的数组）
 * @param cumulative_histogram 输出累积直方图（256个元素的数组）
 */
static void BreezeHistogramCumulative(
    const int* histogram,
    int* cumulative_histogram
) {
    int i;
    
    if (!histogram || !cumulative_histogram) return;
    
    cumulative_histogram[0] = histogram[0];
    
    for (i = 1; i < 256; i++) {
        cumulative_histogram[i] = cumulative_histogram[i - 1] + histogram[i];
    }
}

/**
 * @brief 对灰度图像应用直方图均衡化
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（灰度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeHistogramEqualization(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    int stride_bytes
) {
    int x, y;
    int stride;
    int histogram[256];
    int cumulative_histogram[256];
    int total_pixels;
    unsigned char lut[256];  /* 查找表 */
    
    if (!src || !dst || width <= 0 || height <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    total_pixels = width * height;
    
    /* 计算直方图 */
    BreezeHistogramCompute(src, width, height, histogram, stride_bytes);
    
    /* 计算累积直方图 */
    BreezeHistogramCumulative(histogram, cumulative_histogram);
    
    /* 创建查找表 */
    for (int i = 0; i < 256; i++) {
        /* 归一化累积直方图并缩放到0-255范围 */
        lut[i] = (unsigned char)(255.0f * cumulative_histogram[i] / total_pixels + 0.5f);
    }
    
    /* 应用查找表进行均衡化 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            dst[y * stride + x] = lut[src[y * stride + x]];
        }
    }
}

/**
 * @brief 对灰度图像应用局部直方图均衡化（CLAHE）
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（灰度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param tile_size 局部区域大小（必须是正数）
 * @param clip_limit 对比度限制（0表示无限制）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeHistogramEqualizationCLAHE(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    int tile_size,
    float clip_limit,
    int stride_bytes
) {
    int x, y, i, j, tx, ty;
    int stride;
    int tile_width, tile_height;
    int tile_count_x, tile_count_y;
    int*** histograms;
    unsigned char*** luts;
    
    if (!src || !dst || width <= 0 || height <= 0 || tile_size <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 确保tile_size不超过图像尺寸 */
    if (tile_size > width) tile_size = width;
    if (tile_size > height) tile_size = height;
    
    /* 计算每个区域的大小和区域数量 */
    tile_count_x = (width + tile_size - 1) / tile_size;
    tile_count_y = (height + tile_size - 1) / tile_size;
    tile_width = width / tile_count_x;
    tile_height = height / tile_count_y;
    
    /* 分配内存 */
    histograms = (int***)malloc(tile_count_y * sizeof(int**));
    luts = (unsigned char***)malloc(tile_count_y * sizeof(unsigned char**));
    
    if (!histograms || !luts) {
        if (histograms) free(histograms);
        if (luts) free(luts);
        return;
    }
    
    for (i = 0; i < tile_count_y; i++) {
        histograms[i] = (int**)malloc(tile_count_x * sizeof(int*));
        luts[i] = (unsigned char**)malloc(tile_count_x * sizeof(unsigned char*));
        
        if (!histograms[i] || !luts[i]) {
            /* 清理已分配的内存 */
            for (j = 0; j <= i; j++) {
                if (histograms[j]) free(histograms[j]);
                if (luts[j]) free(luts[j]);
            }
            if (histograms) free(histograms);
            if (luts) free(luts);
            return;
        }
        
        for (j = 0; j < tile_count_x; j++) {
            histograms[i][j] = (int*)calloc(256, sizeof(int));
            luts[i][j] = (unsigned char*)malloc(256 * sizeof(unsigned char));
            
            if (!histograms[i][j] || !luts[i][j]) {
                /* 清理已分配的内存 */
                for (int k = 0; k <= j; k++) {
                    if (histograms[i][k]) free(histograms[i][k]);
                    if (luts[i][k]) free(luts[i][k]);
                }
                for (int k = 0; k < i; k++) {
                    for (int l = 0; l < tile_count_x; l++) {
                        if (histograms[k][l]) free(histograms[k][l]);
                        if (luts[k][l]) free(luts[k][l]);
                    }
                    if (histograms[k]) free(histograms[k]);
                    if (luts[k]) free(luts[k]);
                }
                if (histograms) free(histograms);
                if (luts) free(luts);
                return;
            }
        }
    }
    
    /* 计算每个区域的直方图 */
    for (ty = 0; ty < tile_count_y; ty++) {
        for (tx = 0; tx < tile_count_x; tx++) {
            int* hist = histograms[ty][tx];
            int start_y = ty * tile_height;
            int end_y = (ty + 1) * tile_height;
            int start_x = tx * tile_width;
            int end_x = (tx + 1) * tile_width;
            
            /* 确保不超出图像边界 */
            if (end_y > height) end_y = height;
            if (end_x > width) end_x = width;
            
            /* 计算区域直方图 */
            for (y = start_y; y < end_y; y++) {
                for (x = start_x; x < end_x; x++) {
                    hist[src[y * stride + x]]++;
                }
            }
            
            /* 应用对比度限制 */
            if (clip_limit > 0) {
                int tile_pixels = (end_y - start_y) * (end_x - start_x);
                int clip_value = (int)(clip_limit * tile_pixels / 256.0f + 0.5f);
                int redistribution = 0;
                
                for (i = 0; i < 256; i++) {
                    if (hist[i] > clip_value) {
                        redistribution += (hist[i] - clip_value);
                        hist[i] = clip_value;
                    }
                }
                
                /* 重新分配超出的部分 */
                int redistribution_per_bin = redistribution / 256;
                for (i = 0; i < 256; i++) {
                    hist[i] += redistribution_per_bin;
                }
            }
            
            /* 计算累积直方图并创建查找表 */
            int cum_hist[256];
            cum_hist[0] = hist[0];
            
            for (i = 1; i < 256; i++) {
                cum_hist[i] = cum_hist[i - 1] + hist[i];
            }
            
            int tile_pixels = (end_y - start_y) * (end_x - start_x);
            for (i = 0; i < 256; i++) {
                luts[ty][tx][i] = (unsigned char)(255.0f * cum_hist[i] / tile_pixels + 0.5f);
            }
        }
    }
    
    /* 应用双线性插值进行均衡化 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            /* 计算像素所在的区域和相对位置 */
            float ty_f = (float)y / tile_height;
            float tx_f = (float)x / tile_width;
            int ty_i = (int)ty_f;
            int tx_i = (int)tx_f;
            float ty_alpha = ty_f - ty_i;
            float tx_alpha = tx_f - tx_i;
            
            /* 确保索引在有效范围内 */
            if (ty_i >= tile_count_y - 1) {
                ty_i = tile_count_y - 2;
                ty_alpha = 1.0f;
            }
            if (tx_i >= tile_count_x - 1) {
                tx_i = tile_count_x - 2;
                tx_alpha = 1.0f;
            }
            
            /* 获取四个相邻区域的查找表值 */
            unsigned char pixel_value = src[y * stride + x];
            unsigned char v00 = luts[ty_i][tx_i][pixel_value];
            unsigned char v01 = luts[ty_i][tx_i + 1][pixel_value];
            unsigned char v10 = luts[ty_i + 1][tx_i][pixel_value];
            unsigned char v11 = luts[ty_i + 1][tx_i + 1][pixel_value];
            
            /* 双线性插值 */
            float v0 = v00 * (1.0f - tx_alpha) + v01 * tx_alpha;
            float v1 = v10 * (1.0f - tx_alpha) + v11 * tx_alpha;
            float v = v0 * (1.0f - ty_alpha) + v1 * ty_alpha;
            
            /* 设置结果 */
            dst[y * stride + x] = (unsigned char)(v + 0.5f);
        }
    }
    
    /* 释放内存 */
    for (i = 0; i < tile_count_y; i++) {
        for (j = 0; j < tile_count_x; j++) {
            free(histograms[i][j]);
            free(luts[i][j]);
        }
        free(histograms[i]);
        free(luts[i]);
    }
    free(histograms);
    free(luts);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_HISTOGRAM_H */
