/**
 * @file hough_transform.h
 * @brief 霍夫变换算法实现
 *
 * 该实现提供了霍夫变换算法，用于检测图像中的直线和圆。
 * 霍夫变换是一种特征提取技术，常用于图像分析和计算机视觉。
 */

#ifndef BREEZE_HOUGH_TRANSFORM_H
#define BREEZE_HOUGH_TRANSFORM_H

#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 霍夫直线结构体
 */
typedef struct {
    float rho;      /* 原点到直线的距离 */
    float theta;    /* 直线的角度（弧度） */
    int votes;      /* 投票数（累加器值） */
} BreezeHoughLine;

/**
 * @brief 霍夫圆结构体
 */
typedef struct {
    int x;          /* 圆心x坐标 */
    int y;          /* 圆心y坐标 */
    int radius;     /* 半径 */
    int votes;      /* 投票数（累加器值） */
} BreezeHoughCircle;

/**
 * @brief 应用霍夫直线变换
 *
 * @param src 源图像数据（二值边缘图像）
 * @param width 图像宽度
 * @param height 图像高度
 * @param lines 检测到的直线数组
 * @param max_lines 最大直线数量
 * @param threshold 检测阈值（最小投票数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 * @return 检测到的直线数量
 */
static inline int BreezeHoughLines(
    const unsigned char* src,
    int width, int height,
    BreezeHoughLine* lines,
    int max_lines,
    int threshold,
    int stride_bytes
) {
    int stride;
    int diagonal;
    int rho_count, theta_count;
    int x, y, i, j;
    int line_count = 0;
    int* accumulator = NULL;
    float rho_step, theta_step;
    
    if (!src || !lines || width <= 0 || height <= 0 || max_lines <= 0) return 0;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    /* 计算图像对角线长度（最大可能的rho值） */
    diagonal = (int)ceilf(sqrtf((float)(width * width + height * height)));
    
    /* 设置累加器大小 */
    rho_count = diagonal * 2 + 1;  /* -diagonal 到 +diagonal */
    theta_count = 180;             /* 0 到 179度 */
    
    /* 分配累加器内存 */
    accumulator = (int*)calloc(rho_count * theta_count, sizeof(int));
    if (!accumulator) return 0;
    
    /* 计算步长 */
    rho_step = 1.0f;
    theta_step = 3.14159265f / 180.0f;  /* 弧度制 */
    
    /* 填充累加器 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            if (src[y * stride + x] > 0) {  /* 如果是边缘点 */
                for (i = 0; i < theta_count; i++) {
                    float theta = i * theta_step;
                    float rho = x * cosf(theta) + y * sinf(theta);
                    int rho_idx = (int)(rho + diagonal);
                    
                    if (rho_idx >= 0 && rho_idx < rho_count) {
                        accumulator[rho_idx * theta_count + i]++;
                    }
                }
            }
        }
    }
    
    /* 查找局部最大值 */
    for (i = 1; i < rho_count - 1 && line_count < max_lines; i++) {
        for (j = 1; j < theta_count - 1 && line_count < max_lines; j++) {
            int idx = i * theta_count + j;
            int value = accumulator[idx];
            
            /* 如果超过阈值且是局部最大值 */
            if (value > threshold) {
                int is_max = 1;
                int ni, nj;
                
                /* 检查3x3邻域 */
                for (ni = -1; ni <= 1 && is_max; ni++) {
                    for (nj = -1; nj <= 1 && is_max; nj++) {
                        if (ni == 0 && nj == 0) continue;
                        
                        int neighbor_idx = (i + ni) * theta_count + (j + nj);
                        if (accumulator[neighbor_idx] > value) {
                            is_max = 0;
                        }
                    }
                }
                
                if (is_max) {
                    lines[line_count].rho = (i - diagonal) * rho_step;
                    lines[line_count].theta = j * theta_step;
                    lines[line_count].votes = value;
                    line_count++;
                }
            }
        }
    }
    
    free(accumulator);
    return line_count;
}

/**
 * @brief 在图像上绘制霍夫直线
 *
 * @param dst 目标图像数据
 * @param width 图像宽度
 * @param height 图像高度
 * @param line 要绘制的直线
 * @param color 线条颜色
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeDrawHoughLine(
    unsigned char* dst,
    int width, int height,
    const BreezeHoughLine* line,
    unsigned char color,
    int stride_bytes
) {
    int stride;
    int x0, y0, x1, y1;
    int x, y;
    float cos_theta, sin_theta;
    
    if (!dst || !line || width <= 0 || height <= 0) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    
    cos_theta = cosf(line->theta);
    sin_theta = sinf(line->theta);
    
    /* 计算直线与图像边界的交点 */
    if (fabsf(sin_theta) < 0.001f) {
        /* 垂直线 */
        x0 = x1 = (int)(line->rho / cos_theta);
        y0 = 0;
        y1 = height - 1;
    } else if (fabsf(cos_theta) < 0.001f) {
        /* 水平线 */
        y0 = y1 = (int)(line->rho / sin_theta);
        x0 = 0;
        x1 = width - 1;
    } else {
        /* 斜线 */
        x0 = 0;
        y0 = (int)(line->rho / sin_theta);
        x1 = width - 1;
        y1 = (int)((line->rho - x1 * cos_theta) / sin_theta);
        
        /* 如果线不在图像范围内，则调整端点 */
        if (y0 < 0 || y0 >= height) {
            if (y0 < 0) {
                y0 = 0;
                x0 = (int)((line->rho - y0 * sin_theta) / cos_theta);
            } else {
                y0 = height - 1;
                x0 = (int)((line->rho - y0 * sin_theta) / cos_theta);
            }
        }
        
        if (y1 < 0 || y1 >= height) {
            if (y1 < 0) {
                y1 = 0;
                x1 = (int)((line->rho - y1 * sin_theta) / cos_theta);
            } else {
                y1 = height - 1;
                x1 = (int)((line->rho - y1 * sin_theta) / cos_theta);
            }
        }
    }
    
    /* 使用Bresenham算法绘制直线 */
    {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int e2;
        
        x = x0;
        y = y0;
        
        while (1) {
            if (x >= 0 && x < width && y >= 0 && y < height) {
                dst[y * stride + x] = color;
            }
            
            if (x == x1 && y == y1) break;
            
            e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
}

/**
 * @brief 应用霍夫圆变换
 *
 * @param src 源图像数据（二值边缘图像）
 * @param width 图像宽度
 * @param height 图像高度
 * @param circles 检测到的圆数组
 * @param max_circles 最大圆数量
 * @param min_radius 最小半径
 * @param max_radius 最大半径
 * @param threshold 检测阈值（最小投票数）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 * @return 检测到的圆数量
 */
static inline int BreezeHoughCircles(
    const unsigned char* src,
    int width, int height,
    BreezeHoughCircle* circles,
    int max_circles,
    int min_radius,
    int max_radius,
    int threshold,
    int stride_bytes
) {
    int stride;
    int x, y, r, i, j;
    int circle_count = 0;
    int radius_count;
    int*** accumulator = NULL;
    
    if (!src || !circles || width <= 0 || height <= 0 || max_circles <= 0 ||
        min_radius < 0 || max_radius <= min_radius) return 0;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    radius_count = max_radius - min_radius + 1;
    
    /* 分配3D累加器内存 */
    accumulator = (int***)malloc(width * sizeof(int**));
    if (!accumulator) return 0;
    
    for (i = 0; i < width; i++) {
        accumulator[i] = (int**)malloc(height * sizeof(int*));
        if (!accumulator[i]) {
            for (j = 0; j < i; j++) {
                free(accumulator[j]);
            }
            free(accumulator);
            return 0;
        }
        
        for (j = 0; j < height; j++) {
            accumulator[i][j] = (int*)calloc(radius_count, sizeof(int));
            if (!accumulator[i][j]) {
                for (int k = 0; k < j; k++) {
                    free(accumulator[i][k]);
                }
                for (int k = 0; k < i; k++) {
                    for (int l = 0; l < height; l++) {
                        free(accumulator[k][l]);
                    }
                    free(accumulator[k]);
                }
                free(accumulator);
                return 0;
            }
        }
    }
    
    /* 填充累加器 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            if (src[y * stride + x] > 0) {  /* 如果是边缘点 */
                for (r = min_radius; r <= max_radius; r++) {
                    /* 对于每个可能的半径，在圆周上投票 */
                    for (int angle = 0; angle < 360; angle += 5) {  /* 每5度采样一次 */
                        float rad = angle * 3.14159265f / 180.0f;
                        int a = (int)(x - r * cosf(rad));
                        int b = (int)(y - r * sinf(rad));
                        
                        if (a >= 0 && a < width && b >= 0 && b < height) {
                            accumulator[a][b][r - min_radius]++;
                        }
                    }
                }
            }
        }
    }
    
    /* 查找局部最大值 */
    for (x = 1; x < width - 1 && circle_count < max_circles; x++) {
        for (y = 1; y < height - 1 && circle_count < max_circles; y++) {
            for (r = 0; r < radius_count && circle_count < max_circles; r++) {
                int value = accumulator[x][y][r];
                
                /* 如果超过阈值且是局部最大值 */
                if (value > threshold) {
                    int is_max = 1;
                    
                    /* 检查3x3x3邻域 */
                    for (int dx = -1; dx <= 1 && is_max; dx++) {
                        for (int dy = -1; dy <= 1 && is_max; dy++) {
                            for (int dr = -1; dr <= 1 && is_max; dr++) {
                                if (dx == 0 && dy == 0 && dr == 0) continue;
                                
                                int nx = x + dx;
                                int ny = y + dy;
                                int nr = r + dr;
                                
                                if (nx >= 0 && nx < width && ny >= 0 && ny < height && 
                                    nr >= 0 && nr < radius_count) {
                                    if (accumulator[nx][ny][nr] > value) {
                                        is_max = 0;
                                    }
                                }
                            }
                        }
                    }
                    
                    if (is_max) {
                        circles[circle_count].x = x;
                        circles[circle_count].y = y;
                        circles[circle_count].radius = r + min_radius;
                        circles[circle_count].votes = value;
                        circle_count++;
                    }
                }
            }
        }
    }
    
    /* 释放累加器内存 */
    for (i = 0; i < width; i++) {
        for (j = 0; j < height; j++) {
            free(accumulator[i][j]);
        }
        free(accumulator[i]);
    }
    free(accumulator);
    
    return circle_count;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_HOUGH_TRANSFORM_H */
