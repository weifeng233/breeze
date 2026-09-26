/**
 * @file binary_threshold.h
 * @brief 图像处理的二值化阈值处理实现
 *
 * 该实现提供了灰度图像二值化阈值处理的函数。
 */

#ifndef BREEZE_BINARY_THRESHOLD_H
#define BREEZE_BINARY_THRESHOLD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 对灰度图像应用二值化阈值处理
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param threshold 阈值（0-255）
 * @param max_value 高于阈值的像素使用的最大值
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeBinaryThreshold(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    unsigned char threshold,
    unsigned char max_value,
    int stride_bytes
) {
    int x, y;
    int stride;

    if (!src || !dst || width <= 0 || height <= 0) return;

    stride = stride_bytes > 0 ? stride_bytes : width;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            dst[idx] = (src[idx] > threshold) ? max_value : 0;
        }
    }
}

/**
 * @brief 对灰度图像应用反向二值化阈值处理
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param threshold 阈值（0-255）
 * @param max_value 低于阈值的像素使用的最大值
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static inline void BreezeInverseBinaryThreshold(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    unsigned char threshold,
    unsigned char max_value,
    int stride_bytes
) {
    int x, y;
    int stride;

    if (!src || !dst || width <= 0 || height <= 0) return;

    stride = stride_bytes > 0 ? stride_bytes : width;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            dst[idx] = (src[idx] <= threshold) ? max_value : 0;
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_BINARY_THRESHOLD_H */
