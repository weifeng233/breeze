/**
 * @file otsu_threshold.h
 * @brief 大津法自动阈值处理
 *
 * 该实现提供了大津法（Otsu's method）用于自动确定
 * 二值图像分割的最佳阈值。
 */

#ifndef BREEZE_OTSU_THRESHOLD_H
#define BREEZE_OTSU_THRESHOLD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 使用大津法计算最佳阈值
 *
 * @param src 源图像数据（灰度）
 * @param width 图像宽度
 * @param height 图像高度
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 * @return 最佳阈值（0-255）
 */
static inline unsigned char BreezeOtsuThreshold(
    const unsigned char* src,
    int width, int height,
    int stride_bytes
) {
    int histogram[256] = {0};
    int x, y, i;
    int stride;
    int total_pixels;
    float sum = 0;
    float sumB = 0;
    float wB = 0;
    float wF = 0;
    float mB, mF;
    float max_variance = 0;
    unsigned char threshold = 0;

    if (!src || width <= 0 || height <= 0) return 0;

    stride = stride_bytes > 0 ? stride_bytes : width;
    total_pixels = width * height;

    /* 计算直方图和总和 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            histogram[src[idx]]++;
            sum += src[idx];
        }
    }

    /* 对每个可能的阈值，计算类间方差 */
    for (i = 0; i < 256; i++) {
        wB += histogram[i];                /* 背景权重 */
        if (wB == 0) continue;

        wF = total_pixels - wB;            /* 前景权重 */
        if (wF == 0) break;

        sumB += i * histogram[i];

        mB = sumB / wB;                    /* 背景均值 */
        mF = (sum - sumB) / wF;            /* 前景均值 */

        /* 计算类间方差 */
        float variance = wB * wF * (mB - mF) * (mB - mF);

        /* 如果方差更高则更新阈值 */
        if (variance > max_variance) {
            max_variance = variance;
            threshold = i;
        }
    }

    return threshold;
}

/**
 * @brief 对灰度图像应用大津法阈值处理
 *
 * @param src 源图像数据（灰度）
 * @param dst 目标图像数据（二值）
 * @param width 图像宽度
 * @param height 图像高度
 * @param max_value 高于阈值的像素使用的最大值
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 * @return 使用的阈值
 */
static inline unsigned char BreezeApplyOtsuThreshold(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    unsigned char max_value,
    int stride_bytes
) {
    int x, y;
    int stride;
    unsigned char threshold;

    if (!src || !dst || width <= 0 || height <= 0) return 0;

    stride = stride_bytes > 0 ? stride_bytes : width;

    /* 计算大津法阈值 */
    threshold = BreezeOtsuThreshold(src, width, height, stride_bytes);

    /* 应用阈值 */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int idx = y * stride + x;
            dst[idx] = (src[idx] > threshold) ? max_value : 0;
        }
    }

    return threshold;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_OTSU_THRESHOLD_H */
