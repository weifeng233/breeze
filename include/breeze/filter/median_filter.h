/**
 * @file median_filter.h
 * @brief 中值滤波器实现
 *
 * 该实现提供了中值滤波器，用于去除信号中的脉冲噪声（椒盐噪声）。
 * 中值滤波器对于去除异常值特别有效，同时保持信号的边缘特性。
 */

#ifndef BREEZE_MEDIAN_FILTER_H
#define BREEZE_MEDIAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 中值滤波器状态结构体
 */
typedef struct {
    float* buffer;      /* 数据缓冲区 */
    float* sorted;      /* 排序后的数据缓冲区 */
    int size;           /* 窗口大小 */
    int index;          /* 当前索引 */
    int count;          /* 当前缓冲区中的数据数量 */
} BreezeMedianFilter;

/**
 * @brief 初始化中值滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param buffer 数据缓冲区
 * @param sorted 排序缓冲区（与buffer大小相同）
 * @param size 窗口大小
 */
static void BreezeMedianFilter_Init(
    BreezeMedianFilter* filter,
    float* buffer,
    float* sorted,
    int size
) {
    int i;

    if (filter && buffer && sorted && size > 0) {
        filter->buffer = buffer;
        filter->sorted = sorted;
        filter->size = size;
        filter->index = 0;
        filter->count = 0;

        /* 初始化缓冲区 */
        for (i = 0; i < size; i++) {
            buffer[i] = 0.0f;
            sorted[i] = 0.0f;
        }
    }
}

/**
 * @brief 插入排序算法
 *
 * @param arr 要排序的数组
 * @param n 数组大小
 */
static void BreezeMedianFilter_InsertionSort(float* arr, int n) {
    int i, j;
    float key;

    for (i = 1; i < n; i++) {
        key = arr[i];
        j = i - 1;

        /* 将比key大的元素向右移动 */
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j = j - 1;
        }
        arr[j + 1] = key;
    }
}

/**
 * @brief 更新中值滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param input 新的输入值
 * @return 中值滤波后的输出值
 */
static float BreezeMedianFilter_Update(BreezeMedianFilter* filter, float input) {
    int i;
    float median;

    if (!filter || !filter->buffer || !filter->sorted || filter->size <= 0) {
        return input;
    }

    /* 将新值添加到缓冲区 */
    filter->buffer[filter->index] = input;
    filter->index = (filter->index + 1) % filter->size;

    /* 更新计数 */
    if (filter->count < filter->size) {
        filter->count++;
    }

    /* 复制缓冲区到排序数组 */
    for (i = 0; i < filter->count; i++) {
        filter->sorted[i] = filter->buffer[i];
    }

    /* 对排序数组进行排序 */
    BreezeMedianFilter_InsertionSort(filter->sorted, filter->count);

    /* 计算中值 */
    if (filter->count % 2 == 0) {
        /* 偶数个元素，取中间两个的平均值 */
        median = (filter->sorted[filter->count / 2 - 1] + filter->sorted[filter->count / 2]) / 2.0f;
    } else {
        /* 奇数个元素，取中间的元素 */
        median = filter->sorted[filter->count / 2];
    }

    return median;
}

/**
 * @brief 重置中值滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 */
static void BreezeMedianFilter_Reset(BreezeMedianFilter* filter) {
    int i;

    if (filter && filter->buffer && filter->size > 0) {
        filter->index = 0;
        filter->count = 0;

        /* 清空缓冲区 */
        for (i = 0; i < filter->size; i++) {
            filter->buffer[i] = 0.0f;
            filter->sorted[i] = 0.0f;
        }
    }
}

/**
 * @brief 对图像应用中值滤波
 *
 * @param src 源图像数据
 * @param dst 目标图像数据
 * @param width 图像宽度
 * @param height 图像高度
 * @param kernel_size 滤波核大小（必须是奇数，如3、5、7等）
 * @param stride_bytes 每行的字节数（如果为0，则使用宽度）
 */
static void BreezeMedianFilterImage(
    const unsigned char* src,
    unsigned char* dst,
    int width, int height,
    int kernel_size,
    int stride_bytes
) {
    int x, y, i, j, k;
    int stride;
    int half_kernel;
    unsigned char* window;
    
    if (!src || !dst || width <= 0 || height <= 0 || kernel_size <= 0) return;
    
    /* 确保kernel_size是奇数 */
    if (kernel_size % 2 == 0) kernel_size++;
    
    /* 分配临时窗口缓冲区 */
    window = (unsigned char*)malloc(kernel_size * kernel_size);
    if (!window) return;
    
    stride = stride_bytes > 0 ? stride_bytes : width;
    half_kernel = kernel_size / 2;
    
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            /* 收集窗口内的像素 */
            k = 0;
            for (j = -half_kernel; j <= half_kernel; j++) {
                for (i = -half_kernel; i <= half_kernel; i++) {
                    int nx = x + i;
                    int ny = y + j;
                    
                    /* 边界处理 */
                    if (nx < 0) nx = 0;
                    if (nx >= width) nx = width - 1;
                    if (ny < 0) ny = 0;
                    if (ny >= height) ny = height - 1;
                    
                    window[k++] = src[ny * stride + nx];
                }
            }
            
            /* 对窗口内的像素进行排序 */
            for (i = 0; i < k - 1; i++) {
                for (j = 0; j < k - i - 1; j++) {
                    if (window[j] > window[j + 1]) {
                        unsigned char temp = window[j];
                        window[j] = window[j + 1];
                        window[j + 1] = temp;
                    }
                }
            }
            
            /* 取中值作为输出 */
            dst[y * stride + x] = window[k / 2];
        }
    }
    
    free(window);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_MEDIAN_FILTER_H */
