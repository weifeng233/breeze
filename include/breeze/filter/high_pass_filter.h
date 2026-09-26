/**
 * @file high_pass_filter.h
 * @brief 高通滤波器实现
 *
 * 该实现提供了一阶高通滤波器，用于提取信号的快速变化部分，
 * 同时抑制信号的低频成分或直流分量。
 */

#ifndef BREEZE_HIGH_PASS_FILTER_H
#define BREEZE_HIGH_PASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶高通滤波器状态结构体
 */
typedef struct {
    float alpha;        /* 滤波系数（0.0到1.0） */
    float prev_input;   /* 上一次的输入值 */
    float prev_output;  /* 上一次的输出值 */
    int initialized;    /* 是否已初始化 */
} BreezeHighPassFilter;

/**
 * @brief 初始化一阶高通滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）- 值越大滤波效果越强，但响应越慢
 */
static inline void BreezeHighPassFilter_Init(BreezeHighPassFilter* filter, float alpha) {
    if (filter) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->prev_input = 0.0f;
        filter->prev_output = 0.0f;
        filter->initialized = 0;
    }
}

/**
 * @brief 使用时间常数初始化一阶高通滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param time_constant 时间常数（秒）
 * @param sample_time 采样时间（秒）
 */
static inline void BreezeHighPassFilter_InitWithTimeConstant(
    BreezeHighPassFilter* filter,
    float time_constant,
    float sample_time
) {
    float alpha;

    if (filter && time_constant > 0.0f && sample_time > 0.0f) {
        /* 根据时间常数和采样时间计算alpha */
        alpha = time_constant / (time_constant + sample_time);
        
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->prev_input = 0.0f;
        filter->prev_output = 0.0f;
        filter->initialized = 0;
    }
}

/**
 * @brief 设置滤波系数
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）
 */
static inline void BreezeHighPassFilter_SetAlpha(BreezeHighPassFilter* filter, float alpha) {
    if (filter) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
    }
}

/**
 * @brief 使用时间常数设置滤波系数
 *
 * @param filter 指向滤波器结构体的指针
 * @param time_constant 时间常数（秒）
 * @param sample_time 采样时间（秒）
 */
static inline void BreezeHighPassFilter_SetTimeConstant(
    BreezeHighPassFilter* filter,
    float time_constant,
    float sample_time
) {
    float alpha;

    if (filter && time_constant > 0.0f && sample_time > 0.0f) {
        /* 根据时间常数和采样时间计算alpha */
        alpha = time_constant / (time_constant + sample_time);
        
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
    }
}

/**
 * @brief 更新高通滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param input 新的输入值
 * @return 滤波后的输出值
 */
static inline float BreezeHighPassFilter_Update(BreezeHighPassFilter* filter, float input) {
    float output;

    if (!filter) return 0.0f;

    if (!filter->initialized) {
        filter->prev_input = input;
        filter->prev_output = 0.0f;
        filter->initialized = 1;
        return 0.0f;
    }

    /* 一阶高通滤波器公式: output = alpha * (prev_output + input - prev_input) */
    output = filter->alpha * (filter->prev_output + input - filter->prev_input);
    
    /* 更新状态 */
    filter->prev_input = input;
    filter->prev_output = output;

    return output;
}

/**
 * @brief 重置高通滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param input_value 输入值重置
 */
static inline void BreezeHighPassFilter_Reset(BreezeHighPassFilter* filter, float input_value) {
    if (filter) {
        filter->prev_input = input_value;
        filter->prev_output = 0.0f;
        filter->initialized = 1;
    }
}

/**
 * @brief 直流阻断滤波器（特殊的高通滤波器）状态结构体
 */
typedef struct {
    float alpha;        /* 滤波系数（0.0到1.0） */
    float avg;          /* 信号的移动平均值 */
    int initialized;    /* 是否已初始化 */
} BreezeDCBlocker;

/**
 * @brief 初始化直流阻断滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）- 值越小滤波效果越强，但响应越慢
 */
static inline void BreezeDCBlocker_Init(BreezeDCBlocker* filter, float alpha) {
    if (filter) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->avg = 0.0f;
        filter->initialized = 0;
    }
}

/**
 * @brief 更新直流阻断滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param input 新的输入值
 * @return 去除直流分量后的输出值
 */
static inline float BreezeDCBlocker_Update(BreezeDCBlocker* filter, float input) {
    if (!filter) return input;

    if (!filter->initialized) {
        filter->avg = input;
        filter->initialized = 1;
        return 0.0f;
    }

    /* 更新移动平均值 */
    filter->avg = filter->alpha * input + (1.0f - filter->alpha) * filter->avg;
    
    /* 输出为输入减去移动平均值 */
    return input - filter->avg;
}

/**
 * @brief 重置直流阻断滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param value 重置值
 */
static inline void BreezeDCBlocker_Reset(BreezeDCBlocker* filter, float value) {
    if (filter) {
        filter->avg = value;
        filter->initialized = 1;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_HIGH_PASS_FILTER_H */
