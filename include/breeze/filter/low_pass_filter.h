/**
 * @file low_pass_filter.h
 * @brief 低通滤波器实现
 *
 * 该实现提供了几种常用的低通滤波器，用于抑制高频噪声。
 * 包括一阶低通滤波器和指数加权移动平均滤波器。
 */

#ifndef BREEZE_LOW_PASS_FILTER_H
#define BREEZE_LOW_PASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶低通滤波器状态结构体
 */
typedef struct {
    float alpha;        /* 滤波系数（0.0到1.0） */
    float prev_output;  /* 上一次的输出值 */
    int initialized;    /* 是否已初始化 */
} BreezeLowPassFilter;

/**
 * @brief 初始化一阶低通滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）- 值越小滤波效果越强，但响应越慢
 * @param initial_value 初始输出值
 */
static void BreezeLowPassFilter_Init(
    BreezeLowPassFilter* filter,
    float alpha,
    float initial_value
) {
    if (filter) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->prev_output = initial_value;
        filter->initialized = 1;
    }
}

/**
 * @brief 使用时间常数初始化一阶低通滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param time_constant 时间常数（秒）
 * @param sample_time 采样时间（秒）
 * @param initial_value 初始输出值
 */
static void BreezeLowPassFilter_InitWithTimeConstant(
    BreezeLowPassFilter* filter,
    float time_constant,
    float sample_time,
    float initial_value
) {
    float alpha;

    if (filter && time_constant > 0.0f && sample_time > 0.0f) {
        /* 根据时间常数和采样时间计算alpha */
        alpha = sample_time / (time_constant + sample_time);
        
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->prev_output = initial_value;
        filter->initialized = 1;
    }
}

/**
 * @brief 设置滤波系数
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）
 */
static void BreezeLowPassFilter_SetAlpha(BreezeLowPassFilter* filter, float alpha) {
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
static void BreezeLowPassFilter_SetTimeConstant(
    BreezeLowPassFilter* filter,
    float time_constant,
    float sample_time
) {
    float alpha;

    if (filter && time_constant > 0.0f && sample_time > 0.0f) {
        /* 根据时间常数和采样时间计算alpha */
        alpha = sample_time / (time_constant + sample_time);
        
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
    }
}

/**
 * @brief 更新滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param input 新的输入值
 * @return 滤波后的输出值
 */
static float BreezeLowPassFilter_Update(BreezeLowPassFilter* filter, float input) {
    float output;

    if (!filter) return input;

    if (!filter->initialized) {
        filter->prev_output = input;
        filter->initialized = 1;
        return input;
    }

    /* 一阶低通滤波器公式: output = alpha * input + (1 - alpha) * prev_output */
    output = filter->alpha * input + (1.0f - filter->alpha) * filter->prev_output;
    filter->prev_output = output;

    return output;
}

/**
 * @brief 重置滤波器状态
 *
 * @param filter 指向滤波器结构体的指针
 * @param value 重置值
 */
static void BreezeLowPassFilter_Reset(BreezeLowPassFilter* filter, float value) {
    if (filter) {
        filter->prev_output = value;
        filter->initialized = 1;
    }
}

/**
 * @brief 指数加权移动平均滤波器状态结构体
 */
typedef struct {
    float alpha;        /* 平滑系数（0.0到1.0） */
    float avg;          /* 当前平均值 */
    int initialized;    /* 是否已初始化 */
} BreezeEWMAFilter;

/**
 * @brief 初始化指数加权移动平均滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param alpha 平滑系数（0.0到1.0）- 值越小平滑效果越强
 * @param initial_value 初始平均值
 */
static void BreezeEWMAFilter_Init(
    BreezeEWMAFilter* filter,
    float alpha,
    float initial_value
) {
    if (filter) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        filter->alpha = alpha;
        filter->avg = initial_value;
        filter->initialized = 1;
    }
}

/**
 * @brief 更新指数加权移动平均滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param input 新的输入值
 * @return 滤波后的平均值
 */
static float BreezeEWMAFilter_Update(BreezeEWMAFilter* filter, float input) {
    if (!filter) return input;

    if (!filter->initialized) {
        filter->avg = input;
        filter->initialized = 1;
        return input;
    }

    /* 指数加权移动平均公式: avg = alpha * input + (1 - alpha) * avg */
    filter->avg = filter->alpha * input + (1.0f - filter->alpha) * filter->avg;

    return filter->avg;
}

/**
 * @brief 重置指数加权移动平均滤波器
 *
 * @param filter 指向滤波器结构体的指针
 * @param value 重置值
 */
static void BreezeEWMAFilter_Reset(BreezeEWMAFilter* filter, float value) {
    if (filter) {
        filter->avg = value;
        filter->initialized = 1;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_LOW_PASS_FILTER_H */
