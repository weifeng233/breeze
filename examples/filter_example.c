/**
 * @file filter_example.c
 * @brief Breeze框架滤波器模块的使用示例
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/breeze/breeze.h"

/* 卡尔曼滤波器示例函数 */
void kalman_filter_example(void) {
    BreezeKalmanFilter1D filter;
    float measurement, filtered_value;
    int i;

    printf("\n卡尔曼滤波器示例\n");
    printf("------------------------------------\n");

    /* 初始化卡尔曼滤波器 */
    /* 参数: q=过程噪声, r=测量噪声, p_init=初始协方差, x_init=初始状态 */
    BreezeKalmanFilter1D_Init(&filter, 0.01f, 0.1f, 1.0f, 0.0f);

    /* 模拟带噪声的测量数据 */
    printf("原始值\t测量值\t滤波值\n");
    for (i = 0; i < 10; i++) {
        float true_value = (float)i;
        /* 添加随机噪声 */
        measurement = true_value + ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
        
        /* 使用卡尔曼滤波器处理 */
        filtered_value = BreezeKalmanFilter1D_Update(&filter, measurement);
        
        printf("%.2f\t%.2f\t%.2f\n", true_value, measurement, filtered_value);
    }
}

/* 低通滤波器示例函数 */
void low_pass_filter_example(void) {
    BreezeLowPassFilter filter;
    float input, filtered_value;
    int i;

    printf("\n低通滤波器示例\n");
    printf("------------------------------------\n");

    /* 初始化低通滤波器，alpha=0.2（较强的滤波效果） */
    BreezeLowPassFilter_Init(&filter, 0.2f, 0.0f);

    /* 模拟带高频噪声的信号 */
    printf("原始值\t带噪声值\t滤波值\n");
    for (i = 0; i < 10; i++) {
        float true_value = sinf((float)i * 0.1f) * 5.0f;
        /* 添加高频噪声 */
        input = true_value + sinf((float)i * 2.0f) * 1.0f;
        
        /* 使用低通滤波器处理 */
        filtered_value = BreezeLowPassFilter_Update(&filter, input);
        
        printf("%.2f\t%.2f\t\t%.2f\n", true_value, input, filtered_value);
    }

    /* 使用时间常数初始化的示例 */
    printf("\n使用时间常数初始化的低通滤波器:\n");
    BreezeLowPassFilter_InitWithTimeConstant(&filter, 0.5f, 0.1f, 0.0f);
    
    for (i = 0; i < 5; i++) {
        float true_value = (float)i;
        input = true_value + ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
        filtered_value = BreezeLowPassFilter_Update(&filter, input);
        printf("输入: %.2f, 滤波后: %.2f\n", input, filtered_value);
    }
}

/* 高通滤波器示例函数 */
void high_pass_filter_example(void) {
    BreezeHighPassFilter filter;
    float input, filtered_value;
    int i;

    printf("\n高通滤波器示例\n");
    printf("------------------------------------\n");

    /* 初始化高通滤波器，alpha=0.8（中等滤波效果） */
    BreezeHighPassFilter_Init(&filter, 0.8f);

    /* 模拟带有直流偏移的信号 */
    printf("原始值\t带偏移值\t滤波值\n");
    for (i = 0; i < 10; i++) {
        float true_value = sinf((float)i * 0.5f) * 2.0f;
        /* 添加直流偏移 */
        input = true_value + 5.0f;
        
        /* 使用高通滤波器处理 */
        filtered_value = BreezeHighPassFilter_Update(&filter, input);
        
        printf("%.2f\t%.2f\t\t%.2f\n", true_value, input, filtered_value);
    }

    /* 直流阻断滤波器示例 */
    printf("\n直流阻断滤波器示例:\n");
    BreezeDCBlocker dc_blocker;
    BreezeDCBlocker_Init(&dc_blocker, 0.05f);
    
    for (i = 0; i < 10; i++) {
        float true_value = sinf((float)i * 0.5f) * 2.0f;
        input = true_value + 5.0f;
        filtered_value = BreezeDCBlocker_Update(&dc_blocker, input);
        printf("输入: %.2f, 滤波后: %.2f\n", input, filtered_value);
    }
}

/* 中值滤波器示例函数 */
void median_filter_example(void) {
    BreezeMedianFilter filter;
    float buffer[5], sorted[5];
    float input, filtered_value;
    int i;

    printf("\n中值滤波器示例\n");
    printf("------------------------------------\n");

    /* 初始化中值滤波器，窗口大小为5 */
    BreezeMedianFilter_Init(&filter, buffer, sorted, 5);

    /* 模拟带有脉冲噪声的信号 */
    printf("原始值\t带噪声值\t滤波值\n");
    for (i = 0; i < 15; i++) {
        float true_value = (float)i * 0.5f;
        
        /* 添加脉冲噪声（每3个样本添加一个脉冲） */
        if (i % 3 == 0) {
            input = true_value + 10.0f;
        } else {
            input = true_value + ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
        }
        
        /* 使用中值滤波器处理 */
        filtered_value = BreezeMedianFilter_Update(&filter, input);
        
        printf("%.2f\t%.2f\t\t%.2f\n", true_value, input, filtered_value);
    }

    /* 图像中值滤波示例 */
    printf("\n图像中值滤波示例:\n");
    unsigned char image[9] = {
        50, 100, 50,
        150, 255, 150,  /* 255是脉冲噪声 */
        50, 100, 50
    };
    unsigned char result[9] = {0};
    
    printf("原始图像:\n");
    printf("%3d %3d %3d\n", image[0], image[1], image[2]);
    printf("%3d %3d %3d\n", image[3], image[4], image[5]);
    printf("%3d %3d %3d\n", image[6], image[7], image[8]);
    
    BreezeMedianFilterImage(image, result, 3, 3, 3, 0);
    
    printf("中值滤波后:\n");
    printf("%3d %3d %3d\n", result[0], result[1], result[2]);
    printf("%3d %3d %3d\n", result[3], result[4], result[5]);
    printf("%3d %3d %3d\n", result[6], result[7], result[8]);
}

/* 组合滤波器示例 */
void combined_filter_example(void) {
    BreezeLowPassFilter lpf;
    BreezeHighPassFilter hpf;
    float input, lpf_output, band_pass_output;
    int i;

    printf("\n组合滤波器示例（带通滤波器）\n");
    printf("------------------------------------\n");

    /* 初始化低通和高通滤波器 */
    BreezeLowPassFilter_Init(&lpf, 0.5f, 0.0f);
    BreezeHighPassFilter_Init(&hpf, 0.8f);

    /* 模拟包含多种频率的信号 */
    printf("原始值\t低通滤波\t带通滤波\n");
    for (i = 0; i < 10; i++) {
        /* 生成包含低频、中频和高频成分的信号 */
        input = sinf((float)i * 0.1f) * 3.0f +  /* 低频 */
                sinf((float)i * 0.5f) * 2.0f +  /* 中频 */
                sinf((float)i * 2.0f) * 1.0f;   /* 高频 */
        
        /* 先应用低通滤波器 */
        lpf_output = BreezeLowPassFilter_Update(&lpf, input);
        
        /* 然后对低通滤波结果应用高通滤波器，形成带通滤波器 */
        band_pass_output = BreezeHighPassFilter_Update(&hpf, lpf_output);
        
        printf("%.2f\t%.2f\t\t%.2f\n", input, lpf_output, band_pass_output);
    }
}

/* 滤波器模块示例 */
void filter_examples(void) {
    printf("\n滤波器模块示例\n");
    printf("==============================\n");

    kalman_filter_example();
    low_pass_filter_example();
    high_pass_filter_example();
    median_filter_example();
    combined_filter_example();
}

int main(void) {
    printf("Breeze框架滤波器示例\n");
    printf("=======================================\n");

    filter_examples();

    return 0;
}
