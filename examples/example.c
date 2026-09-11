/**
 * @file example.c
 * @brief Breeze框架的使用示例
 */

#include <stdio.h>
#include <stdlib.h>
#include "../include/breeze/breeze.h"

/* 互补滤波器示例函数 */
void complementary_filter_example(void) {
    BreezeComplementaryFilter filter;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;

    printf("互补滤波器示例\n");

    /* 初始化滤波器，alpha=0.98，dt=0.01秒 */
    BreezeComplementaryFilter_Init(&filter, 0.98f, 0.01f);

    /* 模拟传感器读数 */
    gyro_x = 0.1f;   /* 弧度/秒 */
    gyro_y = 0.05f;  /* 弧度/秒 */
    gyro_z = 0.02f;  /* 弧度/秒 */

    accel_x = 0.1f;  /* 重力加速度g */
    accel_y = 0.0f;  /* 重力加速度g */
    accel_z = 0.98f; /* 重力加速度g */

    /* 使用传感器数据更新滤波器 */
    BreezeComplementaryFilter_Update(&filter, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

    /* 获取估计的姿态角 */
    printf("横滚角: %.2f 度\n", BreezeComplementaryFilter_GetRoll(&filter) * 57.3f);
    printf("俯仰角: %.2f 度\n", BreezeComplementaryFilter_GetPitch(&filter) * 57.3f);
}

/* 位置式PID控制器示例函数 */
void position_pid_example(void) {
    BreezePIDController pid;
    float measurement, output;
    int i;

    printf("\n位置式PID控制器示例\n");
    printf("------------------------------------\n");

    /* 初始化位置式PID控制器 */
    BreezePIDController_Init(&pid, BREEZE_PID_POSITION, 1.0f, 0.1f, 0.05f, 0.1f, -100.0f, 100.0f);

    /* 设置目标值 */
    BreezePIDController_SetSetpoint(&pid, 50.0f);

    /* 模拟控制循环 */
    measurement = 0.0f;
    for (i = 0; i < 10; i++) {
        /* 计算控制输出 */
        output = BreezePIDController_Compute(&pid, measurement);

        /* 模拟过程（简化版） */
        measurement += output * 0.1f;

        printf("迭代 %d: 测量值=%.2f, 输出=%.2f\n", i+1, measurement, output);
    }
}

/* 增量式PID控制器示例函数 */
void incremental_pid_example(void) {
    BreezePIDController pid;
    float measurement, output;
    int i;

    printf("\n增量式PID控制器示例\n");
    printf("---------------------------------------\n");

    /* 初始化增量式PID控制器 */
    BreezePIDController_Init(&pid, BREEZE_PID_INCREMENTAL, 1.0f, 0.1f, 0.05f, 0.1f, -100.0f, 100.0f);

    /* 设置目标值 */
    BreezePIDController_SetSetpoint(&pid, 50.0f);

    /* 模拟控制循环 */
    measurement = 0.0f;
    for (i = 0; i < 10; i++) {
        /* 计算控制输出 */
        output = BreezePIDController_Compute(&pid, measurement);

        /* 模拟过程（简化版） */
        measurement += output * 0.1f;

        printf("迭代 %d: 测量值=%.2f, 输出=%.2f\n", i+1, measurement, output);
    }
}

/* 组合PID控制器示例 */
void pid_controller_example(void) {
    printf("\nPID控制器示例\n");
    printf("=====================\n");

    /* 运行两种PID控制器示例 */
    position_pid_example();
    incremental_pid_example();
}

/* 基础图像处理示例函数 */
void basic_image_processing_example(void) {
    unsigned char image[9] = {
        50, 100, 50,
        150, 200, 150,
        50, 100, 50
    };
    unsigned char result[9] = {0};
    unsigned char threshold;

    printf("\n基础图像处理示例\n");
    printf("=====================\n");

    /* 二值化阈值处理示例 */
    printf("二值化阈值处理:\n");
    BreezeBinaryThreshold(image, result, 3, 3, 100, 255, 0);
    printf("原始图像: %3d %3d %3d\n", image[0], image[1], image[2]);
    printf("         %3d %3d %3d\n", image[3], image[4], image[5]);
    printf("         %3d %3d %3d\n", image[6], image[7], image[8]);
    printf("处理结果: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);

    /* 大津法阈值处理示例 */
    printf("\n大津法阈值处理:\n");
    threshold = BreezeOtsuThreshold(image, 3, 3, 0);
    printf("计算得到的阈值: %d\n", threshold);

    /* Sobel算子示例 */
    printf("\nSobel边缘检测算子:\n");
    BreezeSobelOperator(image, result, 3, 3, 0);
    printf("边缘幅度:\n");
    printf("         %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);
}

/* 高斯模糊示例函数 */
void gaussian_blur_example(void) {
    unsigned char image[9] = {
        50, 100, 50,
        150, 200, 150,
        50, 100, 50
    };
    unsigned char result[9] = {0};

    printf("\n高斯模糊示例\n");
    printf("=====================\n");

    /* 应用高斯模糊 */
    BreezeGaussianBlur(image, result, 3, 3, 0.8f, 3, 0);

    printf("原始图像: %3d %3d %3d\n", image[0], image[1], image[2]);
    printf("         %3d %3d %3d\n", image[3], image[4], image[5]);
    printf("         %3d %3d %3d\n", image[6], image[7], image[8]);
    printf("高斯模糊: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);
}

/* Canny边缘检测示例函数 */
void canny_edge_example(void) {
    unsigned char image[9] = {
        50, 100, 50,
        150, 200, 150,
        50, 100, 50
    };
    unsigned char result[9] = {0};

    printf("\nCanny边缘检测示例\n");
    printf("=====================\n");

    /* 应用Canny边缘检测 */
    BreezeCannyEdgeDetection(image, result, 3, 3, 0.8f, 30.0f, 70.0f, 0);

    printf("原始图像: %3d %3d %3d\n", image[0], image[1], image[2]);
    printf("         %3d %3d %3d\n", image[3], image[4], image[5]);
    printf("         %3d %3d %3d\n", image[6], image[7], image[8]);
    printf("边缘检测: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);
}

/* 形态学操作示例函数 */
void morphology_example(void) {
    unsigned char image[9] = {
        0, 255, 0,
        255, 255, 255,
        0, 255, 0
    };
    unsigned char kernel[9] = {0};
    unsigned char result[9] = {0};

    printf("\n形态学操作示例\n");
    printf("=====================\n");

    /* 创建十字形结构元素 */
    BreezeMorphologyCreateKernel(kernel, 3, 1);

    printf("原始图像: %3d %3d %3d\n", image[0], image[1], image[2]);
    printf("         %3d %3d %3d\n", image[3], image[4], image[5]);
    printf("         %3d %3d %3d\n", image[6], image[7], image[8]);

    /* 膨胀操作 */
    BreezeMorphologyDilate(image, result, 3, 3, kernel, 3, 0);
    printf("\n膨胀结果: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);

    /* 腐蚀操作 */
    BreezeMorphologyErode(image, result, 3, 3, kernel, 3, 0);
    printf("\n腐蚀结果: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("         %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("         %3d %3d %3d\n", result[6], result[7], result[8]);
}

/* 直方图处理示例函数 */
void histogram_example(void) {
    unsigned char image[9] = {
        50, 100, 50,
        150, 200, 150,
        50, 100, 50
    };
    unsigned char result[9] = {0};
    int histogram[256] = {0};

    printf("\n直方图处理示例\n");
    printf("=====================\n");

    /* 计算直方图 */
    BreezeHistogramCompute(image, 3, 3, histogram, 0);

    printf("原始图像: %3d %3d %3d\n", image[0], image[1], image[2]);
    printf("         %3d %3d %3d\n", image[3], image[4], image[5]);
    printf("         %3d %3d %3d\n", image[6], image[7], image[8]);

    printf("\n直方图（非零值）:\n");
    for (int i = 0; i < 256; i++) {
        if (histogram[i] > 0) {
            printf("灰度值 %3d: %d 个像素\n", i, histogram[i]);
        }
    }

    /* 直方图均衡化 */
    BreezeHistogramEqualization(image, result, 3, 3, 0);
    printf("\n均衡化结果: %3d %3d %3d\n", result[0], result[1], result[2]);
    printf("           %3d %3d %3d\n", result[3], result[4], result[5]);
    printf("           %3d %3d %3d\n", result[6], result[7], result[8]);
}

/* 霍夫变换示例函数 */
void hough_transform_example(void) {
    /* 这个示例只展示霍夫变换的基本用法，实际应用需要更大的图像 */
    printf("\n霍夫变换示例\n");
    printf("=====================\n");
    printf("霍夫变换通常用于检测较大图像中的直线和圆。\n");
    printf("在小型示例图像上效果有限，此处仅展示API用法。\n");

    /* 霍夫直线变换示例 */
    printf("\n霍夫直线变换API示例:\n");
    printf("BreezeHoughLines(src, width, height, lines, max_lines, threshold, stride_bytes);\n");

    /* 霍夫圆变换示例 */
    printf("\n霍夫圆变换API示例:\n");
    printf("BreezeHoughCircles(src, width, height, circles, max_circles, min_radius, max_radius, threshold, stride_bytes);\n");
}

/* 图像处理示例函数 */
void image_processing_example(void) {
    printf("\n图像处理模块示例\n");
    printf("========================\n");

    basic_image_processing_example();
    gaussian_blur_example();
    canny_edge_example();
    morphology_example();
    histogram_example();
    hough_transform_example();
}

int main(void) {
    printf("Breeze框架示例\n");
    printf("========================\n\n");

    complementary_filter_example();
    pid_controller_example();
    image_processing_example();

    return 0;
}
