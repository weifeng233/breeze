/**
 * @file math_example.c
 * @brief Breeze框架数学工具模块的使用示例
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/breeze/breeze.h"

/* 矩阵运算示例函数 */
void matrix_example(void) {
    BreezeMatrix a, b, c;
    int i, j;
    
    printf("\n矩阵运算示例\n");
    printf("------------------------------------\n");
    
    /* 初始化矩阵 */
    BreezeMatrix_Init(&a, 2, 2);
    BreezeMatrix_Init(&b, 2, 2);
    BreezeMatrix_Init(&c, 2, 2);
    
    /* 设置矩阵A的元素 */
    BreezeMatrix_SetElement(&a, 0, 0, 1.0f);
    BreezeMatrix_SetElement(&a, 0, 1, 2.0f);
    BreezeMatrix_SetElement(&a, 1, 0, 3.0f);
    BreezeMatrix_SetElement(&a, 1, 1, 4.0f);
    
    /* 设置矩阵B的元素 */
    BreezeMatrix_SetElement(&b, 0, 0, 5.0f);
    BreezeMatrix_SetElement(&b, 0, 1, 6.0f);
    BreezeMatrix_SetElement(&b, 1, 0, 7.0f);
    BreezeMatrix_SetElement(&b, 1, 1, 8.0f);
    
    /* 打印矩阵A */
    printf("矩阵A:\n");
    for (i = 0; i < a.rows; i++) {
        for (j = 0; j < a.cols; j++) {
            printf("%.1f ", BreezeMatrix_GetElement(&a, i, j));
        }
        printf("\n");
    }
    
    /* 打印矩阵B */
    printf("\n矩阵B:\n");
    for (i = 0; i < b.rows; i++) {
        for (j = 0; j < b.cols; j++) {
            printf("%.1f ", BreezeMatrix_GetElement(&b, i, j));
        }
        printf("\n");
    }
    
    /* 矩阵加法 */
    BreezeMatrix_Add(&c, &a, &b);
    
    printf("\n矩阵A + 矩阵B:\n");
    for (i = 0; i < c.rows; i++) {
        for (j = 0; j < c.cols; j++) {
            printf("%.1f ", BreezeMatrix_GetElement(&c, i, j));
        }
        printf("\n");
    }
    
    /* 矩阵乘法 */
    BreezeMatrix_Multiply(&c, &a, &b);
    
    printf("\n矩阵A * 矩阵B:\n");
    for (i = 0; i < c.rows; i++) {
        for (j = 0; j < c.cols; j++) {
            printf("%.1f ", BreezeMatrix_GetElement(&c, i, j));
        }
        printf("\n");
    }
    
    /* 矩阵转置 */
    BreezeMatrix_Transpose(&c, &a);
    
    printf("\n矩阵A的转置:\n");
    for (i = 0; i < c.rows; i++) {
        for (j = 0; j < c.cols; j++) {
            printf("%.1f ", BreezeMatrix_GetElement(&c, i, j));
        }
        printf("\n");
    }
    
    /* 计算行列式 */
    float det = BreezeMatrix_Determinant(&a);
    printf("\n矩阵A的行列式: %.1f\n", det);
    
    /* 矩阵求逆 */
    if (BreezeMatrix_Inverse(&c, &a)) {
        printf("\n矩阵A的逆:\n");
        for (i = 0; i < c.rows; i++) {
            for (j = 0; j < c.cols; j++) {
                printf("%.3f ", BreezeMatrix_GetElement(&c, i, j));
            }
            printf("\n");
        }
    } else {
        printf("\n矩阵A不可逆\n");
    }
}

/* 向量运算示例函数 */
void vector_example(void) {
    BreezeVector3D a, b, c;
    float dot_product;
    
    printf("\n向量运算示例\n");
    printf("------------------------------------\n");
    
    /* 初始化向量 */
    BreezeVector3D_Init(&a, 1.0f, 2.0f, 3.0f);
    BreezeVector3D_Init(&b, 4.0f, 5.0f, 6.0f);
    
    /* 打印向量 */
    printf("向量A: (%.1f, %.1f, %.1f)\n", a.x, a.y, a.z);
    printf("向量B: (%.1f, %.1f, %.1f)\n", b.x, b.y, b.z);
    
    /* 向量加法 */
    BreezeVector3D_Add(&c, &a, &b);
    printf("\n向量A + 向量B: (%.1f, %.1f, %.1f)\n", c.x, c.y, c.z);
    
    /* 向量减法 */
    BreezeVector3D_Subtract(&c, &a, &b);
    printf("向量A - 向量B: (%.1f, %.1f, %.1f)\n", c.x, c.y, c.z);
    
    /* 向量标量乘法 */
    BreezeVector3D_ScalarMultiply(&c, &a, 2.0f);
    printf("向量A * 2: (%.1f, %.1f, %.1f)\n", c.x, c.y, c.z);
    
    /* 向量点积 */
    dot_product = BreezeVector3D_DotProduct(&a, &b);
    printf("向量A · 向量B: %.1f\n", dot_product);
    
    /* 向量叉积 */
    BreezeVector3D_CrossProduct(&c, &a, &b);
    printf("向量A × 向量B: (%.1f, %.1f, %.1f)\n", c.x, c.y, c.z);
    
    /* 向量长度 */
    printf("向量A的长度: %.3f\n", BreezeVector3D_Length(&a));
    
    /* 向量归一化 */
    BreezeVector3D_Normalize(&c, &a);
    printf("向量A归一化: (%.3f, %.3f, %.3f)\n", c.x, c.y, c.z);
    printf("归一化后的长度: %.3f\n", BreezeVector3D_Length(&c));
}

/* 四元数运算示例函数 */
void quaternion_example(void) {
    BreezeQuaternion q1, q2, q3;
    BreezeVector3D axis, v, rotated_v;
    float roll, pitch, yaw;
    
    printf("\n四元数运算示例\n");
    printf("------------------------------------\n");
    
    /* 初始化四元数 */
    BreezeQuaternion_SetIdentity(&q1);
    printf("单位四元数: (%.1f, %.1f, %.1f, %.1f)\n", q1.w, q1.x, q1.y, q1.z);
    
    /* 从欧拉角创建四元数 */
    roll = 0.1f;  /* 横滚角（弧度） */
    pitch = 0.2f; /* 俯仰角（弧度） */
    yaw = 0.3f;   /* 航向角（弧度） */
    
    BreezeQuaternion_FromEulerZYX(&q2, roll, pitch, yaw);
    printf("\n从欧拉角(%.1f, %.1f, %.1f)创建的四元数: (%.3f, %.3f, %.3f, %.3f)\n",
           roll, pitch, yaw, q2.w, q2.x, q2.y, q2.z);
    
    /* 从轴角表示创建四元数 */
    BreezeVector3D_Init(&axis, 0.0f, 0.0f, 1.0f);  /* Z轴 */
    BreezeQuaternion_FromAxisAngle(&q3, &axis, M_PI / 4.0f);  /* 绕Z轴旋转45度 */
    
    printf("\n绕Z轴旋转45度的四元数: (%.3f, %.3f, %.3f, %.3f)\n",
           q3.w, q3.x, q3.y, q3.z);
    
    /* 四元数乘法 */
    BreezeQuaternion_Multiply(&q1, &q2, &q3);
    printf("\n四元数乘法结果: (%.3f, %.3f, %.3f, %.3f)\n",
           q1.w, q1.x, q1.y, q1.z);
    
    /* 四元数旋转向量 */
    BreezeVector3D_Init(&v, 1.0f, 0.0f, 0.0f);  /* X轴单位向量 */
    BreezeQuaternion_RotateVector(&rotated_v, &q3, &v);
    
    printf("\n旋转前的向量: (%.1f, %.1f, %.1f)\n", v.x, v.y, v.z);
    printf("旋转后的向量: (%.3f, %.3f, %.3f)\n", rotated_v.x, rotated_v.y, rotated_v.z);
    
    /* 四元数转欧拉角 */
    BreezeQuaternion_ToEulerZYX(&q2, &roll, &pitch, &yaw);
    printf("\n四元数转换回欧拉角: (%.3f, %.3f, %.3f)\n", roll, pitch, yaw);
}

/* 插值算法示例函数 */
void interpolation_example(void) {
    float x0, y0, x1, y1, x;
    float y_linear, y_cosine;
    BreezeBezierPoint p0, p1, p2, p3, result;
    int i;
    
    printf("\n插值算法示例\n");
    printf("------------------------------------\n");
    
    /* 线性插值 */
    x0 = 0.0f;
    y0 = 0.0f;
    x1 = 10.0f;
    y1 = 5.0f;
    
    printf("线性插值示例:\n");
    printf("起点: (%.1f, %.1f), 终点: (%.1f, %.1f)\n", x0, y0, x1, y1);
    printf("x\ty_linear\ty_cosine\n");
    
    for (i = 0; i <= 10; i += 2) {
        x = (float)i;
        y_linear = BreezeInterpolation_Linear(x0, y0, x1, y1, x);
        y_cosine = BreezeInterpolation_Cosine(y0, y1, x / 10.0f);
        
        printf("%.1f\t%.2f\t%.2f\n", x, y_linear, y_cosine);
    }
    
    /* 贝塞尔曲线插值 */
    printf("\n三次贝塞尔曲线示例:\n");
    
    p0.x = 0.0f;
    p0.y = 0.0f;
    
    p1.x = 1.0f;
    p1.y = 3.0f;
    
    p2.x = 3.0f;
    p2.y = 3.0f;
    
    p3.x = 4.0f;
    p3.y = 0.0f;
    
    printf("控制点: (%.1f, %.1f), (%.1f, %.1f), (%.1f, %.1f), (%.1f, %.1f)\n",
           p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
    printf("t\tx\ty\n");
    
    for (i = 0; i <= 10; i += 2) {
        float t = i / 10.0f;
        BreezeBezierCurve_Cubic(&p0, &p1, &p2, &p3, t, &result);
        
        printf("%.1f\t%.2f\t%.2f\n", t, result.x, result.y);
    }
}

/* 数学工具模块示例 */
void math_examples(void) {
    printf("\n数学工具模块示例\n");
    printf("==============================\n");
    
    matrix_example();
    vector_example();
    quaternion_example();
    interpolation_example();
}

int main(void) {
    printf("Breeze框架数学工具模块示例\n");
    printf("=======================================\n");
    
    math_examples();
    
    return 0;
}
