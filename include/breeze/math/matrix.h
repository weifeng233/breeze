/**
 * @file matrix.h
 * @brief 矩阵运算实现
 *
 * 该实现提供了基本的矩阵运算功能，支持最多4x4的矩阵。
 * 包括矩阵加减乘、转置、求逆等操作。
 */

#ifndef BREEZE_MATRIX_H
#define BREEZE_MATRIX_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 矩阵结构体（最多支持4x4矩阵）
 */
typedef struct {
    float data[4][4];         /* 矩阵数据 */
    int rows;                 /* 行数 */
    int cols;                 /* 列数 */
} BreezeMatrix;

/**
 * @brief 初始化矩阵
 *
 * @param matrix 指向矩阵结构体的指针
 * @param rows 行数（最多4）
 * @param cols 列数（最多4）
 */
static inline void BreezeMatrix_Init(BreezeMatrix* matrix, int rows, int cols) {
    int i, j;
    
    if (!matrix || rows <= 0 || cols <= 0 || rows > 4 || cols > 4) return;
    
    matrix->rows = rows;
    matrix->cols = cols;
    
    /* 初始化为零矩阵 */
    for (i = 0; i < rows; i++) {
        for (j = 0; j < cols; j++) {
            matrix->data[i][j] = 0.0f;
        }
    }
}

/**
 * @brief 设置矩阵为单位矩阵
 *
 * @param matrix 指向矩阵结构体的指针
 */
static inline void BreezeMatrix_SetIdentity(BreezeMatrix* matrix) {
    int i, j;
    int min_dim;
    
    if (!matrix) return;
    
    min_dim = matrix->rows < matrix->cols ? matrix->rows : matrix->cols;
    
    for (i = 0; i < matrix->rows; i++) {
        for (j = 0; j < matrix->cols; j++) {
            matrix->data[i][j] = (i == j && i < min_dim) ? 1.0f : 0.0f;
        }
    }
}

/**
 * @brief 设置矩阵元素值
 *
 * @param matrix 指向矩阵结构体的指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @param value 元素值
 */
static inline void BreezeMatrix_SetElement(BreezeMatrix* matrix, int row, int col, float value) {
    if (!matrix || row < 0 || col < 0 || row >= matrix->rows || col >= matrix->cols) return;
    
    matrix->data[row][col] = value;
}

/**
 * @brief 获取矩阵元素值
 *
 * @param matrix 指向矩阵结构体的指针
 * @param row 行索引（从0开始）
 * @param col 列索引（从0开始）
 * @return 元素值，如果索引无效则返回0
 */
static inline float BreezeMatrix_GetElement(const BreezeMatrix* matrix, int row, int col) {
    if (!matrix || row < 0 || col < 0 || row >= matrix->rows || col >= matrix->cols) return 0.0f;
    
    return matrix->data[row][col];
}

/**
 * @brief 矩阵加法
 *
 * @param result 结果矩阵
 * @param a 输入矩阵A
 * @param b 输入矩阵B
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Add(BreezeMatrix* result, const BreezeMatrix* a, const BreezeMatrix* b) {
    int i, j;
    
    if (!result || !a || !b) return 0;
    if (a->rows != b->rows || a->cols != b->cols) return 0;
    
    result->rows = a->rows;
    result->cols = a->cols;
    
    for (i = 0; i < a->rows; i++) {
        for (j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] + b->data[i][j];
        }
    }
    
    return 1;
}

/**
 * @brief 矩阵减法
 *
 * @param result 结果矩阵
 * @param a 输入矩阵A
 * @param b 输入矩阵B
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Subtract(BreezeMatrix* result, const BreezeMatrix* a, const BreezeMatrix* b) {
    int i, j;
    
    if (!result || !a || !b) return 0;
    if (a->rows != b->rows || a->cols != b->cols) return 0;
    
    result->rows = a->rows;
    result->cols = a->cols;
    
    for (i = 0; i < a->rows; i++) {
        for (j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] - b->data[i][j];
        }
    }
    
    return 1;
}

/**
 * @brief 矩阵乘法
 *
 * @param result 结果矩阵
 * @param a 输入矩阵A
 * @param b 输入矩阵B
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Multiply(BreezeMatrix* result, const BreezeMatrix* a, const BreezeMatrix* b) {
    int i, j, k;
    BreezeMatrix temp;
    
    if (!result || !a || !b) return 0;
    if (a->cols != b->rows) return 0;
    
    /* 使用临时矩阵以支持原地操作 */
    BreezeMatrix_Init(&temp, a->rows, b->cols);
    
    for (i = 0; i < a->rows; i++) {
        for (j = 0; j < b->cols; j++) {
            temp.data[i][j] = 0.0f;
            for (k = 0; k < a->cols; k++) {
                temp.data[i][j] += a->data[i][k] * b->data[k][j];
            }
        }
    }
    
    /* 复制结果 */
    result->rows = a->rows;
    result->cols = b->cols;
    
    for (i = 0; i < result->rows; i++) {
        for (j = 0; j < result->cols; j++) {
            result->data[i][j] = temp.data[i][j];
        }
    }
    
    return 1;
}

/**
 * @brief 矩阵标量乘法
 *
 * @param result 结果矩阵
 * @param a 输入矩阵
 * @param scalar 标量值
 */
static inline void BreezeMatrix_ScalarMultiply(BreezeMatrix* result, const BreezeMatrix* a, float scalar) {
    int i, j;
    
    if (!result || !a) return;
    
    result->rows = a->rows;
    result->cols = a->cols;
    
    for (i = 0; i < a->rows; i++) {
        for (j = 0; j < a->cols; j++) {
            result->data[i][j] = a->data[i][j] * scalar;
        }
    }
}

/**
 * @brief 矩阵转置
 *
 * @param result 结果矩阵
 * @param a 输入矩阵
 */
static inline void BreezeMatrix_Transpose(BreezeMatrix* result, const BreezeMatrix* a) {
    int i, j;
    BreezeMatrix temp;
    
    if (!result || !a) return;
    
    /* 使用临时矩阵以支持原地操作 */
    BreezeMatrix_Init(&temp, a->cols, a->rows);
    
    for (i = 0; i < a->rows; i++) {
        for (j = 0; j < a->cols; j++) {
            temp.data[j][i] = a->data[i][j];
        }
    }
    
    /* 复制结果 */
    result->rows = a->cols;
    result->cols = a->rows;
    
    for (i = 0; i < result->rows; i++) {
        for (j = 0; j < result->cols; j++) {
            result->data[i][j] = temp.data[i][j];
        }
    }
}

/**
 * @brief 计算2x2矩阵的行列式
 *
 * @param a 输入矩阵
 * @return 行列式值
 */
static inline float BreezeMatrix_Determinant2x2(const BreezeMatrix* a) {
    if (!a || a->rows != 2 || a->cols != 2) return 0.0f;
    
    return a->data[0][0] * a->data[1][1] - a->data[0][1] * a->data[1][0];
}

/**
 * @brief 计算3x3矩阵的行列式
 *
 * @param a 输入矩阵
 * @return 行列式值
 */
static inline float BreezeMatrix_Determinant3x3(const BreezeMatrix* a) {
    float det;
    
    if (!a || a->rows != 3 || a->cols != 3) return 0.0f;
    
    det = a->data[0][0] * (a->data[1][1] * a->data[2][2] - a->data[1][2] * a->data[2][1])
        - a->data[0][1] * (a->data[1][0] * a->data[2][2] - a->data[1][2] * a->data[2][0])
        + a->data[0][2] * (a->data[1][0] * a->data[2][1] - a->data[1][1] * a->data[2][0]);
    
    return det;
}

/**
 * @brief 计算矩阵的行列式（支持2x2和3x3矩阵）
 *
 * @param a 输入矩阵
 * @return 行列式值，如果矩阵不是方阵或不支持的大小则返回0
 */
static inline float BreezeMatrix_Determinant(const BreezeMatrix* a) {
    if (!a || a->rows != a->cols) return 0.0f;
    
    if (a->rows == 2) {
        return BreezeMatrix_Determinant2x2(a);
    } else if (a->rows == 3) {
        return BreezeMatrix_Determinant3x3(a);
    }
    
    /* 4x4及更大矩阵的行列式计算较为复杂，这里不实现 */
    return 0.0f;
}

/**
 * @brief 计算2x2矩阵的逆
 *
 * @param result 结果矩阵
 * @param a 输入矩阵
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Inverse2x2(BreezeMatrix* result, const BreezeMatrix* a) {
    float det;
    
    if (!result || !a || a->rows != 2 || a->cols != 2) return 0;
    
    det = BreezeMatrix_Determinant2x2(a);
    
    if (fabsf(det) < 1e-6f) return 0;  /* 矩阵接近奇异 */
    
    result->rows = 2;
    result->cols = 2;
    
    result->data[0][0] = a->data[1][1] / det;
    result->data[0][1] = -a->data[0][1] / det;
    result->data[1][0] = -a->data[1][0] / det;
    result->data[1][1] = a->data[0][0] / det;
    
    return 1;
}

/**
 * @brief 计算3x3矩阵的逆
 *
 * @param result 结果矩阵
 * @param a 输入矩阵
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Inverse3x3(BreezeMatrix* result, const BreezeMatrix* a) {
    float det;
    
    if (!result || !a || a->rows != 3 || a->cols != 3) return 0;
    
    det = BreezeMatrix_Determinant3x3(a);
    
    if (fabsf(det) < 1e-6f) return 0;  /* 矩阵接近奇异 */
    
    result->rows = 3;
    result->cols = 3;
    
    /* 计算伴随矩阵的元素 */
    result->data[0][0] = (a->data[1][1] * a->data[2][2] - a->data[1][2] * a->data[2][1]) / det;
    result->data[0][1] = (a->data[0][2] * a->data[2][1] - a->data[0][1] * a->data[2][2]) / det;
    result->data[0][2] = (a->data[0][1] * a->data[1][2] - a->data[0][2] * a->data[1][1]) / det;
    
    result->data[1][0] = (a->data[1][2] * a->data[2][0] - a->data[1][0] * a->data[2][2]) / det;
    result->data[1][1] = (a->data[0][0] * a->data[2][2] - a->data[0][2] * a->data[2][0]) / det;
    result->data[1][2] = (a->data[0][2] * a->data[1][0] - a->data[0][0] * a->data[1][2]) / det;
    
    result->data[2][0] = (a->data[1][0] * a->data[2][1] - a->data[1][1] * a->data[2][0]) / det;
    result->data[2][1] = (a->data[0][1] * a->data[2][0] - a->data[0][0] * a->data[2][1]) / det;
    result->data[2][2] = (a->data[0][0] * a->data[1][1] - a->data[0][1] * a->data[1][0]) / det;
    
    return 1;
}

/**
 * @brief 计算矩阵的逆（支持2x2和3x3矩阵）
 *
 * @param result 结果矩阵
 * @param a 输入矩阵
 * @return 成功返回1，失败返回0
 */
static inline int BreezeMatrix_Inverse(BreezeMatrix* result, const BreezeMatrix* a) {
    if (!result || !a || a->rows != a->cols) return 0;
    
    if (a->rows == 2) {
        return BreezeMatrix_Inverse2x2(result, a);
    } else if (a->rows == 3) {
        return BreezeMatrix_Inverse3x3(result, a);
    }
    
    /* 4x4及更大矩阵的逆计算较为复杂，这里不实现 */
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_MATRIX_H */
