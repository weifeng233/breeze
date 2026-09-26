/**
 * @file state_feedback_controller.h
 * @brief 状态反馈控制器实现
 *
 * 该实现提供了状态反馈控制器，包括线性状态反馈和线性二次型调节器(LQR)。
 * 状态反馈控制器基于系统的状态变量进行控制，适用于多输入多输出系统。
 */

#ifndef BREEZE_STATE_FEEDBACK_CONTROLLER_H
#define BREEZE_STATE_FEEDBACK_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 线性状态反馈控制器结构体（最多支持4个状态变量）
 */
typedef struct {
    float K[4];               /* 状态反馈增益向量 */
    float reference;          /* 参考输入 */
    float N;                  /* 前馈增益 */
    
    int state_dim;            /* 状态向量维度 */
    
    float u_min;              /* 最小控制输入 */
    float u_max;              /* 最大控制输入 */
} BreezeStateFeedbackController;

/**
 * @brief 初始化线性状态反馈控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param K 状态反馈增益向量
 * @param state_dim 状态向量维度（最多4）
 * @param N 前馈增益（用于跟踪控制）
 * @param u_min 最小控制输入
 * @param u_max 最大控制输入
 */
static inline void BreezeStateFeedbackController_Init(
    BreezeStateFeedbackController* controller,
    const float* K,
    int state_dim,
    float N,
    float u_min, float u_max
) {
    int i;
    
    if (!controller || !K || state_dim <= 0 || state_dim > 4) return;
    
    for (i = 0; i < state_dim; i++) {
        controller->K[i] = K[i];
    }
    
    controller->state_dim = state_dim;
    controller->reference = 0.0f;
    controller->N = N;
    
    controller->u_min = u_min;
    controller->u_max = u_max;
}

/**
 * @brief 设置参考输入
 *
 * @param controller 指向控制器结构体的指针
 * @param reference 参考输入值
 */
static inline void BreezeStateFeedbackController_SetReference(
    BreezeStateFeedbackController* controller,
    float reference
) {
    if (!controller) return;
    
    controller->reference = reference;
}

/**
 * @brief 计算状态反馈控制输出
 *
 * @param controller 指向控制器结构体的指针
 * @param state 系统状态向量
 * @return 控制输出
 */
static inline float BreezeStateFeedbackController_Compute(
    BreezeStateFeedbackController* controller,
    const float* state
) {
    int i;
    float u = 0.0f;
    
    if (!controller || !state) return 0.0f;
    
    /* 计算状态反馈项 */
    for (i = 0; i < controller->state_dim; i++) {
        u -= controller->K[i] * state[i];
    }
    
    /* 添加前馈项用于参考跟踪 */
    u += controller->N * controller->reference;
    
    /* 限制控制输出范围 */
    if (u > controller->u_max) {
        u = controller->u_max;
    } else if (u < controller->u_min) {
        u = controller->u_min;
    }
    
    return u;
}

/**
 * @brief 线性二次型调节器(LQR)结构体（最多支持4个状态变量）
 */
typedef struct {
    /* 系统模型参数 */
    float A[4][4];            /* 系统矩阵 */
    float B[4];               /* 输入矩阵 */
    
    /* LQR参数 */
    float Q[4];               /* 状态权重（对角线元素） */
    float R;                  /* 控制权重 */
    
    /* 控制器参数 */
    float K[4];               /* 状态反馈增益向量 */
    float reference;          /* 参考输入 */
    float N;                  /* 前馈增益 */
    
    int state_dim;            /* 状态向量维度 */
    int max_iterations;       /* 最大迭代次数 */
    float convergence_tol;    /* 收敛容差 */
    
    float u_min;              /* 最小控制输入 */
    float u_max;              /* 最大控制输入 */
} BreezeLQRController;

/**
 * @brief 初始化LQR控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param A 系统矩阵
 * @param B 输入矩阵
 * @param Q 状态权重（对角线元素）
 * @param R 控制权重
 * @param state_dim 状态向量维度（最多4）
 * @param N 前馈增益（用于跟踪控制）
 * @param u_min 最小控制输入
 * @param u_max 最大控制输入
 * @param max_iterations 最大迭代次数
 * @param convergence_tol 收敛容差
 */
static inline void BreezeLQRController_Init(
    BreezeLQRController* controller,
    const float A[][4],
    const float* B,
    const float* Q,
    float R,
    int state_dim,
    float N,
    float u_min, float u_max,
    int max_iterations,
    float convergence_tol
) {
    int i, j;
    
    if (!controller || !A || !B || !Q || state_dim <= 0 || state_dim > 4) return;
    
    /* 复制系统模型参数 */
    for (i = 0; i < state_dim; i++) {
        for (j = 0; j < state_dim; j++) {
            controller->A[i][j] = A[i][j];
        }
        controller->B[i] = B[i];
        controller->Q[i] = Q[i];
        controller->K[i] = 0.0f;  /* 初始化增益为0 */
    }
    
    controller->R = R;
    controller->state_dim = state_dim;
    controller->reference = 0.0f;
    controller->N = N;
    
    controller->u_min = u_min;
    controller->u_max = u_max;
    
    controller->max_iterations = max_iterations;
    controller->convergence_tol = convergence_tol;
    
    /* 注意：实际的LQR增益计算需要求解黎卡提方程，这通常需要数值方法
     * 在实际应用中，可以使用离线计算的增益值，或者使用简化的迭代方法
     * 完整的LQR求解器需要矩阵运算支持，将在数学工具模块中实现
     */
}

/**
 * @brief 设置LQR控制器的增益向量（通常由外部计算）
 *
 * @param controller 指向控制器结构体的指针
 * @param K 状态反馈增益向量
 */
static inline void BreezeLQRController_SetGains(
    BreezeLQRController* controller,
    const float* K
) {
    int i;
    
    if (!controller || !K) return;
    
    for (i = 0; i < controller->state_dim; i++) {
        controller->K[i] = K[i];
    }
}

/**
 * @brief 设置参考输入
 *
 * @param controller 指向控制器结构体的指针
 * @param reference 参考输入值
 */
static inline void BreezeLQRController_SetReference(
    BreezeLQRController* controller,
    float reference
) {
    if (!controller) return;
    
    controller->reference = reference;
}

/**
 * @brief 计算LQR控制输出
 *
 * @param controller 指向控制器结构体的指针
 * @param state 系统状态向量
 * @return 控制输出
 */
static inline float BreezeLQRController_Compute(
    BreezeLQRController* controller,
    const float* state
) {
    int i;
    float u = 0.0f;
    
    if (!controller || !state) return 0.0f;
    
    /* 计算状态反馈项 */
    for (i = 0; i < controller->state_dim; i++) {
        u -= controller->K[i] * state[i];
    }
    
    /* 添加前馈项用于参考跟踪 */
    u += controller->N * controller->reference;
    
    /* 限制控制输出范围 */
    if (u > controller->u_max) {
        u = controller->u_max;
    } else if (u < controller->u_min) {
        u = controller->u_min;
    }
    
    return u;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_STATE_FEEDBACK_CONTROLLER_H */
