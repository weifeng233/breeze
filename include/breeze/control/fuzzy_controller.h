/**
 * @file fuzzy_controller.h
 * @brief 模糊控制器实现
 *
 * 该实现提供了一个简单的模糊控制器，用于处理非线性系统和不确定性。
 * 模糊控制器基于模糊逻辑，使用语言规则而不是精确的数学模型来控制系统。
 */

#ifndef BREEZE_FUZZY_CONTROLLER_H
#define BREEZE_FUZZY_CONTROLLER_H

#include <math.h>    /* expf */
#include <stddef.h>  /* NULL */
#include <stdlib.h>  /* malloc, free */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 模糊集合成员函数类型
 */
typedef enum {
    BREEZE_FUZZY_TRIANGULAR,  /* 三角形成员函数 */
    BREEZE_FUZZY_TRAPEZOIDAL, /* 梯形成员函数 */
    BREEZE_FUZZY_GAUSSIAN     /* 高斯成员函数 */
} BreezeFuzzyMembershipType;

/**
 * @brief 模糊集合成员函数结构体
 */
typedef struct {
    BreezeFuzzyMembershipType type; /* 成员函数类型 */
    float params[4];                /* 成员函数参数 */
    const char* name;               /* 语言变量名称 */
} BreezeFuzzyMembership;

/**
 * @brief 模糊规则结构体
 */
typedef struct {
    int input1_idx;                 /* 输入1的模糊集合索引 */
    int input2_idx;                 /* 输入2的模糊集合索引 */
    int output_idx;                 /* 输出的模糊集合索引 */
} BreezeFuzzyRule;

/**
 * @brief 模糊控制器结构体
 */
typedef struct {
    /* 输入1的模糊集合 */
    BreezeFuzzyMembership* input1_memberships;
    int input1_membership_count;
    
    /* 输入2的模糊集合 */
    BreezeFuzzyMembership* input2_memberships;
    int input2_membership_count;
    
    /* 输出的模糊集合 */
    BreezeFuzzyMembership* output_memberships;
    int output_membership_count;
    
    /* 模糊规则 */
    BreezeFuzzyRule* rules;
    int rule_count;
    
    /* 输入范围 */
    float input1_min;
    float input1_max;
    float input2_min;
    float input2_max;
    
    /* 输出范围 */
    float output_min;
    float output_max;
    
    /* 离散化参数 */
    int discretization_level;
    float* output_discretization;
} BreezeFuzzyController;

/**
 * @brief 初始化模糊控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param input1_min 输入1的最小值
 * @param input1_max 输入1的最大值
 * @param input2_min 输入2的最小值
 * @param input2_max 输入2的最大值
 * @param output_min 输出的最小值
 * @param output_max 输出的最大值
 * @param discretization_level 离散化级别（越高越精确，但计算量越大）
 */
static inline void BreezeFuzzyController_Init(
    BreezeFuzzyController* controller,
    float input1_min, float input1_max,
    float input2_min, float input2_max,
    float output_min, float output_max,
    int discretization_level
) {
    if (!controller) return;
    
    controller->input1_memberships = NULL;
    controller->input1_membership_count = 0;
    
    controller->input2_memberships = NULL;
    controller->input2_membership_count = 0;
    
    controller->output_memberships = NULL;
    controller->output_membership_count = 0;
    
    controller->rules = NULL;
    controller->rule_count = 0;
    
    controller->input1_min = input1_min;
    controller->input1_max = input1_max;
    controller->input2_min = input2_min;
    controller->input2_max = input2_max;
    controller->output_min = output_min;
    controller->output_max = output_max;
    
    controller->discretization_level = discretization_level;
    controller->output_discretization = NULL;
}

/**
 * @brief 设置输入1的模糊集合
 *
 * @param controller 指向控制器结构体的指针
 * @param memberships 模糊集合数组
 * @param count 模糊集合数量
 */
static inline void BreezeFuzzyController_SetInput1Memberships(
    BreezeFuzzyController* controller,
    BreezeFuzzyMembership* memberships,
    int count
) {
    if (!controller) return;
    
    controller->input1_memberships = memberships;
    controller->input1_membership_count = count;
}

/**
 * @brief 设置输入2的模糊集合
 *
 * @param controller 指向控制器结构体的指针
 * @param memberships 模糊集合数组
 * @param count 模糊集合数量
 */
static inline void BreezeFuzzyController_SetInput2Memberships(
    BreezeFuzzyController* controller,
    BreezeFuzzyMembership* memberships,
    int count
) {
    if (!controller) return;
    
    controller->input2_memberships = memberships;
    controller->input2_membership_count = count;
}

/**
 * @brief 设置输出的模糊集合
 *
 * @param controller 指向控制器结构体的指针
 * @param memberships 模糊集合数组
 * @param count 模糊集合数量
 */
static inline void BreezeFuzzyController_SetOutputMemberships(
    BreezeFuzzyController* controller,
    BreezeFuzzyMembership* memberships,
    int count
) {
    if (!controller) return;
    
    controller->output_memberships = memberships;
    controller->output_membership_count = count;
}

/**
 * @brief 设置模糊规则
 *
 * @param controller 指向控制器结构体的指针
 * @param rules 规则数组
 * @param count 规则数量
 */
static inline void BreezeFuzzyController_SetRules(
    BreezeFuzzyController* controller,
    BreezeFuzzyRule* rules,
    int count
) {
    if (!controller) return;
    
    controller->rules = rules;
    controller->rule_count = count;
}

/**
 * @brief 计算三角形成员函数的隶属度
 *
 * @param x 输入值
 * @param a 左边界
 * @param b 中心点
 * @param c 右边界
 * @return 隶属度（0.0到1.0）
 */
static inline float BreezeFuzzyController_TriangularMembership(
    float x, float a, float b, float c
) {
    if (x <= a || x >= c) return 0.0f;
    if (x <= b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

/**
 * @brief 计算梯形成员函数的隶属度
 *
 * @param x 输入值
 * @param a 左边界
 * @param b 左肩
 * @param c 右肩
 * @param d 右边界
 * @return 隶属度（0.0到1.0）
 */
static inline float BreezeFuzzyController_TrapezoidalMembership(
    float x, float a, float b, float c, float d
) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

/**
 * @brief 计算高斯成员函数的隶属度
 *
 * @param x 输入值
 * @param c 中心点
 * @param sigma 标准差
 * @return 隶属度（0.0到1.0）
 */
static inline float BreezeFuzzyController_GaussianMembership(
    float x, float c, float sigma
) {
    float temp = (x - c) / sigma;
    return expf(-0.5f * temp * temp);
}

/**
 * @brief 计算模糊集合的隶属度
 *
 * @param membership 模糊集合
 * @param x 输入值
 * @return 隶属度（0.0到1.0）
 */
static inline float BreezeFuzzyController_CalculateMembership(
    const BreezeFuzzyMembership* membership,
    float x
) {
    if (!membership) return 0.0f;
    
    switch (membership->type) {
        case BREEZE_FUZZY_TRIANGULAR:
            return BreezeFuzzyController_TriangularMembership(
                x, membership->params[0], membership->params[1], membership->params[2]
            );
        
        case BREEZE_FUZZY_TRAPEZOIDAL:
            return BreezeFuzzyController_TrapezoidalMembership(
                x, membership->params[0], membership->params[1], 
                membership->params[2], membership->params[3]
            );
        
        case BREEZE_FUZZY_GAUSSIAN:
            return BreezeFuzzyController_GaussianMembership(
                x, membership->params[0], membership->params[1]
            );
        
        default:
            return 0.0f;
    }
}

/**
 * @brief 计算模糊控制器输出
 *
 * @param controller 指向控制器结构体的指针
 * @param input1 输入1的值
 * @param input2 输入2的值
 * @return 控制器输出
 */
static inline float BreezeFuzzyController_Compute(
    BreezeFuzzyController* controller,
    float input1, float input2
) {
    int i, j;
    float numerator = 0.0f;
    float denominator = 0.0f;
    float output_value;
    
    if (!controller || !controller->rules || controller->rule_count <= 0) {
        return 0.0f;
    }
    
    /* 限制输入范围 */
    if (input1 < controller->input1_min) input1 = controller->input1_min;
    if (input1 > controller->input1_max) input1 = controller->input1_max;
    if (input2 < controller->input2_min) input2 = controller->input2_min;
    if (input2 > controller->input2_max) input2 = controller->input2_max;
    
    /* 离散化输出空间 */
    if (!controller->output_discretization) {
        controller->output_discretization = (float*)malloc(
            controller->discretization_level * sizeof(float)
        );
        
        if (!controller->output_discretization) {
            return 0.0f;  /* 内存分配失败 */
        }
        
        float step = (controller->output_max - controller->output_min) / 
                    (controller->discretization_level - 1);
        
        for (i = 0; i < controller->discretization_level; i++) {
            controller->output_discretization[i] = controller->output_min + i * step;
        }
    }
    
    /* 计算每个离散输出点的隶属度 */
    float* output_memberships = (float*)malloc(
        controller->discretization_level * sizeof(float)
    );
    
    if (!output_memberships) {
        return 0.0f;  /* 内存分配失败 */
    }
    
    /* 初始化输出隶属度为0 */
    for (i = 0; i < controller->discretization_level; i++) {
        output_memberships[i] = 0.0f;
    }
    
    /* 应用模糊规则 */
    for (i = 0; i < controller->rule_count; i++) {
        BreezeFuzzyRule* rule = &controller->rules[i];
        
        /* 计算输入的隶属度 */
        float input1_membership = BreezeFuzzyController_CalculateMembership(
            &controller->input1_memberships[rule->input1_idx], input1
        );
        
        float input2_membership = BreezeFuzzyController_CalculateMembership(
            &controller->input2_memberships[rule->input2_idx], input2
        );
        
        /* 计算规则的激活度（使用min作为AND操作） */
        float activation = input1_membership < input2_membership ? 
                          input1_membership : input2_membership;
        
        /* 应用规则到输出空间 */
        for (j = 0; j < controller->discretization_level; j++) {
            float output_membership = BreezeFuzzyController_CalculateMembership(
                &controller->output_memberships[rule->output_idx],
                controller->output_discretization[j]
            );
            
            /* 使用max作为OR操作 */
            float truncated = activation < output_membership ? activation : output_membership;
            if (truncated > output_memberships[j]) {
                output_memberships[j] = truncated;
            }
        }
    }
    
    /* 使用重心法进行解模糊化 */
    for (i = 0; i < controller->discretization_level; i++) {
        numerator += controller->output_discretization[i] * output_memberships[i];
        denominator += output_memberships[i];
    }
    
    free(output_memberships);
    
    if (denominator < 0.000001f) {
        output_value = (controller->output_min + controller->output_max) / 2.0f;
    } else {
        output_value = numerator / denominator;
    }
    
    /* 限制输出范围 */
    if (output_value < controller->output_min) output_value = controller->output_min;
    if (output_value > controller->output_max) output_value = controller->output_max;
    
    return output_value;
}

/**
 * @brief 释放模糊控制器资源
 *
 * @param controller 指向控制器结构体的指针
 */
static inline void BreezeFuzzyController_Free(BreezeFuzzyController* controller) {
    if (!controller) return;
    
    if (controller->output_discretization) {
        free(controller->output_discretization);
        controller->output_discretization = NULL;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_FUZZY_CONTROLLER_H */
