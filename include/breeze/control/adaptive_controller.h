/**
 * @file adaptive_controller.h
 * @brief 自适应控制器实现
 *
 * 该实现提供了自适应控制器，包括模型参考自适应控制(MRAC)和自调整PID控制器。
 * 自适应控制器能够根据系统响应自动调整控制参数，适用于参数不确定或时变的系统。
 */

#ifndef BREEZE_ADAPTIVE_CONTROLLER_H
#define BREEZE_ADAPTIVE_CONTROLLER_H

#include <math.h>  /* fabsf */
#include "pid_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 模型参考自适应控制器结构体
 */
typedef struct {
    /* 参考模型参数 */
    float a_m;                /* 参考模型特征多项式系数 */
    float b_m;                /* 参考模型输入增益 */
    
    /* 控制器参数 */
    float theta[2];           /* 自适应参数 [theta1, theta2] */
    float gamma[2];           /* 自适应增益 [gamma1, gamma2] */
    
    /* 状态变量 */
    float x_m;                /* 参考模型状态 */
    float x;                  /* 系统状态 */
    float prev_x;             /* 上一次系统状态 */
    float prev_u;             /* 上一次控制输入 */
    
    /* 控制限制 */
    float u_min;              /* 最小控制输入 */
    float u_max;              /* 最大控制输入 */
    
    float dt;                 /* 时间步长（秒） */
} BreezeMRAC;

/**
 * @brief 自调整PID控制器结构体
 */
typedef struct {
    BreezePIDController pid;  /* 基础PID控制器 */
    
    /* 自适应参数 */
    float kp_min;             /* 最小比例增益 */
    float kp_max;             /* 最大比例增益 */
    float ki_min;             /* 最小积分增益 */
    float ki_max;             /* 最大积分增益 */
    float kd_min;             /* 最小微分增益 */
    float kd_max;             /* 最大微分增益 */
    
    float adaptation_rate;    /* 自适应速率 */
    float error_threshold;    /* 误差阈值，超过此值时进行自适应 */
    
    float prev_error;         /* 上一次误差 */
    float prev_output;        /* 上一次输出 */
    
    int adaptation_counter;   /* 自适应计数器 */
    int adaptation_period;    /* 自适应周期（控制步数） */
} BreezeAdaptivePID;

/**
 * @brief 初始化模型参考自适应控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param a_m 参考模型特征多项式系数
 * @param b_m 参考模型输入增益
 * @param gamma1 自适应增益1
 * @param gamma2 自适应增益2
 * @param u_min 最小控制输入
 * @param u_max 最大控制输入
 * @param dt 时间步长（秒）
 */
static inline void BreezeMRAC_Init(
    BreezeMRAC* controller,
    float a_m, float b_m,
    float gamma1, float gamma2,
    float u_min, float u_max,
    float dt
) {
    if (!controller) return;
    
    controller->a_m = a_m;
    controller->b_m = b_m;
    
    controller->theta[0] = 0.0f;
    controller->theta[1] = 0.0f;
    
    controller->gamma[0] = gamma1;
    controller->gamma[1] = gamma2;
    
    controller->x_m = 0.0f;
    controller->x = 0.0f;
    controller->prev_x = 0.0f;
    controller->prev_u = 0.0f;
    
    controller->u_min = u_min;
    controller->u_max = u_max;
    
    controller->dt = dt;
}

/**
 * @brief 更新模型参考自适应控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param r 参考输入
 * @param y 系统输出
 * @return 控制输入
 */
static inline float BreezeMRAC_Update(BreezeMRAC* controller, float r, float y) {
    float u;
    float error;
    
    if (!controller) return 0.0f;
    
    /* 更新系统状态 */
    controller->prev_x = controller->x;
    controller->x = y;
    
    /* 更新参考模型状态 */
    controller->x_m = (1.0f + controller->a_m * controller->dt) * controller->x_m + 
                     controller->b_m * controller->dt * r;
    
    /* 计算跟踪误差 */
    error = controller->x_m - controller->x;
    
    /* 更新自适应参数 */
    controller->theta[0] += controller->gamma[0] * error * r * controller->dt;
    controller->theta[1] += controller->gamma[1] * error * controller->x * controller->dt;
    
    /* 计算控制输入 */
    u = controller->theta[0] * r + controller->theta[1] * controller->x;
    
    /* 限制控制输入范围 */
    if (u > controller->u_max) {
        u = controller->u_max;
    } else if (u < controller->u_min) {
        u = controller->u_min;
    }
    
    controller->prev_u = u;
    
    return u;
}

/**
 * @brief 初始化自调整PID控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param type PID控制器类型
 * @param kp 初始比例增益
 * @param ki 初始积分增益
 * @param kd 初始微分增益
 * @param dt 时间步长（秒）
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 * @param adaptation_rate 自适应速率（0.0到1.0）
 * @param error_threshold 误差阈值
 * @param adaptation_period 自适应周期（控制步数）
 */
static inline void BreezeAdaptivePID_Init(
    BreezeAdaptivePID* controller,
    BreezePIDType type,
    float kp, float ki, float kd,
    float dt,
    float output_min, float output_max,
    float adaptation_rate,
    float error_threshold,
    int adaptation_period
) {
    if (!controller) return;
    
    /* 初始化基础PID控制器 */
    BreezePIDController_Init(&controller->pid, type, kp, ki, kd, dt, output_min, output_max);
    
    /* 设置自适应参数范围 */
    controller->kp_min = kp * 0.1f;
    controller->kp_max = kp * 5.0f;
    controller->ki_min = ki * 0.1f;
    controller->ki_max = ki * 5.0f;
    controller->kd_min = kd * 0.1f;
    controller->kd_max = kd * 5.0f;
    
    controller->adaptation_rate = adaptation_rate;
    controller->error_threshold = error_threshold;
    
    controller->prev_error = 0.0f;
    controller->prev_output = 0.0f;
    
    controller->adaptation_counter = 0;
    controller->adaptation_period = adaptation_period;
}

/**
 * @brief 更新自调整PID控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param setpoint 设定值
 * @param measurement 测量值
 * @return 控制输出
 */
static inline float BreezeAdaptivePID_Compute(
    BreezeAdaptivePID* controller,
    float setpoint, float measurement
) {
    float output;
    float error;
    float error_change;
    
    if (!controller) return 0.0f;
    
    /* 设置PID控制器的设定值 */
    controller->pid.setpoint = setpoint;
    
    /* 计算当前误差 */
    error = setpoint - measurement;
    
    /* 使用基础PID控制器计算输出 */
    output = BreezePIDController_Compute(&controller->pid, measurement);
    
    /* 计算误差变化量 */
    error_change = error - controller->prev_error;
    /* 注意：这里原先还算了 output_change = output - prev_output，但它从未被读过。
     * 下面的自适应规则只用 error / prev_error / error_change，输出变化量不参与。
     * 已删除该局部变量；prev_output 字段仍然照旧保存，属对外可见状态。
     * 详见 docs/REVIEW.md 里记录的这处"注释与代码不一致"。 */
    
    /* 自适应逻辑 */
    controller->adaptation_counter++;
    
    if (controller->adaptation_counter >= controller->adaptation_period) {
        controller->adaptation_counter = 0;
        
        /* 只有当误差超过阈值时才进行自适应 */
        if (fabsf(error) > controller->error_threshold) {
            float kp_delta = 0.0f;
            float ki_delta = 0.0f;
            float kd_delta = 0.0f;
            
            /* 基于误差及其变化调整PID参数
             * （原文写的是"基于误差和输出变化"，但代码从未读过输出变化量） */
            /* 比例项调整：误差大且同向时增加Kp，误差小或反向时减小Kp */
            if (error * controller->prev_error > 0.0f && fabsf(error) > fabsf(controller->prev_error)) {
                kp_delta = controller->adaptation_rate * 0.1f;
            } else {
                kp_delta = -controller->adaptation_rate * 0.05f;
            }
            
            /* 积分项调整：持续误差时增加Ki，振荡时减小Ki */
            if (error * controller->prev_error > 0.0f) {
                ki_delta = controller->adaptation_rate * 0.05f;
            } else {
                ki_delta = -controller->adaptation_rate * 0.1f;
            }
            
            /* 微分项调整：快速变化时增加Kd，缓慢变化时减小Kd */
            if (fabsf(error_change) > fabsf(controller->error_threshold)) {
                kd_delta = controller->adaptation_rate * 0.1f;
            } else {
                kd_delta = -controller->adaptation_rate * 0.05f;
            }
            
            /* 应用参数调整 */
            controller->pid.kp += kp_delta;
            controller->pid.ki += ki_delta;
            controller->pid.kd += kd_delta;
            
            /* 确保参数在允许范围内 */
            if (controller->pid.kp < controller->kp_min) controller->pid.kp = controller->kp_min;
            if (controller->pid.kp > controller->kp_max) controller->pid.kp = controller->kp_max;
            
            if (controller->pid.ki < controller->ki_min) controller->pid.ki = controller->ki_min;
            if (controller->pid.ki > controller->ki_max) controller->pid.ki = controller->ki_max;
            
            if (controller->pid.kd < controller->kd_min) controller->pid.kd = controller->kd_min;
            if (controller->pid.kd > controller->kd_max) controller->pid.kd = controller->kd_max;
        }
    }
    
    /* 保存当前值用于下一次迭代 */
    controller->prev_error = error;
    controller->prev_output = output;
    
    return output;
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_ADAPTIVE_CONTROLLER_H */
