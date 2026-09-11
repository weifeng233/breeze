/**
 * @file pid_controller.h
 * @brief PID控制器实现
 *
 * 该实现提供了位置式（绝对式）和增量式（速度式）两种PID控制器，
 * 具有抗积分饱和保护和微分滤波功能。
 */

#ifndef BREEZE_PID_CONTROLLER_H
#define BREEZE_PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID控制器类型枚举
 */
typedef enum {
    BREEZE_PID_POSITION,    /* 位置式（绝对式）PID控制器 */
    BREEZE_PID_INCREMENTAL  /* 增量式（速度式）PID控制器 */
} BreezePIDType;

/**
 * @brief PID控制器状态结构体
 */
typedef struct {
    BreezePIDType type;     /* 控制器类型（位置式或增量式） */

    float kp;               /* 比例增益 */
    float ki;               /* 积分增益 */
    float kd;               /* 微分增益 */

    float setpoint;         /* 设定值 */
    float integral;         /* 累积误差（用于位置式） */
    float prev_error;       /* 上一次误差值 */
    float prev_prev_error;  /* 上上次误差值（用于增量式） */
    float prev_measurement; /* 上一次测量值（用于滤波微分） */
    float prev_output;      /* 上一次输出值（用于增量式） */

    float output_min;       /* 输出最小值 */
    float output_max;       /* 输出最大值 */

    float integral_min;     /* 积分最小值（抗积分饱和） */
    float integral_max;     /* 积分最大值（抗积分饱和） */

    float alpha;            /* 微分滤波系数（0.0到1.0） */
    float dt;               /* 时间步长（秒） */
} BreezePIDController;

/**
 * @brief 初始化PID控制器
 *
 * @param pid 指向PID控制器结构体的指针
 * @param type 控制器类型（位置式或增量式）
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 * @param dt 时间步长（秒）
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 */
static void BreezePIDController_Init(
    BreezePIDController* pid,
    BreezePIDType type,
    float kp, float ki, float kd,
    float dt,
    float output_min, float output_max
) {
    if (pid) {
        pid->type = type;
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;

        pid->setpoint = 0.0f;
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->prev_prev_error = 0.0f;
        pid->prev_measurement = 0.0f;
        pid->prev_output = 0.0f;

        pid->output_min = output_min;
        pid->output_max = output_max;

        pid->integral_min = output_min;
        pid->integral_max = output_max;

        pid->alpha = 0.1f;  /* 默认微分滤波系数值 */
        pid->dt = dt;
    }
}

/**
 * @brief 设置微分滤波系数
 *
 * @param pid 指向PID控制器结构体的指针
 * @param alpha 滤波系数（0.0到1.0）- 值越低滤波效果越强
 */
static void BreezePIDController_SetDerivativeFilter(BreezePIDController* pid, float alpha) {
    if (pid) {
        /* 将alpha限制在0和1之间 */
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;

        pid->alpha = alpha;
    }
}

/**
 * @brief 设置积分限制以防止积分饱和
 *
 * @param pid 指向PID控制器结构体的指针
 * @param integral_min 积分最小值
 * @param integral_max 积分最大值
 */
static void BreezePIDController_SetIntegralLimits(
    BreezePIDController* pid,
    float integral_min, float integral_max
) {
    if (pid) {
        pid->integral_min = integral_min;
        pid->integral_max = integral_max;
    }
}

/**
 * @brief 设置控制器目标值
 *
 * @param pid 指向PID控制器结构体的指针
 * @param setpoint 期望值
 */
static void BreezePIDController_SetSetpoint(BreezePIDController* pid, float setpoint) {
    if (pid) {
        pid->setpoint = setpoint;
    }
}

/**
 * @brief 重置控制器状态
 *
 * @param pid 指向PID控制器结构体的指针
 */
static void BreezePIDController_Reset(BreezePIDController* pid) {
    if (pid) {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        pid->prev_prev_error = 0.0f;
        pid->prev_measurement = 0.0f;
        pid->prev_output = 0.0f;
    }
}

/**
 * @brief 使用位置式（绝对式）计算PID控制输出
 *
 * @param pid 指向PID控制器结构体的指针
 * @param measurement 当前过程值
 * @return 控制输出
 */
static float BreezePIDController_ComputePosition(BreezePIDController* pid, float measurement) {
    float error, p_term, i_term, d_term;
    float derivative, filtered_derivative;
    float output;

    if (!pid) return 0.0f;

    /* 计算误差 */
    error = pid->setpoint - measurement;

    /* 比例项 */
    p_term = pid->kp * error;

    /* 带抗积分饱和的积分项 */
    pid->integral += error * pid->dt;

    /* 应用积分限制（抗积分饱和） */
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < pid->integral_min) {
        pid->integral = pid->integral_min;
    }

    i_term = pid->ki * pid->integral;

    /* 微分项（基于测量值以避免微分突跳） */
    derivative = (measurement - pid->prev_measurement) / pid->dt;
    filtered_derivative = pid->alpha * derivative + (1.0f - pid->alpha) * (pid->prev_error - error) / pid->dt;
    d_term = -pid->kd * filtered_derivative;  /* 负号是因为我们使用的是测量值 */

    /* 保存值用于下一次迭代 */
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    /* 计算总输出 */
    output = p_term + i_term + d_term;

    /* 应用输出限制 */
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

/**
 * @brief 使用增量式（速度式）计算PID控制输出
 *
 * @param pid 指向PID控制器结构体的指针
 * @param measurement 当前过程值
 * @return 需要添加到上一次输出的控制输出增量
 */
static float BreezePIDController_ComputeIncremental(BreezePIDController* pid, float measurement) {
    float error, delta_u;
    float delta_p, delta_i, delta_d;
    float output;

    if (!pid) return 0.0f;

    /* 计算误差 */
    error = pid->setpoint - measurement;

    /* 计算增量项 */
    delta_p = pid->kp * (error - pid->prev_error);
    delta_i = pid->ki * error * pid->dt;
    delta_d = pid->kd * (error - 2.0f * pid->prev_error + pid->prev_prev_error) / (pid->dt * pid->dt);

    /* 计算输出增量 */
    delta_u = delta_p + delta_i + delta_d;

    /* 更新误差历史 */
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;

    /* 通过将增量添加到上一次输出来计算新输出 */
    output = pid->prev_output + delta_u;

    /* 应用输出限制 */
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    /* 保存当前输出用于下一次迭代 */
    pid->prev_output = output;

    return output;
}

/**
 * @brief 根据控制器类型计算PID控制输出
 *
 * @param pid 指向PID控制器结构体的指针
 * @param measurement 当前过程值
 * @return 控制输出
 */
static float BreezePIDController_Compute(BreezePIDController* pid, float measurement) {
    if (!pid) return 0.0f;

    if (pid->type == BREEZE_PID_POSITION) {
        return BreezePIDController_ComputePosition(pid, measurement);
    } else {
        return BreezePIDController_ComputeIncremental(pid, measurement);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_PID_CONTROLLER_H */
