/**
 * @file balance_controller.h
 * @brief 倒立摆平台的平衡控制器
 *
 * 该实现提供了用于平衡倒立摆平台的控制算法，
 * 如自平衡机器人、赛格威等。
 */

#ifndef BREEZE_BALANCE_CONTROLLER_H
#define BREEZE_BALANCE_CONTROLLER_H

#include "../pid_controller.h"
#include "../../filter/complementary_filter.h"
#include "mobile_platform_hal.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 平衡控制器配置
 */
typedef struct {
    float wheel_radius;           /* 轮子半径（米） */
    float wheel_distance;         /* 轮子之间的距离（米） */
    float max_tilt_angle;         /* 最大安全倾斜角度（弧度） */
    float max_speed;              /* 最大前进速度（米/秒） */
    float max_angular_speed;      /* 最大角速度（弧度/秒） */
    float target_tilt_angle;      /* 目标倾斜角度（弧度，通常为0） */
    int left_motor_id;            /* 左电机ID */
    int right_motor_id;           /* 右电机ID */
    int left_encoder_id;          /* 左编码器ID */
    int right_encoder_id;         /* 右编码器ID */
    float encoder_resolution;     /* 每轮旋转的编码器计数 */
} BreezeBalanceControllerConfig;

/**
 * @brief 平衡控制器结构体
 */
typedef struct {
    BreezeBalanceControllerConfig config;  /* 平台配置 */
    BreezePIDController angle_pid;         /* 角度PID控制器 */
    BreezePIDController speed_pid;         /* 速度PID控制器 */
    BreezePIDController turn_pid;          /* 转向PID控制器 */
    BreezeComplementaryFilter imu_filter;  /* 用于角度估计的IMU滤波器 */
    BreezeMotorControlFunc set_motor;      /* 电机控制函数 */
    BreezeEncoderFunc get_encoder;         /* 编码器读取函数 */
    BreezeIMUDataFunc get_imu_data;        /* IMU数据函数 */
    float target_speed;                    /* 目标前进速度（米/秒） */
    float target_turn_rate;                /* 目标转向速率（弧度/秒） */
    float current_speed;                   /* 当前估计速度（米/秒） */
    float current_angle;                   /* 当前估计倾斜角度（弧度） */
    float dt;                              /* 控制循环时间步长（秒） */
} BreezeBalanceController;

/**
 * @brief 初始化平衡控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param config 平台配置
 * @param set_motor 电机控制函数
 * @param get_encoder 编码器读取函数
 * @param get_imu_data IMU数据函数
 * @param dt 控制循环时间步长（秒）
 */
static inline void BreezeBalanceController_Init(
    BreezeBalanceController* controller,
    BreezeBalanceControllerConfig config,
    BreezeMotorControlFunc set_motor,
    BreezeEncoderFunc get_encoder,
    BreezeIMUDataFunc get_imu_data,
    float dt
) {
    if (!controller || !set_motor || !get_encoder || !get_imu_data) return;

    controller->config = config;
    controller->set_motor = set_motor;
    controller->get_encoder = get_encoder;
    controller->get_imu_data = get_imu_data;
    controller->target_speed = 0.0f;
    controller->target_turn_rate = 0.0f;
    controller->current_speed = 0.0f;
    controller->current_angle = 0.0f;
    controller->dt = dt;

    /* 初始化PID控制器 */
    /* 角度PID控制器 - 通常需要快速响应 */
    BreezePIDController_Init(&controller->angle_pid, BREEZE_PID_POSITION,
                            10.0f, 0.0f, 0.1f, dt, -1.0f, 1.0f);

    /* 速度PID控制器 - 通常响应较慢 */
    BreezePIDController_Init(&controller->speed_pid, BREEZE_PID_POSITION,
                            0.5f, 0.05f, 0.0f, dt, -0.5f, 0.5f);

    /* 转向PID控制器 */
    BreezePIDController_Init(&controller->turn_pid, BREEZE_PID_POSITION,
                            1.0f, 0.0f, 0.0f, dt, -0.5f, 0.5f);

    /* 初始化IMU的互补滤波器 */
    BreezeComplementaryFilter_Init(&controller->imu_filter, 0.98f, dt);
}

/**
 * @brief 设置角度PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static inline void BreezeBalanceController_SetAnglePIDParams(
    BreezeBalanceController* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->angle_pid.kp = kp;
    controller->angle_pid.ki = ki;
    controller->angle_pid.kd = kd;
}

/**
 * @brief 设置速度PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static inline void BreezeBalanceController_SetSpeedPIDParams(
    BreezeBalanceController* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->speed_pid.kp = kp;
    controller->speed_pid.ki = ki;
    controller->speed_pid.kd = kd;
}

/**
 * @brief 设置转向PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static inline void BreezeBalanceController_SetTurnPIDParams(
    BreezeBalanceController* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->turn_pid.kp = kp;
    controller->turn_pid.ki = ki;
    controller->turn_pid.kd = kd;
}

/**
 * @brief 设置目标速度和转向速率
 *
 * @param controller 指向控制器结构体的指针
 * @param speed 目标前进速度（米/秒）
 * @param turn_rate 目标转向速率（弧度/秒，正值=逆时针）
 */
static inline void BreezeBalanceController_SetTargets(
    BreezeBalanceController* controller,
    float speed, float turn_rate
) {
    if (!controller) return;

    /* 将速度限制在配置的最大值内 */
    if (speed > controller->config.max_speed) {
        speed = controller->config.max_speed;
    } else if (speed < -controller->config.max_speed) {
        speed = -controller->config.max_speed;
    }

    /* 将转向速率限制在配置的最大值内 */
    if (turn_rate > controller->config.max_angular_speed) {
        turn_rate = controller->config.max_angular_speed;
    } else if (turn_rate < -controller->config.max_angular_speed) {
        turn_rate = -controller->config.max_angular_speed;
    }

    controller->target_speed = speed;
    controller->target_turn_rate = turn_rate;
}

/**
 * @brief 将编码器计数转换为轮速
 *
 * @param controller 指向控制器结构体的指针
 * @param encoder_counts 自上次更新以来的编码器计数
 * @return 轮速（米/秒）
 */
static inline float BreezeBalanceController_EncoderToSpeed(
    BreezeBalanceController* controller,
    float encoder_counts
) {
    float wheel_circumference;
    float wheel_revolutions;
    float wheel_speed;

    if (!controller) return 0.0f;

    wheel_circumference = 2.0f * 3.14159f * controller->config.wheel_radius;
    wheel_revolutions = encoder_counts / controller->config.encoder_resolution;
    wheel_speed = wheel_revolutions * wheel_circumference / controller->dt;

    return wheel_speed;
}

/**
 * @brief 更新平衡控制器
 *
 * 该函数应以dt指定的速率定期调用。
 * 它读取IMU和编码器，更新角度估计，使用级联PID控制器
 * 计算控制输出，并设置电机速度。
 *
 * @param controller 指向控制器结构体的指针
 * @return 如果保持平衡返回1，如果倾斜角度超过安全限制返回0
 */
static inline int BreezeBalanceController_Update(BreezeBalanceController* controller) {
    BreezeIMUData imu_data;
    float left_encoder_counts, right_encoder_counts;
    float left_wheel_speed, right_wheel_speed;
    float angle_output, speed_output, turn_output;
    float left_motor_output, right_motor_output;

    if (!controller || !controller->set_motor || !controller->get_encoder || !controller->get_imu_data) {
        return 0;
    }

    /* 读取IMU数据 */
    if (!controller->get_imu_data(&imu_data)) {
        return 0; /* IMU读取失败 */
    }

    /* 更新互补滤波器以获取俯仰角 */
    BreezeComplementaryFilter_Update(
        &controller->imu_filter,
        imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z
    );

    /* 获取俯仰角（假设这是平衡轴） */
    controller->current_angle = BreezeComplementaryFilter_GetPitch(&controller->imu_filter);

    /* 检查角度是否超过安全限制 */
    if (fabsf(controller->current_angle) > controller->config.max_tilt_angle) {
        /* 为安全起见停止电机 */
        controller->set_motor(controller->config.left_motor_id, 0.0f);
        controller->set_motor(controller->config.right_motor_id, 0.0f);
        return 0; /* 平衡丢失 */
    }

    /* 读取编码器并计算当前速度 */
    left_encoder_counts = controller->get_encoder(controller->config.left_encoder_id, 1);
    right_encoder_counts = controller->get_encoder(controller->config.right_encoder_id, 1);

    left_wheel_speed = BreezeBalanceController_EncoderToSpeed(controller, left_encoder_counts);
    right_wheel_speed = BreezeBalanceController_EncoderToSpeed(controller, right_encoder_counts);

    /* 平均轮速以获得机器人速度 */
    controller->current_speed = (left_wheel_speed + right_wheel_speed) / 2.0f;

    /* 级联PID控制:
     * 1. 速度PID控制器输出目标角度
     * 2. 角度PID控制器输出电机功率
     */

    /* 速度PID - 输出是对目标角度的调整 */
    BreezePIDController_SetSetpoint(&controller->speed_pid, controller->target_speed);
    speed_output = BreezePIDController_Compute(&controller->speed_pid, controller->current_speed);

    /* 角度PID - 设定点是目标角度加上速度调整 */
    BreezePIDController_SetSetpoint(&controller->angle_pid,
                                  controller->config.target_tilt_angle + speed_output);
    angle_output = BreezePIDController_Compute(&controller->angle_pid, controller->current_angle);

    /* 转向PID - 输出是对电机的差分调整 */
    BreezePIDController_SetSetpoint(&controller->turn_pid, controller->target_turn_rate);
    turn_output = BreezePIDController_Compute(&controller->turn_pid,
                                           (right_wheel_speed - left_wheel_speed) /
                                           controller->config.wheel_distance);

    /* 组合输出用于最终电机控制 */
    left_motor_output = angle_output - turn_output;
    right_motor_output = angle_output + turn_output;

    /* 将输出限制在-1.0到1.0范围内 */
    if (left_motor_output > 1.0f) left_motor_output = 1.0f;
    if (left_motor_output < -1.0f) left_motor_output = -1.0f;

    if (right_motor_output > 1.0f) right_motor_output = 1.0f;
    if (right_motor_output < -1.0f) right_motor_output = -1.0f;

    /* 设置电机输出 */
    controller->set_motor(controller->config.left_motor_id, left_motor_output);
    controller->set_motor(controller->config.right_motor_id, right_motor_output);

    return 1; /* Balance maintained */
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_BALANCE_CONTROLLER_H */
