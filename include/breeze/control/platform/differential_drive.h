/**
 * @file differential_drive.h
 * @brief 差速驱动（坦克式）平台控制器
 *
 * 该实现提供了差速驱动平台的控制算法（也称为滑移转向或坦克式驱动），
 * 其中两侧的两个独立驱动的轮子或履带同时控制速度和方向。
 */

#ifndef BREEZE_DIFFERENTIAL_DRIVE_H
#define BREEZE_DIFFERENTIAL_DRIVE_H

#include "../pid_controller.h"
#include "mobile_platform_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 差速驱动控制器配置
 */
typedef struct {
    float wheel_distance;       /* 左右轮之间的距离（米） */
    float wheel_radius;         /* 轮子半径（米） */
    float max_linear_speed;     /* 最大线速度（米/秒） */
    float max_angular_speed;    /* 最大角速度（弧度/秒） */
    int left_motor_id;          /* 左电机ID */
    int right_motor_id;         /* 右电机ID */
    int left_encoder_id;        /* 左编码器ID */
    int right_encoder_id;       /* 右编码器ID */
    float encoder_resolution;   /* 每轮旋转的编码器计数 */
} BreezeDifferentialDriveConfig;

/**
 * @brief 差速驱动控制器结构体
 */
typedef struct {
    BreezeDifferentialDriveConfig config;  /* 平台配置 */
    BreezePIDController left_pid;          /* 左轮PID控制器 */
    BreezePIDController right_pid;         /* 右轮PID控制器 */
    BreezeMotorControlFunc set_motor;      /* 电机控制函数 */
    BreezeEncoderFunc get_encoder;         /* 编码器读取函数 */
    float target_linear_speed;             /* 目标线速度（米/秒） */
    float target_angular_speed;            /* 目标角速度（弧度/秒） */
    float dt;                              /* 控制循环时间步长（秒） */
} BreezeDifferentialDrive;

/**
 * @brief 初始化差速驱动控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param config 平台配置
 * @param set_motor 电机控制函数
 * @param get_encoder 编码器读取函数
 * @param dt 控制循环时间步长（秒）
 */
static void BreezeDifferentialDrive_Init(
    BreezeDifferentialDrive* controller,
    BreezeDifferentialDriveConfig config,
    BreezeMotorControlFunc set_motor,
    BreezeEncoderFunc get_encoder,
    float dt
) {
    float max_wheel_speed;

    if (!controller || !set_motor || !get_encoder) return;

    controller->config = config;
    controller->set_motor = set_motor;
    controller->get_encoder = get_encoder;
    controller->target_linear_speed = 0.0f;
    controller->target_angular_speed = 0.0f;
    controller->dt = dt;

    /* 计算最大轮速 */
    max_wheel_speed = config.max_linear_speed +
                     (config.max_angular_speed * config.wheel_distance / 2.0f);

    /* 初始化两个轮子的PID控制器 */
    BreezePIDController_Init(&controller->left_pid, BREEZE_PID_POSITION,
                            1.0f, 0.1f, 0.05f, dt, -1.0f, 1.0f);

    BreezePIDController_Init(&controller->right_pid, BREEZE_PID_POSITION,
                            1.0f, 0.1f, 0.05f, dt, -1.0f, 1.0f);
}

/**
 * @brief 设置两个轮子的PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static void BreezeDifferentialDrive_SetPIDParams(
    BreezeDifferentialDrive* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->left_pid.kp = kp;
    controller->left_pid.ki = ki;
    controller->left_pid.kd = kd;

    controller->right_pid.kp = kp;
    controller->right_pid.ki = ki;
    controller->right_pid.kd = kd;
}

/**
 * @brief 设置差速驱动平台的目标速度
 *
 * @param controller 指向控制器结构体的指针
 * @param linear_speed 目标线速度（米/秒）
 * @param angular_speed 目标角速度（弧度/秒，正值=逆时针）
 */
static void BreezeDifferentialDrive_SetSpeed(
    BreezeDifferentialDrive* controller,
    float linear_speed,
    float angular_speed
) {
    if (!controller) return;

    /* 将速度限制在配置的最大值内 */
    if (linear_speed > controller->config.max_linear_speed) {
        linear_speed = controller->config.max_linear_speed;
    } else if (linear_speed < -controller->config.max_linear_speed) {
        linear_speed = -controller->config.max_linear_speed;
    }

    if (angular_speed > controller->config.max_angular_speed) {
        angular_speed = controller->config.max_angular_speed;
    } else if (angular_speed < -controller->config.max_angular_speed) {
        angular_speed = -controller->config.max_angular_speed;
    }

    controller->target_linear_speed = linear_speed;
    controller->target_angular_speed = angular_speed;
}

/**
 * @brief 将编码器计数转换为轮速
 *
 * @param controller 指向控制器结构体的指针
 * @param encoder_counts 自上次更新以来的编码器计数
 * @return 轮速（米/秒）
 */
static float BreezeDifferentialDrive_EncoderToSpeed(
    BreezeDifferentialDrive* controller,
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
 * @brief 更新差速驱动控制器
 *
 * 该函数应以dt指定的速率定期调用。
 * 它读取编码器，使用PID控制器计算控制输出，
 * 并设置电机速度。
 *
 * @param controller 指向控制器结构体的指针
 */
static void BreezeDifferentialDrive_Update(BreezeDifferentialDrive* controller) {
    float left_target_speed, right_target_speed;
    float left_current_speed, right_current_speed;
    float left_encoder_counts, right_encoder_counts;
    float left_output, right_output;

    if (!controller || !controller->set_motor || !controller->get_encoder) return;

    /* 根据线速度和角速度计算目标轮速 */
    left_target_speed = controller->target_linear_speed -
                       (controller->target_angular_speed * controller->config.wheel_distance / 2.0f);

    right_target_speed = controller->target_linear_speed +
                        (controller->target_angular_speed * controller->config.wheel_distance / 2.0f);

    /* 读取编码器并转换为轮速 */
    left_encoder_counts = controller->get_encoder(controller->config.left_encoder_id, 1);
    right_encoder_counts = controller->get_encoder(controller->config.right_encoder_id, 1);

    left_current_speed = BreezeDifferentialDrive_EncoderToSpeed(controller, left_encoder_counts);
    right_current_speed = BreezeDifferentialDrive_EncoderToSpeed(controller, right_encoder_counts);

    /* 设置PID目标值 */
    BreezePIDController_SetSetpoint(&controller->left_pid, left_target_speed);
    BreezePIDController_SetSetpoint(&controller->right_pid, right_target_speed);

    /* 计算PID输出 */
    left_output = BreezePIDController_Compute(&controller->left_pid, left_current_speed);
    right_output = BreezePIDController_Compute(&controller->right_pid, right_current_speed);

    /* 设置电机速度 */
    controller->set_motor(controller->config.left_motor_id, left_output);
    controller->set_motor(controller->config.right_motor_id, right_output);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_DIFFERENTIAL_DRIVE_H */
