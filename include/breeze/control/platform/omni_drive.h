/**
 * @file omni_drive.h
 * @brief 使用全向轮的全向平台控制器
 *
 * 该实现提供了使用3个或4个以不同配置排列的全向轮的
 * 全向平台控制算法。
 */

#ifndef BREEZE_OMNI_DRIVE_H
#define BREEZE_OMNI_DRIVE_H

#include "../pid_controller.h"
#include "mobile_platform_hal.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 全向轮驱动配置类型
 */
typedef enum {
    BREEZE_OMNI_THREE_WHEEL_120DEG, /* 3个轮子相隔120度 */
    BREEZE_OMNI_FOUR_WHEEL_90DEG    /* 4个轮子相隔90度（X型配置） */
} BreezeOmniDriveType;

/**
 * @brief 全向轮驱动控制器配置
 */
typedef struct {
    BreezeOmniDriveType type;     /* 平台配置类型 */
    float wheel_radius;           /* 轮子半径（米） */
    float wheel_distance;         /* 从中心到轮子的距离（米） */
    float max_linear_speed;       /* 最大线速度（米/秒） */
    float max_angular_speed;      /* 最大角速度（弧度/秒） */
    int num_wheels;               /* 轮子数量（3或4） */
    int motor_ids[4];             /* 每个轮子的电机ID */
    int encoder_ids[4];           /* 每个轮子的编码器ID */
    float wheel_angles[4];        /* 轮子角度（弧度，从x轴开始） */
    float encoder_resolution;     /* 每轮旋转的编码器计数 */
} BreezeOmniDriveConfig;

/**
 * @brief 全向轮驱动控制器结构体
 */
typedef struct {
    BreezeOmniDriveConfig config;   /* 平台配置 */
    BreezePIDController wheel_pid[4]; /* 每个轮子的PID控制器 */
    BreezeMotorControlFunc set_motor; /* 电机控制函数 */
    BreezeEncoderFunc get_encoder;    /* 编码器读取函数 */
    float target_vx;                 /* 目标X方向速度（米/秒，前进为正） */
    float target_vy;                 /* 目标Y方向速度（米/秒，左侧为正） */
    float target_omega;              /* 目标角速度（弧度/秒，逆时针为正） */
    float dt;                        /* 控制循环时间步长（秒） */
} BreezeOmniDrive;

/**
 * @brief 初始化全向轮驱动控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param config_type 平台配置类型
 * @param wheel_radius 轮子半径（米）
 * @param wheel_distance 从中心到轮子的距离（米）
 * @param max_linear_speed 最大线速度（米/秒）
 * @param max_angular_speed 最大角速度（弧度/秒）
 * @param motor_ids 电机ID数组
 * @param encoder_ids 编码器ID数组
 * @param encoder_resolution 每轮旋转的编码器计数
 * @param set_motor 电机控制函数
 * @param get_encoder 编码器读取函数
 * @param dt 控制循环时间步长（秒）
 */
static void BreezeOmniDrive_Init(
    BreezeOmniDrive* controller,
    BreezeOmniDriveType config_type,
    float wheel_radius,
    float wheel_distance,
    float max_linear_speed,
    float max_angular_speed,
    int* motor_ids,
    int* encoder_ids,
    float encoder_resolution,
    BreezeMotorControlFunc set_motor,
    BreezeEncoderFunc get_encoder,
    float dt
) {
    int i;

    if (!controller || !motor_ids || !encoder_ids || !set_motor || !get_encoder) return;

    controller->config.type = config_type;
    controller->config.wheel_radius = wheel_radius;
    controller->config.wheel_distance = wheel_distance;
    controller->config.max_linear_speed = max_linear_speed;
    controller->config.max_angular_speed = max_angular_speed;
    controller->config.encoder_resolution = encoder_resolution;

    controller->set_motor = set_motor;
    controller->get_encoder = get_encoder;
    controller->target_vx = 0.0f;
    controller->target_vy = 0.0f;
    controller->target_omega = 0.0f;
    controller->dt = dt;

    /* 根据平台类型进行配置 */
    if (config_type == BREEZE_OMNI_THREE_WHEEL_120DEG) {
        controller->config.num_wheels = 3;

        /* 设置轮子角度（从x轴开始的弧度） */
        controller->config.wheel_angles[0] = 0.0f;               /* 轮子1在0度 */
        controller->config.wheel_angles[1] = 2.0f * 3.14159f / 3.0f; /* 轮子2在120度 */
        controller->config.wheel_angles[2] = 4.0f * 3.14159f / 3.0f; /* 轮子3在240度 */
    } else { /* BREEZE_OMNI_FOUR_WHEEL_90DEG */
        controller->config.num_wheels = 4;

        /* 设置轮子角度（从x轴开始的弧度） */
        controller->config.wheel_angles[0] = 3.14159f / 4.0f;     /* 轮子1在45度 */
        controller->config.wheel_angles[1] = 3.0f * 3.14159f / 4.0f; /* 轮子2在135度 */
        controller->config.wheel_angles[2] = 5.0f * 3.14159f / 4.0f; /* 轮子3在225度 */
        controller->config.wheel_angles[3] = 7.0f * 3.14159f / 4.0f; /* 轮子4在315度 */
    }

    /* 复制电机和编码器ID */
    for (i = 0; i < controller->config.num_wheels; i++) {
        controller->config.motor_ids[i] = motor_ids[i];
        controller->config.encoder_ids[i] = encoder_ids[i];
    }

    /* 初始化所有轮子的PID控制器 */
    for (i = 0; i < controller->config.num_wheels; i++) {
        BreezePIDController_Init(&controller->wheel_pid[i], BREEZE_PID_POSITION,
                                1.0f, 0.1f, 0.05f, dt, -1.0f, 1.0f);
    }
}

/**
 * @brief 设置所有轮子的PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static void BreezeOmniDrive_SetPIDParams(
    BreezeOmniDrive* controller,
    float kp, float ki, float kd
) {
    int i;

    if (!controller) return;

    for (i = 0; i < controller->config.num_wheels; i++) {
        controller->wheel_pid[i].kp = kp;
        controller->wheel_pid[i].ki = ki;
        controller->wheel_pid[i].kd = kd;
    }
}

/**
 * @brief 设置全向轮驱动平台的目标速度
 *
 * @param controller 指向控制器结构体的指针
 * @param vx 目标X方向速度（米/秒，前进为正）
 * @param vy 目标Y方向速度（米/秒，左侧为正）
 * @param omega 目标角速度（弧度/秒，逆时针为正）
 */
static void BreezeOmniDrive_SetVelocity(
    BreezeOmniDrive* controller,
    float vx, float vy, float omega
) {
    if (!controller) return;

    /* 将线速度限制在配置的最大值内 */
    float linear_speed = sqrtf(vx*vx + vy*vy);
    if (linear_speed > controller->config.max_linear_speed && linear_speed > 0) {
        float scale = controller->config.max_linear_speed / linear_speed;
        vx *= scale;
        vy *= scale;
    }

    /* 将角速度限制在配置的最大值内 */
    if (omega > controller->config.max_angular_speed) {
        omega = controller->config.max_angular_speed;
    } else if (omega < -controller->config.max_angular_speed) {
        omega = -controller->config.max_angular_speed;
    }

    controller->target_vx = vx;
    controller->target_vy = vy;
    controller->target_omega = omega;
}

/**
 * @brief 将编码器计数转换为轮速
 *
 * @param controller 指向控制器结构体的指针
 * @param encoder_counts 自上次更新以来的编码器计数
 * @return 轮速（米/秒）
 */
static float BreezeOmniDrive_EncoderToSpeed(
    BreezeOmniDrive* controller,
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
 * @brief 更新全向轮驱动控制器
 *
 * 该函数应以dt指定的速率定期调用。
 * 它读取编码器，使用PID控制器计算控制输出，
 * 并设置电机速度。
 *
 * @param controller 指向控制器结构体的指针
 */
static void BreezeOmniDrive_Update(BreezeOmniDrive* controller) {
    float wheel_target_speeds[4];
    float wheel_current_speeds[4];
    float wheel_outputs[4];
    float encoder_counts;
    int i;

    if (!controller || !controller->set_motor || !controller->get_encoder) return;

    /* 使用逆运动学计算目标轮速 */
    for (i = 0; i < controller->config.num_wheels; i++) {
        float angle = controller->config.wheel_angles[i];

        /* 将机器人速度投影到轮子方向 */
        wheel_target_speeds[i] = controller->target_vx * cosf(angle) +
                                controller->target_vy * sinf(angle);

        /* 添加旋转分量 */
        wheel_target_speeds[i] += controller->config.wheel_distance * controller->target_omega;
    }

    /* 如果任何轮速超过最大值，则归一化轮速 */
    float max_speed = 0.0f;
    for (i = 0; i < controller->config.num_wheels; i++) {
        float abs_speed = fabsf(wheel_target_speeds[i]);
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }

    if (max_speed > controller->config.max_linear_speed && max_speed > 0) {
        float scale = controller->config.max_linear_speed / max_speed;
        for (i = 0; i < controller->config.num_wheels; i++) {
            wheel_target_speeds[i] *= scale;
        }
    }

    /* 读取编码器并转换为轮速 */
    for (i = 0; i < controller->config.num_wheels; i++) {
        encoder_counts = controller->get_encoder(controller->config.encoder_ids[i], 1);
        wheel_current_speeds[i] = BreezeOmniDrive_EncoderToSpeed(controller, encoder_counts);
    }

    /* 计算每个轮子的PID输出 */
    for (i = 0; i < controller->config.num_wheels; i++) {
        BreezePIDController_SetSetpoint(&controller->wheel_pid[i], wheel_target_speeds[i]);
        wheel_outputs[i] = BreezePIDController_Compute(&controller->wheel_pid[i], wheel_current_speeds[i]);

        /* 将输出归一化到-1.0到1.0范围 */
        if (wheel_outputs[i] > 1.0f) wheel_outputs[i] = 1.0f;
        if (wheel_outputs[i] < -1.0f) wheel_outputs[i] = -1.0f;

        /* 设置电机速度 */
        controller->set_motor(controller->config.motor_ids[i], wheel_outputs[i]);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_OMNI_DRIVE_H */
