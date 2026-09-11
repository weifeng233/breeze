/**
 * @file mecanum_drive.h
 * @brief 麦克纳姆轮全向平台控制器
 *
 * 该实现提供了麦克纳姆轮平台的控制算法，
 * 允许全向运动，包括横向移动和旋转。
 */

#ifndef BREEZE_MECANUM_DRIVE_H
#define BREEZE_MECANUM_DRIVE_H

#include "../pid_controller.h"
#include "mobile_platform_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 麦克纳姆轮位置
 */
typedef enum {
    BREEZE_MECANUM_FRONT_LEFT = 0,
    BREEZE_MECANUM_FRONT_RIGHT = 1,
    BREEZE_MECANUM_REAR_LEFT = 2,
    BREEZE_MECANUM_REAR_RIGHT = 3
} BreezeMecanumWheelPosition;

/**
 * @brief 麦克纳姆轮驱动控制器配置
 */
typedef struct {
    float wheel_radius;          /* 轮子半径（米） */
    float wheel_distance_x;      /* 前后轮之间的距离（米） */
    float wheel_distance_y;      /* 左右轮之间的距离（米） */
    float max_linear_speed;      /* 最大线速度（米/秒） */
    float max_angular_speed;     /* 最大角速度（弧度/秒） */
    int motor_ids[4];            /* 每个轮子位置的电机ID */
    int encoder_ids[4];          /* 每个轮子位置的编码器ID */
    float encoder_resolution;    /* 每轮旋转的编码器计数 */
} BreezeMecanumDriveConfig;

/**
 * @brief 麦克纳姆轮驱动控制器结构体
 */
typedef struct {
    BreezeMecanumDriveConfig config;  /* 平台配置 */
    BreezePIDController wheel_pid[4];  /* 每个轮子的PID控制器 */
    BreezeMotorControlFunc set_motor;  /* 电机控制函数 */
    BreezeEncoderFunc get_encoder;     /* 编码器读取函数 */
    float target_vx;                  /* 目标X方向速度（米/秒，前进为正） */
    float target_vy;                  /* 目标Y方向速度（米/秒，左侧为正） */
    float target_omega;               /* 目标角速度（弧度/秒，逆时针为正） */
    float dt;                         /* 控制循环时间步长（秒） */
} BreezeMecanumDrive;

/**
 * @brief 初始化麦克纳姆轮驱动控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param config 平台配置
 * @param set_motor 电机控制函数
 * @param get_encoder 编码器读取函数
 * @param dt 控制循环时间步长（秒）
 */
static void BreezeMecanumDrive_Init(
    BreezeMecanumDrive* controller,
    BreezeMecanumDriveConfig config,
    BreezeMotorControlFunc set_motor,
    BreezeEncoderFunc get_encoder,
    float dt
) {
    int i;

    if (!controller || !set_motor || !get_encoder) return;

    controller->config = config;
    controller->set_motor = set_motor;
    controller->get_encoder = get_encoder;
    controller->target_vx = 0.0f;
    controller->target_vy = 0.0f;
    controller->target_omega = 0.0f;
    controller->dt = dt;

    /* 初始化所有轮子的PID控制器 */
    for (i = 0; i < 4; i++) {
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
static void BreezeMecanumDrive_SetPIDParams(
    BreezeMecanumDrive* controller,
    float kp, float ki, float kd
) {
    int i;

    if (!controller) return;

    for (i = 0; i < 4; i++) {
        controller->wheel_pid[i].kp = kp;
        controller->wheel_pid[i].ki = ki;
        controller->wheel_pid[i].kd = kd;
    }
}

/**
 * @brief 设置麦克纳姆轮驱动平台的目标速度
 *
 * @param controller 指向控制器结构体的指针
 * @param vx 目标X方向速度（米/秒，前进为正）
 * @param vy 目标Y方向速度（米/秒，左侧为正）
 * @param omega 目标角速度（弧度/秒，逆时针为正）
 */
static void BreezeMecanumDrive_SetVelocity(
    BreezeMecanumDrive* controller,
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
static float BreezeMecanumDrive_EncoderToSpeed(
    BreezeMecanumDrive* controller,
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
 * @brief 更新麦克纳姆轮驱动控制器
 *
 * 该函数应以dt指定的速率定期调用。
 * 它读取编码器，使用PID控制器计算控制输出，
 * 并设置电机速度。
 *
 * @param controller 指向控制器结构体的指针
 */
static void BreezeMecanumDrive_Update(BreezeMecanumDrive* controller) {
    float wheel_target_speeds[4];
    float wheel_current_speeds[4];
    float wheel_outputs[4];
    float encoder_counts;
    int i;
    float l_x, l_y; /* 轮子之间距离的一半 */

    if (!controller || !controller->set_motor || !controller->get_encoder) return;

    l_x = controller->config.wheel_distance_x / 2.0f;
    l_y = controller->config.wheel_distance_y / 2.0f;

    /* 使用逆运动学计算目标轮速 */
    /* 左前轮 */
    wheel_target_speeds[BREEZE_MECANUM_FRONT_LEFT] =
        controller->target_vx - controller->target_vy - (l_x + l_y) * controller->target_omega;

    /* 右前轮 */
    wheel_target_speeds[BREEZE_MECANUM_FRONT_RIGHT] =
        controller->target_vx + controller->target_vy + (l_x + l_y) * controller->target_omega;

    /* 左后轮 */
    wheel_target_speeds[BREEZE_MECANUM_REAR_LEFT] =
        controller->target_vx + controller->target_vy - (l_x + l_y) * controller->target_omega;

    /* 右后轮 */
    wheel_target_speeds[BREEZE_MECANUM_REAR_RIGHT] =
        controller->target_vx - controller->target_vy + (l_x + l_y) * controller->target_omega;

    /* 如果任何轮速超过最大值，则归一化轮速 */
    float max_speed = 0.0f;
    for (i = 0; i < 4; i++) {
        float abs_speed = fabsf(wheel_target_speeds[i]);
        if (abs_speed > max_speed) {
            max_speed = abs_speed;
        }
    }

    if (max_speed > controller->config.max_linear_speed && max_speed > 0) {
        float scale = controller->config.max_linear_speed / max_speed;
        for (i = 0; i < 4; i++) {
            wheel_target_speeds[i] *= scale;
        }
    }

    /* 读取编码器并转换为轮速 */
    for (i = 0; i < 4; i++) {
        encoder_counts = controller->get_encoder(controller->config.encoder_ids[i], 1);
        wheel_current_speeds[i] = BreezeMecanumDrive_EncoderToSpeed(controller, encoder_counts);
    }

    /* 计算每个轮子的PID输出 */
    for (i = 0; i < 4; i++) {
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

#endif /* BREEZE_MECANUM_DRIVE_H */
