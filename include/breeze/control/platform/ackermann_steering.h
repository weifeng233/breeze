/**
 * @file ackermann_steering.h
 * @brief 阿克曼转向平台控制器
 *
 * 该实现提供了阿克曼转向平台的控制算法（类似汽车转向），
 * 其中前轮以略微不同的角度转向，以确保所有车轮在转弯时沿着适当的弧线行驶。
 */

#ifndef BREEZE_ACKERMANN_STEERING_H
#define BREEZE_ACKERMANN_STEERING_H

#include "../pid_controller.h"
#include "mobile_platform_hal.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 阿克曼转向控制器配置
 */
typedef struct {
    float wheelbase;             /* 前后轴之间的距离（米） */
    float track_width;           /* 左右轮之间的距离（米） */
    float wheel_radius;          /* 轮子半径（米） */
    float max_speed;             /* 最大前进速度（米/秒） */
    float max_steering_angle;    /* 最大转向角度（弧度） */
    int drive_motor_id;          /* 驱动电机ID */
    int steering_motor_id;       /* 转向电机/舵机ID */
    int encoder_id;              /* 驱动编码器ID */
    float encoder_resolution;    /* 每轮旋转的编码器计数 */
    float steering_ratio;        /* 转向电机与实际转向角度之间的比率 */
} BreezeAckermannConfig;

/**
 * @brief 阿克曼转向控制器结构体
 */
typedef struct {
    BreezeAckermannConfig config;     /* 平台配置 */
    BreezePIDController speed_pid;     /* 速度PID控制器 */
    BreezePIDController steering_pid;  /* 转向PID控制器 */
    BreezeMotorControlFunc set_motor;  /* 电机控制函数 */
    BreezeEncoderFunc get_encoder;     /* 编码器读取函数 */
    float target_speed;               /* 目标前进速度（米/秒） */
    float target_steering_angle;      /* 目标转向角度（弧度） */
    float dt;                         /* 控制循环时间步长（秒） */
} BreezeAckermannSteering;

/**
 * @brief 初始化阿克曼转向控制器
 *
 * @param controller 指向控制器结构体的指针
 * @param config 平台配置
 * @param set_motor 电机控制函数
 * @param get_encoder 编码器读取函数
 * @param dt 控制循环时间步长（秒）
 */
static inline void BreezeAckermannSteering_Init(
    BreezeAckermannSteering* controller,
    BreezeAckermannConfig config,
    BreezeMotorControlFunc set_motor,
    BreezeEncoderFunc get_encoder,
    float dt
) {
    if (!controller || !set_motor || !get_encoder) return;

    controller->config = config;
    controller->set_motor = set_motor;
    controller->get_encoder = get_encoder;
    controller->target_speed = 0.0f;
    controller->target_steering_angle = 0.0f;
    controller->dt = dt;

    /* 初始化PID控制器 */
    BreezePIDController_Init(&controller->speed_pid, BREEZE_PID_POSITION,
                            1.0f, 0.1f, 0.05f, dt, -1.0f, 1.0f);

    BreezePIDController_Init(&controller->steering_pid, BREEZE_PID_POSITION,
                            1.0f, 0.1f, 0.05f, dt, -1.0f, 1.0f);
}

/**
 * @brief 设置速度控制的PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static inline void BreezeAckermannSteering_SetSpeedPIDParams(
    BreezeAckermannSteering* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->speed_pid.kp = kp;
    controller->speed_pid.ki = ki;
    controller->speed_pid.kd = kd;
}

/**
 * @brief 设置转向控制的PID控制器参数
 *
 * @param controller 指向控制器结构体的指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
static inline void BreezeAckermannSteering_SetSteeringPIDParams(
    BreezeAckermannSteering* controller,
    float kp, float ki, float kd
) {
    if (!controller) return;

    controller->steering_pid.kp = kp;
    controller->steering_pid.ki = ki;
    controller->steering_pid.kd = kd;
}

/**
 * @brief 设置目标速度和转向角度
 *
 * @param controller 指向控制器结构体的指针
 * @param speed 目标前进速度（米/秒）
 * @param steering_angle 目标转向角度（弧度，正值=左转）
 */
static inline void BreezeAckermannSteering_SetTargets(
    BreezeAckermannSteering* controller,
    float speed,
    float steering_angle
) {
    if (!controller) return;

    /* 将速度和转向角度限制在配置的最大值内 */
    if (speed > controller->config.max_speed) {
        speed = controller->config.max_speed;
    } else if (speed < -controller->config.max_speed) {
        speed = -controller->config.max_speed;
    }

    if (steering_angle > controller->config.max_steering_angle) {
        steering_angle = controller->config.max_steering_angle;
    } else if (steering_angle < -controller->config.max_steering_angle) {
        steering_angle = -controller->config.max_steering_angle;
    }

    controller->target_speed = speed;
    controller->target_steering_angle = steering_angle;
}

/**
 * @brief 将编码器计数转换为轮速
 *
 * @param controller 指向控制器结构体的指针
 * @param encoder_counts 自上次更新以来的编码器计数
 * @return 轮速（米/秒）
 */
static inline float BreezeAckermannSteering_EncoderToSpeed(
    BreezeAckermannSteering* controller,
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
 * @brief 基于阿克曼几何计算内外轮转向角度
 *
 * @param controller 指向控制器结构体的指针
 * @param steering_angle 中心转向角度
 * @param inner_angle 用于存储内轮角度的指针
 * @param outer_angle 用于存储外轮角度的指针
 */
static inline void BreezeAckermannSteering_CalculateWheelAngles(
    BreezeAckermannSteering* controller,
    float steering_angle,
    float* inner_angle,
    float* outer_angle
) {
    float turning_radius;
    float inner_radius;
    float outer_radius;

    if (!controller || !inner_angle || !outer_angle) return;

    /* 对于小角度，简化处理，两个轮子使用相同的角度 */
    if (fabsf(steering_angle) < 0.01f) {
        *inner_angle = steering_angle;
        *outer_angle = steering_angle;
        return;
    }

    /* 根据中心转向角度计算转弯半径 */
    turning_radius = controller->config.wheelbase / tanf(steering_angle);

    /* 计算内外转弯半径 */
    if (steering_angle > 0) {
        /* 左转 */
        inner_radius = turning_radius - (controller->config.track_width / 2.0f);
        outer_radius = turning_radius + (controller->config.track_width / 2.0f);
    } else {
        /* 右转 */
        inner_radius = turning_radius + (controller->config.track_width / 2.0f);
        outer_radius = turning_radius - (controller->config.track_width / 2.0f);
    }

    /* 计算内外轮的转向角度 */
    *inner_angle = atanf(controller->config.wheelbase / inner_radius);
    *outer_angle = atanf(controller->config.wheelbase / outer_radius);

    /* 保持原始转向角度的符号 */
    if (steering_angle < 0) {
        *inner_angle = -*inner_angle;
        *outer_angle = -*outer_angle;
    }
}

/**
 * @brief 更新阿克曼转向控制器
 *
 * 该函数应以dt指定的速率定期调用。
 * 它读取编码器，使用PID控制器计算控制输出，
 * 并设置电机速度。
 *
 * @param controller 指向控制器结构体的指针
 */
static inline void BreezeAckermannSteering_Update(BreezeAckermannSteering* controller) {
    float current_speed;
    float encoder_counts;
    float speed_output, steering_output;
    float inner_angle, outer_angle;

    if (!controller || !controller->set_motor || !controller->get_encoder) return;

    /* 读取编码器并转换为速度 */
    encoder_counts = controller->get_encoder(controller->config.encoder_id, 1);
    current_speed = BreezeAckermannSteering_EncoderToSpeed(controller, encoder_counts);

    /* 设置PID目标值 */
    BreezePIDController_SetSetpoint(&controller->speed_pid, controller->target_speed);
    BreezePIDController_SetSetpoint(&controller->steering_pid, controller->target_steering_angle);

    /* 计算PID输出 */
    speed_output = BreezePIDController_Compute(&controller->speed_pid, current_speed);

    /* 对于转向，我们假设在实际系统中会使用位置反馈（例如电位器） */
    /* 这里我们通过以前馈方式使用PID控制器来简化 */
    steering_output = controller->target_steering_angle / controller->config.max_steering_angle;

    /* 计算阿克曼转向角度（仅供参考，在这个简化的控制中未使用） */
    BreezeAckermannSteering_CalculateWheelAngles(
        controller,
        controller->target_steering_angle,
        &inner_angle,
        &outer_angle
    );

    /* 设置电机输出 */
    controller->set_motor(controller->config.drive_motor_id, speed_output);
    controller->set_motor(controller->config.steering_motor_id, steering_output);
}

#ifdef __cplusplus
}
#endif

#endif /* BREEZE_ACKERMANN_STEERING_H */
