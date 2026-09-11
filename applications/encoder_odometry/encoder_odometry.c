/**
 * @file encoder_odometry.c
 * @brief 基于编码器的里程计应用，实现测速和测距功能
 * 
 * 该应用使用Breeze框架实现一个简单的里程计，通过编码器数据
 * 计算小车的速度和行驶距离。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../include/breeze/breeze.h"

/* 小车状态结构体 */
typedef struct {
    float x;          /* X坐标（米） */
    float y;          /* Y坐标（米） */
    float theta;      /* 方向角（弧度） */
    float v;          /* 线速度（米/秒） */
    float omega;      /* 角速度（弧度/秒） */
    float left_dist;  /* 左轮累计行驶距离（米） */
    float right_dist; /* 右轮累计行驶距离（米） */
    float total_dist; /* 总行驶距离（米） */
} CarState;

/* 里程计应用结构体 */
typedef struct {
    BreezeDifferentialDrive controller;  /* 差速驱动控制器 */
    CarState state;                      /* 小车当前状态 */
    float start_time;                    /* 开始时间 */
    float current_time;                  /* 当前时间 */
} EncoderOdometry;

/* 硬件抽象层函数 */
void set_motor(int motor_id, float speed);
float get_encoder(int encoder_id, int reset);
float get_time(void);

/* 应用功能函数 */
void odometry_init(EncoderOdometry* odom);
void odometry_update(EncoderOdometry* odom, float dt);
void odometry_reset(EncoderOdometry* odom);
void odometry_print_state(EncoderOdometry* odom);

/* 主函数 */
int main(void) {
    EncoderOdometry odom;
    float prev_time, current_time, dt;
    char command;
    
    printf("基于编码器的里程计应用\n");
    printf("=================================\n");
    
    /* 初始化里程计 */
    odometry_init(&odom);
    
    /* 主循环 */
    prev_time = get_time();
    
    while (1) {
        /* 计算时间步长 */
        current_time = get_time();
        dt = current_time - prev_time;
        prev_time = current_time;
        
        /* 更新里程计状态 */
        odometry_update(&odom, dt);
        
        /* 打印里程计状态 */
        odometry_print_state(&odom);
        
        /* 处理用户命令 */
        printf("\n命令: [w]前进 [s]后退 [a]左转 [d]右转 [空格]停止 [r]重置里程计 [q]退出\n");
        scanf(" %c", &command);
        
        switch (command) {
            case 'w': /* 前进 */
                BreezeDifferentialDrive_SetSpeed(&odom.controller, 0.5f, 0.0f);
                break;
                
            case 's': /* 后退 */
                BreezeDifferentialDrive_SetSpeed(&odom.controller, -0.5f, 0.0f);
                break;
                
            case 'a': /* 左转 */
                BreezeDifferentialDrive_SetSpeed(&odom.controller, 0.3f, 1.0f);
                break;
                
            case 'd': /* 右转 */
                BreezeDifferentialDrive_SetSpeed(&odom.controller, 0.3f, -1.0f);
                break;
                
            case ' ': /* 停止 */
                BreezeDifferentialDrive_SetSpeed(&odom.controller, 0.0f, 0.0f);
                break;
                
            case 'r': /* 重置里程计 */
                odometry_reset(&odom);
                break;
                
            case 'q': /* 退出 */
                return 0;
                
            default:
                break;
        }
    }
    
    return 0;
}

/* 硬件抽象层函数实现 */

/* 设置电机速度 */
void set_motor(int motor_id, float speed) {
    printf("电机 %d 设置速度: %.2f\n", motor_id, speed);
    /* 在实际应用中，这里应该调用硬件驱动函数控制电机 */
}

/* 获取编码器计数 */
float get_encoder(int encoder_id, int reset) {
    static float counts[2] = {0};
    static float speeds[2] = {0};
    float result;
    
    /* 模拟编码器计数，根据当前设定的速度增加计数 */
    counts[encoder_id] += speeds[encoder_id];
    result = counts[encoder_id];
    
    if (reset) {
        counts[encoder_id] = 0;
    }
    
    return result;
}

/* 获取当前时间（秒） */
float get_time(void) {
    static float time = 0.0f;
    
    /* 模拟时间流逝，每次调用增加0.1秒 */
    time += 0.1f;
    
    return time;
}

/* 应用功能函数实现 */

/* 初始化里程计 */
void odometry_init(EncoderOdometry* odom) {
    BreezeDifferentialDriveConfig config;
    
    if (!odom) return;
    
    /* 清空结构体 */
    memset(odom, 0, sizeof(EncoderOdometry));
    
    /* 配置差速驱动 */
    config.wheel_distance = 0.2f;       /* 轮子之间距离20厘米 */
    config.wheel_radius = 0.05f;        /* 轮子半径5厘米 */
    config.max_linear_speed = 1.0f;     /* 最大线速度1米/秒 */
    config.max_angular_speed = 2.0f;    /* 最大角速度2弧度/秒 */
    config.left_motor_id = 0;
    config.right_motor_id = 1;
    config.left_encoder_id = 0;
    config.right_encoder_id = 1;
    config.encoder_resolution = 360.0f; /* 每转360个计数 */
    
    /* 初始化控制器 */
    BreezeDifferentialDrive_Init(&odom->controller, config, set_motor, get_encoder, 0.1f);
    
    /* 设置PID参数 */
    BreezeDifferentialDrive_SetPIDParams(&odom->controller, 1.0f, 0.1f, 0.05f);
    
    /* 记录开始时间 */
    odom->start_time = get_time();
    odom->current_time = odom->start_time;
    
    printf("里程计初始化完成\n");
}

/* 更新里程计状态 */
void odometry_update(EncoderOdometry* odom, float dt) {
    float left_encoder, right_encoder;
    float left_dist, right_dist;
    float delta_left, delta_right;
    float delta_dist, delta_theta;
    
    if (!odom) return;
    
    /* 更新控制器 */
    BreezeDifferentialDrive_Update(&odom->controller);
    
    /* 获取编码器数据（不重置） */
    left_encoder = odom->controller.get_encoder(odom->controller.config.left_encoder_id, 0);
    right_encoder = odom->controller.get_encoder(odom->controller.config.right_encoder_id, 0);
    
    /* 计算轮子行驶距离 */
    left_dist = left_encoder / odom->controller.config.encoder_resolution * 
                (2.0f * M_PI * odom->controller.config.wheel_radius);
    right_dist = right_encoder / odom->controller.config.encoder_resolution * 
                 (2.0f * M_PI * odom->controller.config.wheel_radius);
    
    /* 计算距离增量 */
    delta_left = left_dist - odom->state.left_dist;
    delta_right = right_dist - odom->state.right_dist;
    
    /* 更新累计距离 */
    odom->state.left_dist = left_dist;
    odom->state.right_dist = right_dist;
    
    /* 计算小车位移和转向 */
    delta_dist = (delta_left + delta_right) / 2.0f;
    delta_theta = (delta_right - delta_left) / odom->controller.config.wheel_distance;
    
    /* 更新小车位置和方向 */
    odom->state.theta += delta_theta;
    odom->state.x += delta_dist * cosf(odom->state.theta);
    odom->state.y += delta_dist * sinf(odom->state.theta);
    
    /* 更新总行驶距离 */
    odom->state.total_dist += fabsf(delta_dist);
    
    /* 计算当前速度 */
    odom->state.v = (delta_left + delta_right) / (2.0f * dt);
    odom->state.omega = (delta_right - delta_left) / (odom->controller.config.wheel_distance * dt);
    
    /* 更新时间 */
    odom->current_time = get_time();
}

/* 重置里程计 */
void odometry_reset(EncoderOdometry* odom) {
    if (!odom) return;
    
    /* 重置位置和方向 */
    odom->state.x = 0.0f;
    odom->state.y = 0.0f;
    odom->state.theta = 0.0f;
    odom->state.total_dist = 0.0f;
    
    /* 记录当前时间为新的开始时间 */
    odom->start_time = odom->current_time;
    
    printf("里程计已重置\n");
}

/* 打印里程计状态 */
void odometry_print_state(EncoderOdometry* odom) {
    float run_time;
    
    if (!odom) return;
    
    run_time = odom->current_time - odom->start_time;
    
    printf("\n里程计状态:\n");
    printf("位置: (%.2f, %.2f) 方向: %.2f°\n", 
           odom->state.x, odom->state.y, odom->state.theta * 180.0f / M_PI);
    printf("速度: %.2f m/s 角速度: %.2f rad/s\n", 
           odom->state.v, odom->state.omega);
    printf("左轮距离: %.2f m 右轮距离: %.2f m\n", 
           odom->state.left_dist, odom->state.right_dist);
    printf("总行驶距离: %.2f m 运行时间: %.1f s\n", 
           odom->state.total_dist, run_time);
    printf("平均速度: %.2f m/s\n", 
           run_time > 0.0f ? odom->state.total_dist / run_time : 0.0f);
}
