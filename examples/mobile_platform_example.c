/**
 * @file mobile_platform_example.c
 * @brief 移动平台控制算法的使用示例
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../include/breeze/breeze.h"

/* 模拟电机控制函数 */
void set_motor(int motor_id, float speed) {
    printf("电机 %d 设置速度: %.2f\n", motor_id, speed);
}

/* 模拟编码器读取函数 */
float get_encoder(int encoder_id, int reset) {
    /* 根据电机ID模拟编码器计数 */
    static float counts[8] = {0};
    float result = counts[encoder_id];

    /* 模拟一些运动 */
    counts[encoder_id] += 10.0f * (1.0f + encoder_id * 0.1f);

    if (reset) {
        counts[encoder_id] = 0;
    }

    return result;
}

/* 模拟IMU数据函数 */
int get_imu_data(BreezeIMUData* imu_data) {
    if (!imu_data) return 0;

    /* 模拟IMU数据 */
    static float angle = 0.0f;

    /* 模拟围绕垂直方向的小振荡 */
    angle = 0.05f * sinf((float)rand() / RAND_MAX);

    imu_data->gyro_x = 0.01f;
    imu_data->gyro_y = 0.02f;
    imu_data->gyro_z = 0.01f;

    /* 模拟几乎垂直的机器人的加速度计数据 */
    imu_data->accel_x = sinf(angle);
    imu_data->accel_y = 0.0f;
    imu_data->accel_z = cosf(angle);

    return 1;
}

/* 差速驱动控制器示例函数 */
void differential_drive_example(void) {
    BreezeDifferentialDrive controller;
    BreezeDifferentialDriveConfig config;
    int i;

    printf("\n差速驱动控制器示例\n");
    printf("------------------------------------\n");

    /* 配置差速驱动 */
    config.wheel_distance = 0.3f;      /* 轮子之间距离30厘米 */
    config.wheel_radius = 0.05f;       /* 轮子半径5厘米 */
    config.max_linear_speed = 1.0f;    /* 最大线速度1米/秒 */
    config.max_angular_speed = 2.0f;   /* 最大角速度2弧度/秒 */
    config.left_motor_id = 0;
    config.right_motor_id = 1;
    config.left_encoder_id = 0;
    config.right_encoder_id = 1;
    config.encoder_resolution = 360.0f; /* 每转360个计数 */

    /* 初始化控制器 */
    BreezeDifferentialDrive_Init(&controller, config, set_motor, get_encoder, 0.1f);

    /* 设置PID参数 */
    BreezeDifferentialDrive_SetPIDParams(&controller, 1.0f, 0.1f, 0.05f);

    /* 模拟前进 */
    printf("\n以0.5米/秒的速度前进:\n");
    BreezeDifferentialDrive_SetSpeed(&controller, 0.5f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeDifferentialDrive_Update(&controller);
    }

    /* 模拟转弯 */
    printf("\n以0.3米/秒的速度前进并以1.0弧度/秒的速度转弯:\n");
    BreezeDifferentialDrive_SetSpeed(&controller, 0.3f, 1.0f);

    for (i = 0; i < 5; i++) {
        BreezeDifferentialDrive_Update(&controller);
    }
}

/* 阿克曼转向控制器示例函数 */
void ackermann_steering_example(void) {
    BreezeAckermannSteering controller;
    BreezeAckermannConfig config;
    int i;

    printf("\n阿克曼转向控制器示例\n");
    printf("------------------------------------\n");

    /* 配置阿克曼转向 */
    config.wheelbase = 0.25f;          /* 轴距25厘米 */
    config.track_width = 0.2f;         /* 轮距20厘米 */
    config.wheel_radius = 0.05f;       /* 轮子半径5厘米 */
    config.max_speed = 1.0f;           /* 最大速度1米/秒 */
    config.max_steering_angle = 0.5f;  /* 最大转向角约30度 */
    config.drive_motor_id = 2;
    config.steering_motor_id = 3;
    config.encoder_id = 2;
    config.encoder_resolution = 360.0f; /* 每转360个计数 */
    config.steering_ratio = 5.0f;       /* 5:1转向比 */

    /* 初始化控制器 */
    BreezeAckermannSteering_Init(&controller, config, set_motor, get_encoder, 0.1f);

    /* 设置PID参数 */
    BreezeAckermannSteering_SetSpeedPIDParams(&controller, 1.0f, 0.1f, 0.05f);
    BreezeAckermannSteering_SetSteeringPIDParams(&controller, 2.0f, 0.0f, 0.1f);

    /* 模拟直线行驶 */
    printf("\n以0.5米/秒的速度直线行驶:\n");
    BreezeAckermannSteering_SetTargets(&controller, 0.5f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeAckermannSteering_Update(&controller);
    }

    /* 模拟转弯 */
    printf("\n以0.3米/秒的速度行驶并以0.3弧度的转向角转弯:\n");
    BreezeAckermannSteering_SetTargets(&controller, 0.3f, 0.3f);

    for (i = 0; i < 5; i++) {
        BreezeAckermannSteering_Update(&controller);
    }
}

/* 麦克纳姆轮驱动控制器示例函数 */
void mecanum_drive_example(void) {
    BreezeMecanumDrive controller;
    BreezeMecanumDriveConfig config;
    int motor_ids[4] = {0, 1, 2, 3};
    int encoder_ids[4] = {0, 1, 2, 3};
    int i;

    printf("\n麦克纳姆轮驱动控制器示例\n");
    printf("-------------------------------\n");

    /* 配置麦克纳姆轮驱动 */
    config.wheel_radius = 0.05f;       /* 轮子半径5厘米 */
    config.wheel_distance_x = 0.3f;    /* 前后轮之间距离30厘米 */
    config.wheel_distance_y = 0.25f;   /* 左右轮之间距离25厘米 */
    config.max_linear_speed = 1.0f;    /* 最大线速度1米/秒 */
    config.max_angular_speed = 2.0f;   /* 最大角速度2弧度/秒 */

    /* 设置电机和编码器ID */
    for (i = 0; i < 4; i++) {
        config.motor_ids[i] = motor_ids[i];
        config.encoder_ids[i] = encoder_ids[i];
    }

    config.encoder_resolution = 360.0f; /* 每转360个计数 */

    /* 初始化控制器 */
    BreezeMecanumDrive_Init(&controller, config, set_motor, get_encoder, 0.1f);

    /* 设置PID参数 */
    BreezeMecanumDrive_SetPIDParams(&controller, 1.0f, 0.1f, 0.05f);

    /* 模拟前进 */
    printf("\n以0.5米/秒的速度前进:\n");
    BreezeMecanumDrive_SetVelocity(&controller, 0.5f, 0.0f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeMecanumDrive_Update(&controller);
    }

    /* 模拟横向移动 */
    printf("\n以0.3米/秒的速度向左横移:\n");
    BreezeMecanumDrive_SetVelocity(&controller, 0.0f, 0.3f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeMecanumDrive_Update(&controller);
    }

    /* 模拟斜向移动并旋转 */
    printf("\n以0.3米/秒的速度向右前方斜行并同时旋转:\n");
    BreezeMecanumDrive_SetVelocity(&controller, 0.3f, -0.3f, 0.5f);

    for (i = 0; i < 5; i++) {
        BreezeMecanumDrive_Update(&controller);
    }
}

/* 全向轮驱动控制器示例函数 */
void omni_drive_example(void) {
    BreezeOmniDrive controller;
    int motor_ids[3] = {0, 1, 2};
    int encoder_ids[3] = {0, 1, 2};
    int i;

    printf("\n全向轮驱动控制器示例\n");
    printf("---------------------------------\n");

    /* 初始化控制器，使用3轮120度配置 */
    BreezeOmniDrive_Init(
        &controller,
        BREEZE_OMNI_THREE_WHEEL_120DEG,
        0.05f,                  /* 轮子半径5厘米 */
        0.15f,                  /* 从中心到轮子的距离15厘米 */
        1.0f,                   /* 最大速度1米/秒 */
        2.0f,                   /* 最大角速度2弧度/秒 */
        motor_ids,
        encoder_ids,
        360.0f,                 /* 每转360个计数 */
        set_motor,
        get_encoder,
        0.1f
    );

    /* 设置PID参数 */
    BreezeOmniDrive_SetPIDParams(&controller, 1.0f, 0.1f, 0.05f);

    /* 模拟前进 */
    printf("\n以0.5米/秒的速度前进:\n");
    BreezeOmniDrive_SetVelocity(&controller, 0.5f, 0.0f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeOmniDrive_Update(&controller);
    }

    /* 模拟横向移动 */
    printf("\n以0.3米/秒的速度向左横移:\n");
    BreezeOmniDrive_SetVelocity(&controller, 0.0f, 0.3f, 0.0f);

    for (i = 0; i < 5; i++) {
        BreezeOmniDrive_Update(&controller);
    }

    /* 模拟原地旋转 */
    printf("\n以1.0弧度/秒的速度原地旋转:\n");
    BreezeOmniDrive_SetVelocity(&controller, 0.0f, 0.0f, 1.0f);

    for (i = 0; i < 5; i++) {
        BreezeOmniDrive_Update(&controller);
    }
}

/* 平衡控制器示例函数 */
void balance_controller_example(void) {
    BreezeBalanceController controller;
    BreezeBalanceControllerConfig config;
    int i;

    printf("\n平衡控制器示例\n");
    printf("-------------------------\n");

    /* 配置平衡控制器 */
    config.wheel_radius = 0.05f;       /* 轮子半径5厘米 */
    config.wheel_distance = 0.2f;      /* 轮子之间距离20厘米 */
    config.max_tilt_angle = 0.5f;      /* 最大倾斜角约30度 */
    config.max_speed = 0.5f;           /* 最大速度0.5米/秒 */
    config.max_angular_speed = 1.0f;   /* 最大角速度1弧度/秒 */
    config.target_tilt_angle = 0.0f;   /* 目标为垂直（0弧度） */
    config.left_motor_id = 0;
    config.right_motor_id = 1;
    config.left_encoder_id = 0;
    config.right_encoder_id = 1;
    config.encoder_resolution = 360.0f; /* 每转360个计数 */

    /* 初始化控制器 */
    BreezeBalanceController_Init(&controller, config, set_motor, get_encoder, get_imu_data, 0.01f);

    /* 设置PID参数 */
    BreezeBalanceController_SetAnglePIDParams(&controller, 10.0f, 0.0f, 0.1f);
    BreezeBalanceController_SetSpeedPIDParams(&controller, 0.5f, 0.05f, 0.0f);
    BreezeBalanceController_SetTurnPIDParams(&controller, 1.0f, 0.0f, 0.0f);

    /* 模拟原地平衡 */
    printf("\n原地平衡:\n");
    BreezeBalanceController_SetTargets(&controller, 0.0f, 0.0f);

    for (i = 0; i < 5; i++) {
        if (BreezeBalanceController_Update(&controller)) {
            printf("平衡保持，迭代 %d\n", i+1);
        } else {
            printf("平衡丢失，迭代 %d\n", i+1);
        }
    }

    /* 模拟平衡前进 */
    printf("\n以0.2米/秒的速度平衡前进:\n");
    BreezeBalanceController_SetTargets(&controller, 0.2f, 0.0f);

    for (i = 0; i < 5; i++) {
        if (BreezeBalanceController_Update(&controller)) {
            printf("平衡保持，迭代 %d\n", i+1);
        } else {
            printf("平衡丢失，迭代 %d\n", i+1);
        }
    }

    /* 模拟平衡转弯 */
    printf("\n以0.5弧度/秒的速度平衡转弯:\n");
    BreezeBalanceController_SetTargets(&controller, 0.0f, 0.5f);

    for (i = 0; i < 5; i++) {
        if (BreezeBalanceController_Update(&controller)) {
            printf("平衡保持，迭代 %d\n", i+1);
        } else {
            printf("平衡丢失，迭代 %d\n", i+1);
        }
    }
}

/* 组合移动平台示例 */
void mobile_platform_examples(void) {
    printf("\n移动平台控制示例\n");
    printf("==============================\n");

    differential_drive_example();
    ackermann_steering_example();
    mecanum_drive_example();
    omni_drive_example();
    balance_controller_example();
}

int main(void) {
    printf("Breeze框架移动平台示例\n");
    printf("=======================================\n");

    mobile_platform_examples();

    return 0;
}
