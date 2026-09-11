/**
 * @file differential_drive_car.c
 * @brief 差速驱动小车应用，包含测速测距和赛道记录重放功能
 *
 * 该应用使用Breeze框架实现一个差速驱动小车，通过编码器数据完成测速和测距，
 * 并实现可以记录和重放赛道的功能。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../include/breeze/breeze.h"

/* 定义文件操作相关函数 */
#define MAX_PATH_POINTS 1000
#define PATH_FILE_NAME "track_data.txt"

/* 路径点结构体 */
typedef struct {
    float x;          /* X坐标（米） */
    float y;          /* Y坐标（米） */
    float theta;      /* 方向角（弧度） */
    float v;          /* 线速度（米/秒） */
    float omega;      /* 角速度（弧度/秒） */
    float timestamp;  /* 时间戳（秒） */
} PathPoint;

/* 小车状态结构体 */
typedef struct {
    float x;          /* X坐标（米） */
    float y;          /* Y坐标（米） */
    float theta;      /* 方向角（弧度） */
    float v;          /* 线速度（米/秒） */
    float omega;      /* 角速度（弧度/秒） */
    float left_dist;  /* 左轮累计行驶距离（米） */
    float right_dist; /* 右轮累计行驶距离（米） */
} CarState;

/* 小车应用结构体 */
typedef struct {
    BreezeDifferentialDrive controller;  /* 差速驱动控制器 */
    CarState state;                      /* 小车当前状态 */
    PathPoint path[MAX_PATH_POINTS];     /* 记录的路径点 */
    int path_count;                      /* 路径点数量 */
    int is_recording;                    /* 是否正在记录 */
    int is_replaying;                    /* 是否正在重放 */
    float start_time;                    /* 记录/重放开始时间 */
    float current_time;                  /* 当前时间 */
} DifferentialDriveCar;

/* 硬件抽象层函数 */
void set_motor(int motor_id, float speed);
float get_encoder(int encoder_id, int reset);
float get_time(void);

/* 应用功能函数 */
void car_init(DifferentialDriveCar* car);
void car_update(DifferentialDriveCar* car, float dt);
void car_start_recording(DifferentialDriveCar* car);
void car_stop_recording(DifferentialDriveCar* car);
void car_start_replaying(DifferentialDriveCar* car);
void car_stop_replaying(DifferentialDriveCar* car);
void car_save_path(DifferentialDriveCar* car, const char* filename);
int car_load_path(DifferentialDriveCar* car, const char* filename);
void car_print_state(DifferentialDriveCar* car);

/* 主函数 */
int main(void) {
    DifferentialDriveCar car;
    float prev_time, current_time, dt;
    char command;

    printf("差速驱动小车应用\n");
    printf("=================================\n");

    /* 初始化小车 */
    car_init(&car);

    /* 主循环 */
    prev_time = get_time();

    while (1) {
        /* 计算时间步长 */
        current_time = get_time();
        dt = current_time - prev_time;
        prev_time = current_time;

        /* 更新小车状态 */
        car_update(&car, dt);

        /* 打印小车状态 */
        car_print_state(&car);

        /* 处理用户命令 */
        printf("\n命令: [w]前进 [s]后退 [a]左转 [d]右转 [空格]停止 [r]开始记录 [t]停止记录 [p]开始重放 [o]停止重放 [q]退出\n");
        scanf(" %c", &command);

        switch (command) {
            case 'w': /* 前进 */
                BreezeDifferentialDrive_SetSpeed(&car.controller, 0.5f, 0.0f);
                break;

            case 's': /* 后退 */
                BreezeDifferentialDrive_SetSpeed(&car.controller, -0.5f, 0.0f);
                break;

            case 'a': /* 左转 */
                BreezeDifferentialDrive_SetSpeed(&car.controller, 0.3f, 1.0f);
                break;

            case 'd': /* 右转 */
                BreezeDifferentialDrive_SetSpeed(&car.controller, 0.3f, -1.0f);
                break;

            case ' ': /* 停止 */
                BreezeDifferentialDrive_SetSpeed(&car.controller, 0.0f, 0.0f);
                break;

            case 'r': /* 开始记录 */
                car_start_recording(&car);
                break;

            case 't': /* 停止记录 */
                car_stop_recording(&car);
                break;

            case 'p': /* 开始重放 */
                car_start_replaying(&car);
                break;

            case 'o': /* 停止重放 */
                car_stop_replaying(&car);
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

/* 初始化小车 */
void car_init(DifferentialDriveCar* car) {
    BreezeDifferentialDriveConfig config;

    if (!car) return;

    /* 清空结构体 */
    memset(car, 0, sizeof(DifferentialDriveCar));

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
    BreezeDifferentialDrive_Init(&car->controller, config, set_motor, get_encoder, 0.1f);

    /* 设置PID参数 */
    BreezeDifferentialDrive_SetPIDParams(&car->controller, 1.0f, 0.1f, 0.05f);

    printf("小车初始化完成\n");
}

/* 更新小车状态 */
void car_update(DifferentialDriveCar* car, float dt) {
    float left_encoder, right_encoder;
    float left_dist, right_dist;
    float delta_left, delta_right;
    float delta_dist, delta_theta;

    if (!car) return;

    /* 更新控制器 */
    BreezeDifferentialDrive_Update(&car->controller);

    /* 获取编码器数据（不重置） */
    left_encoder = car->controller.get_encoder(car->controller.config.left_encoder_id, 0);
    right_encoder = car->controller.get_encoder(car->controller.config.right_encoder_id, 0);

    /* 计算轮子行驶距离 */
    left_dist = left_encoder / car->controller.config.encoder_resolution *
                (2.0f * M_PI * car->controller.config.wheel_radius);
    right_dist = right_encoder / car->controller.config.encoder_resolution *
                 (2.0f * M_PI * car->controller.config.wheel_radius);

    /* 计算距离增量 */
    delta_left = left_dist - car->state.left_dist;
    delta_right = right_dist - car->state.right_dist;

    /* 更新累计距离 */
    car->state.left_dist = left_dist;
    car->state.right_dist = right_dist;

    /* 计算小车位移和转向 */
    delta_dist = (delta_left + delta_right) / 2.0f;
    delta_theta = (delta_right - delta_left) / car->controller.config.wheel_distance;

    /* 更新小车位置和方向 */
    car->state.theta += delta_theta;
    car->state.x += delta_dist * cosf(car->state.theta);
    car->state.y += delta_dist * sinf(car->state.theta);

    /* 计算当前速度 */
    car->state.v = (delta_left + delta_right) / (2.0f * dt);
    car->state.omega = (delta_right - delta_left) / (car->controller.config.wheel_distance * dt);

    /* 更新时间 */
    car->current_time = get_time();

    /* 如果正在记录，添加路径点 */
    if (car->is_recording && car->path_count < MAX_PATH_POINTS) {
        PathPoint* point = &car->path[car->path_count];
        point->x = car->state.x;
        point->y = car->state.y;
        point->theta = car->state.theta;
        point->v = car->state.v;
        point->omega = car->state.omega;
        point->timestamp = car->current_time - car->start_time;
        car->path_count++;
    }

    /* 如果正在重放，根据时间找到对应的路径点并设置速度 */
    if (car->is_replaying && car->path_count > 0) {
        float replay_time = car->current_time - car->start_time;
        int i;

        /* 找到当前时间对应的路径点 */
        for (i = 0; i < car->path_count - 1; i++) {
            if (replay_time >= car->path[i].timestamp &&
                replay_time < car->path[i+1].timestamp) {
                float t = (replay_time - car->path[i].timestamp) /
                          (car->path[i+1].timestamp - car->path[i].timestamp);
                float v = car->path[i].v * (1.0f - t) + car->path[i+1].v * t;
                float omega = car->path[i].omega * (1.0f - t) + car->path[i+1].omega * t;

                /* 设置速度 */
                BreezeDifferentialDrive_SetSpeed(&car->controller, v, omega);
                break;
            }
        }

        /* 如果已经到达最后一个点，停止重放 */
        if (replay_time > car->path[car->path_count-1].timestamp) {
            car_stop_replaying(car);
        }
    }
}

/* 开始记录路径 */
void car_start_recording(DifferentialDriveCar* car) {
    if (!car) return;

    /* 如果已经在记录，先停止 */
    if (car->is_recording) {
        car_stop_recording(car);
    }

    /* 清空路径数据 */
    car->path_count = 0;

    /* 设置标志和开始时间 */
    car->is_recording = 1;
    car->start_time = car->current_time;

    printf("开始记录路径\n");
}

/* 停止记录路径 */
void car_stop_recording(DifferentialDriveCar* car) {
    if (!car || !car->is_recording) return;

    car->is_recording = 0;

    /* 保存路径到文件 */
    car_save_path(car, PATH_FILE_NAME);

    printf("停止记录路径，共记录 %d 个点\n", car->path_count);
}

/* 开始重放路径 */
void car_start_replaying(DifferentialDriveCar* car) {
    if (!car) return;

    /* 如果已经在重放，先停止 */
    if (car->is_replaying) {
        car_stop_replaying(car);
    }

    /* 如果没有路径数据，尝试从文件加载 */
    if (car->path_count == 0) {
        if (!car_load_path(car, PATH_FILE_NAME)) {
            printf("没有可重放的路径数据\n");
            return;
        }
    }

    /* 设置标志和开始时间 */
    car->is_replaying = 1;
    car->start_time = car->current_time;

    printf("开始重放路径，共 %d 个点\n", car->path_count);
}

/* 停止重放路径 */
void car_stop_replaying(DifferentialDriveCar* car) {
    if (!car || !car->is_replaying) return;

    car->is_replaying = 0;

    /* 停止小车 */
    BreezeDifferentialDrive_SetSpeed(&car->controller, 0.0f, 0.0f);

    printf("停止重放路径\n");
}

/* 保存路径到文件 */
void car_save_path(DifferentialDriveCar* car, const char* filename) {
    FILE* file;
    int i;

    if (!car || !filename) return;

    file = fopen(filename, "w");
    if (!file) {
        printf("无法创建文件 %s\n", filename);
        return;
    }

    /* 写入路径点数量 */
    fprintf(file, "%d\n", car->path_count);

    /* 写入每个路径点 */
    for (i = 0; i < car->path_count; i++) {
        PathPoint* point = &car->path[i];
        fprintf(file, "%.6f %.6f %.6f %.6f %.6f %.6f\n",
                point->x, point->y, point->theta,
                point->v, point->omega, point->timestamp);
    }

    fclose(file);
    printf("路径已保存到文件 %s\n", filename);
}

/* 从文件加载路径 */
int car_load_path(DifferentialDriveCar* car, const char* filename) {
    FILE* file;
    int count, i;

    if (!car || !filename) return 0;

    file = fopen(filename, "r");
    if (!file) {
        printf("无法打开文件 %s\n", filename);
        return 0;
    }

    /* 读取路径点数量 */
    if (fscanf(file, "%d", &count) != 1) {
        fclose(file);
        return 0;
    }

    /* 检查数量是否合法 */
    if (count <= 0 || count > MAX_PATH_POINTS) {
        printf("文件中的路径点数量无效: %d\n", count);
        fclose(file);
        return 0;
    }

    /* 读取每个路径点 */
    car->path_count = 0;
    for (i = 0; i < count; i++) {
        PathPoint* point = &car->path[i];
        if (fscanf(file, "%f %f %f %f %f %f",
                  &point->x, &point->y, &point->theta,
                  &point->v, &point->omega, &point->timestamp) != 6) {
            break;
        }
        car->path_count++;
    }

    fclose(file);
    printf("从文件 %s 加载了 %d 个路径点\n", filename, car->path_count);

    return car->path_count > 0;
}

/* 打印小车状态 */
void car_print_state(DifferentialDriveCar* car) {
    if (!car) return;

    printf("\n小车状态:\n");
    printf("位置: (%.2f, %.2f) 方向: %.2f°\n",
           car->state.x, car->state.y, car->state.theta * 180.0f / M_PI);
    printf("速度: %.2f m/s 角速度: %.2f rad/s\n",
           car->state.v, car->state.omega);
    printf("左轮距离: %.2f m 右轮距离: %.2f m\n",
           car->state.left_dist, car->state.right_dist);

    if (car->is_recording) {
        printf("正在记录: %d 个点, 时间: %.1f s\n",
               car->path_count, car->current_time - car->start_time);
    }

    if (car->is_replaying) {
        printf("正在重放: %d/%d, 时间: %.1f s\n",
               car->path_count > 0 ?
               (int)((car->current_time - car->start_time) / car->path[car->path_count-1].timestamp * car->path_count) : 0,
               car->path_count,
               car->current_time - car->start_time);
    }
}