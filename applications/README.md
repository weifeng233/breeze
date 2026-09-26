# Breeze 框架应用示例（旧 C 算法库）

> **这些示例现在可以编译、链接并运行。** 它们依赖 `include/breeze/breeze.h`，
> 该头文件曾经因为两处缺陷而无法编译（`BreezeCommBuffer` 重复定义、
> `BreezeSplineInterpolation_Free` 声明顺序错误），现已修好，并由
> `pwsh tools/check-c.ps1` 在 CI 中守住——修的过程与依据见
> [../docs/REVIEW.md](../docs/REVIEW.md) 第 26 节。
>
> 保留本目录是为了给算法层向 Zig 迁移提供对照，不是教学样例。
> 新的、可运行的示例在 [`../examples/`](../examples)，用 `zig build demo` 运行。

本目录包含基于旧 C 算法库编写的应用示例，展示了如何使用框架的算法模块构建应用。

## 应用列表

### 1. 差速驱动小车 (differential_drive_car)

一个完整的差速驱动小车应用，包含以下功能：

- 基本运动控制（前进、后退、左转、右转）
- 通过编码器数据实现测速和测距
- 路径记录功能，可以记录小车运动轨迹
- 路径重放功能，可以按照记录的轨迹重放小车运动

展示了如何使用差速驱动控制器、PID 控制器和文件操作实现一个具有记录和重放功能的移动机器人。

[查看详细说明](differential_drive_car/README.md)

### 2. 基于编码器的里程计 (encoder_odometry)

一个专注于测速和测距功能的里程计应用，包含以下功能：

- 实时计算小车的线速度和角速度
- 测量左右轮分别行驶的距离以及总行驶距离
- 根据轮子转动计算小车在平面上的位置和方向
- 基本运动控制和里程计重置功能

展示了如何使用编码器数据实现移动机器人的位置估计，是机器人导航的基础。

[查看详细说明](encoder_odometry/README.md)

## 编译状态

以下命令都是实测过的（gcc 13.1.0，`-std=gnu17 -Wall -Wextra -Werror`）：

| 目标 | 命令（在所示目录下执行） | 结果 |
|---|---|---|
| `examples/*.c`（5 个） | `gcc example.c -o example -lm` | ✅ 链接并运行，输出 30–237 行 |
| `applications/differential_drive_car` | `gcc differential_drive_car.c -o differential_drive_car -lm` | ✅ 链接；交互式，`q` 退出 |
| `applications/encoder_odometry` | `gcc encoder_odometry.c -o encoder_odometry -lm` | ✅ 链接；交互式，`q` 退出 |
| `tests/comm/test_comm_interface.c` | `gcc test_comm_interface.c ../../include/breeze/core/globals.c -lm` | ✅ 11/11 通过（42/42 断言） |

两个应用是**交互式**的：`main` 里是 `while (1)` + `scanf`，靠 `q` 退出。因此在 CI 里
它们是被喂入 `q` 并有超时保护的——一个因此挂住的程序会被报成失败，而不是把 CI 挂住。

这道检查由 `pwsh tools/check-c.ps1` 执行，它逐个头文件独立编译、逐个 TU 编译、
把每个程序链接并运行，再与 README 里的计数比对。它需要 gcc，因此不在纯 Zig 的
`zig build ci` 里，而是 CI workflow 的单独一步。

## 硬件抽象

这些应用默认使用模拟的硬件接口函数。要在实际硬件上部署，需要实现：

1. **硬件抽象层函数**：`set_motor`（电机控制）、`get_encoder`（编码器读取）、`get_time`（时间获取）
2. **硬件配置参数**：轮子尺寸与轮距、电机与编码器 ID、编码器分辨率、PID 参数

## 许可证

[MIT](../LICENSE)，与 Breeze 框架一致。

