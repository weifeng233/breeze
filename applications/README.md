# Breeze 框架应用示例（旧 C 算法库）

> **这些示例当前无法编译。** 它们依赖 `include/breeze/breeze.h`，而该头文件聚合的 63 个头文件
> 中有两个存在缺陷（`BreezeCommBuffer` 重复定义、`BreezeSplineInterpolation_Free` 声明顺序错误），
> 详见 [../docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) §8.1。
>
> 保留本目录是为了给算法层向 Zig 迁移提供对照，不是可运行的样例。
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

| 目标 | 命令 | 结果 |
|---|---|---|
| `examples/*.c`（5 个） | `gcc examples/example.c -lm` | ❌ 失败 |
| `applications/differential_drive_car` | `gcc differential_drive_car.c -lm` | ❌ 失败 |
| `applications/encoder_odometry` | `gcc encoder_odometry.c -lm` | ❌ 失败 |
| `tests/comm/test_comm_interface.c` | `gcc test_comm_interface.c ../../include/breeze/core/globals.c -lm` | ✅ 11/11 通过 |

编译失败的原因在头文件，不在应用代码。修好那两个头文件后这些示例应当可以构建，
但那属于算法层迁移工作的一部分。

## 硬件抽象

这些应用默认使用模拟的硬件接口函数。要在实际硬件上部署，需要实现：

1. **硬件抽象层函数**：`set_motor`（电机控制）、`get_encoder`（编码器读取）、`get_time`（时间获取）
2. **硬件配置参数**：轮子尺寸与轮距、电机与编码器 ID、编码器分辨率、PID 参数

## 许可证

[MIT](../LICENSE)，与 Breeze 框架一致。

