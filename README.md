# Breeze Framework

面向 **ARM Cortex-M** 与 **RISC-V** 裸机目标的确定性协作式任务内核，用现代 Zig 编写。
同一份内核可以在主机上用虚拟时钟运行，因此固件逻辑能在工作站上被精确断言。

> **状态**：内核与模块系统已实现。`zig build ci` 通过：**75 个单元测试**、
> **7 个目标交叉编译**（CYT2BL3、CYT4BB7 的 CM0+ 与 CM7、RT1064 各有一个具名目标）。
> 这两个数字由 CI 校验，不允许漂移（见 `.github/workflows/ci.yml`）。
> C 版本算法库仍保留在 `include/`、`src/` 中作为迁移参考，**但它当前无法编译**，
> 原因见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) 第 8 节。

## 设计要点

- **确定性**：非抢占、无上下文切换、无堆、无动态任务。任务按声明顺序执行，时序可静态推理。
- **零栈任务**：每个任务没有独立栈，共用主栈。状态机状态仅 12 字节，与指令条数无关。
- **comptime 任务表**：`inline for` 把任务表展开为直接调用，没有函数指针间接跳转，RAM 中也没有任务表。
- **comptime 状态机**：顺序逻辑（发送 → 等待 → 超时 → 重试 → 放弃）写成编译期指令表，
  编译器生成状态与跳转表，没有手写 `switch`，也没有 `__LINE__` 宏技巧。
- **三个函数的 HAL**：`now` / `criticalEnter` / `criticalExit`，作为 comptime 类型参数传入，
  零虚表开销。主机与目标共用同一份调度器代码。
- **编译期模块组合**：模块用 `Manifest` 声明依赖与外设需求，`App` 在编译期校验——
  缺失的依赖、重复的提供者、拼错的配置字段都会直接编译失败，没有 YAML、没有代码生成步骤。
- **无锁 IPC**：`Channel` 是调用者提供存储的 SPSC 环，`Topic` 是编译期主题 +
  与 LibXR 兼容的遥测帧格式，两者都不分配、不加锁。

## 快速开始

```zig
const breeze = @import("breeze");
const hal = breeze.hal.host;

const Control = struct { ticks: u32 = 0 };

fn poll(ctx: *Control, now: breeze.Tick, events: u32) breeze.Step {
    _ = .{ now, events };
    ctx.ticks += 1;
    return .finished;
}

var control = Control{};   // 任务上下文必须是静态的

const Sched = breeze.Scheduler(hal.HostHal, .{
    .{ .name = "control", .period_ms = 20, .ctx = &control, .poll = poll },
});

pub fn main() void {
    var sched = Sched.init();
    hal.HostHal.runFor(&sched, 1000);   // 虚拟时间，不 sleep
    // control.ticks == 50
}
```

顺序逻辑用编译期指令表表达：

```zig
const instrs = [_]breeze.Instr(Handshake){
    .{ .call = sendRequest },
    .{ .wait_event_timeout = .{ .mask = EVT_ACK, .timeout_ms = 50 } },
    .{ .branch_event = 6 },                    // 收到 ACK → 成功分支
    .{ .branch_if = .{ .pred = mayRetry, .target = 0 } },
    .{ .call = onGiveUp },
    .{ .jump = 7 },
    .{ .call = onSuccess },
    .finish,
};
```

## 模块组合

模块用 `Manifest` 声明自己需要什么、提供什么、多久跑一次：

```zig
const Imu = struct {
    pub const manifest = breeze.Manifest{
        .name = "Imu",
        .hardware = &.{"i2c0"},          // 外设：由应用声明
        .publishes = &.{"attitude"},
        .period_ms = 5,                  // 200 Hz
    };
    pub const Config = struct { alpha: f32 = 0.2 };
    pub const State = struct { attitude: Attitude.Value = .{} };

    pub fn init(self: *State, cfg: Config) void { ... }
    pub fn poll(self: *State, now: Tick, events: u32) Step { ... }
};
```

然后在应用里组装——**依赖检查全部在编译期完成**：

```zig
const App = breeze.AppWithHardware(.{
    .{ .module = Imu,     .state = &imu_state,     .config = .{ .alpha = 0.3 } },
    .{ .module = Chassis, .state = &chassis_state, .config = .{ .counts_per_meter = 1850.0 } },
}, &.{ "i2c0", "uart0", "motor_l", "motor_r" });

const Sched = App.SchedulerFor(BoardHal);
```

`depends` 与 `hardware` 是两条独立的线：前者检查"有没有别的模块提供"，
后者检查"应用有没有声明这个外设"。把外设包装成假模块只为通过检查是没有意义的。

## 遥测

`Topic` 在编译期把名字折成 CRC32，帧格式与 LibXR 兼容：

```zig
const Attitude = breeze.Topic("attitude", extern struct { roll: f32, pitch: f32, yaw: f32 });

var buf: [64]u8 = undefined;
const frame = try Attitude.pack(&buf, &value, timestamp_us);
```

> CRC 多项式未在 LibXR 文档中给出，本实现使用 CRC-32/ISO-HDLC 与 CRC-8/ATM。
> 若要与真实 LibXR 上位机逐字节互通，需先按
> [docs/FUSION.md](docs/FUSION.md) §4.2 复核。

## 构建

```bash
zig build test           # 75 个单元测试
zig build demo           # 主机虚拟时钟演示
zig build check-targets  # 交叉编译 7 个目标
zig build ci             # 格式检查 + 测试 + 目标编译
```

交叉编译不需要额外安装工具链，`build.zig` 里已声明目标。

### 把内核引入另一个仓库

没有包管理器也能用。`tools/vendor.ps1` 把内核复制过去，并写出 `VENDORED.md`
记录来源 commit 与每个文件的 sha256：

```bash
# 复制到目标树
pwsh tools/vendor.ps1 -Dest ../my-project/lib/breeze
# 之后检查两边有没有漂移
pwsh tools/vendor.ps1 -Dest ../my-project/lib/breeze -Check
```

复制的是 `src/breeze_kernel.zig`（只含内核与 `app.zig`，不含平台后端），
使用者自己提供 `now` / `criticalEnter` / `criticalExit` 三个函数。

[Smartcar-Template](https://github.com/weifeng233/Smartcar-Template) 的 cyt2bl3 模板
就是这样接入的，实测整机 RAM 从 404 B 降到 72 B；过程与数据见
[docs/FUSION.md](docs/FUSION.md) §7。

## 实测开销

在 Zig 0.16.0、`-OReleaseSmall` 下实测。复现命令见
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) §4。

| 项 | 大小 |
|---|---|
| `Program` 状态 | 12 字节（与指令条数无关） |
| `TaskState`（每任务） | 24 字节 |
| 每任务额外栈 | 0 |
| Cortex-M0 固件骨架 | 596 B flash / 136 B RAM |
| RISC-V32 固件骨架 | 762 B flash / 136 B RAM |
| 智能车融合固件（CYT2BL3，4 模块 + 遥测） | 1434 B flash / 280 B RAM |
| 调度抖动 | 0 tick |

固件骨架的 136 B RAM 中有 76 B 是示例演示用的 64 字节 UART 接收环及其 12 字节头部；
不使用该环时是 60 B。

## 目标平台支持

| 架构 | 目标三元组 | 状态 |
|---|---|---|
| ARM Cortex-M0 / M0+ | `thumb-freestanding-eabi` | ✅ 已编译验证 |
| ARM Cortex-M4F / M7F | `thumb-freestanding-eabihf` | ✅ 已编译验证 |
| RISC-V RV32 | `riscv32-freestanding-eabi` | ✅ 已编译验证 |
| 主机（测试与仿真） | 任意 | ✅ 虚拟时钟后端 |
| 8051 / MCS-51 | — | ❌ Zig 不支持该架构 |

`zig build check-targets` 的七个具名目标与三款智能车芯片的对应关系：

| 目标名 | 对应芯片 | 三元组 |
|---|---|---|
| `smartcar_cyt2bl3` | CYT2BL3 | `thumb-freestanding-eabihf` / `cortex_m4+vfp4d16sp` |
| `smartcar_cyt4bb7_cm0p` | CYT4BB7 CM0+ | `thumb-freestanding-eabi` / `cortex_m0plus` |
| `smartcar_cyt4bb7_cm7` | CYT4BB7 CM7 | `thumb-freestanding-eabihf` / `cortex_m7+fp_armv8d16sp` |
| `smartcar_rt1064` | RT1064 | `thumb-freestanding-eabihf` / `cortex_m7+fp_armv8d16sp` |
| `cortex_m0` / `cortex_m4` | 接线示例 | — |
| `riscv32` | 接线示例 | — |

RT1064 与 CYT4BB7 的 CM7 核三元组完全相同，因此二者共享同一份编译验证。
之所以仍然单独列出 `smartcar_rt1064`，是为了让"RT1064 已验证"这句话**可以被审阅**——
否则它依赖读者自己发现两块板恰好同三元组。

以上是**交叉编译验证**：证明代码能为该目标编译通过。它不等于已在硬件上运行，
真机验证情况见各模板仓库。

## 文档

| 文档 | 内容 |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | 设计决策、调度语义、中断规则、指令集、实测数据、迁移计划 |
| [docs/FUSION.md](docs/FUSION.md) | 与 LibXR/XRobot 及 Smartcar-Template 的融合方案 |
| [docs/LIBXR-XROBOT.md](docs/LIBXR-XROBOT.md) | LibXR/XRobot 技术调研（外部资料核对记录） |
| [docs/REVIEW.md](docs/REVIEW.md) | 一次外部代码评审的逐条处理记录：采纳、驳回（附证据）、推迟 |

示例代码：

| 文件 | 用途 |
|---|---|
| `examples/scheduler_demo.zig` | 可运行的主机演示（`zig build demo`） |
| `examples/firmware_smartcar.zig` | 四模块融合固件，三款智能车芯片各有具名编译验证目标 |
| `examples/firmware_cortex_m.zig` | Cortex-M 接线示例，也是交叉编译检查 |
| `examples/firmware_riscv.zig` | RISC-V 接线示例，同上 |

## 许可证

[MIT](LICENSE)。`include/`、`src/` 下的 C 算法库是同一许可证下的历史代码。

本项目参考了 [LibXR](https://github.com/Jiu-xiao/libxr) 与
[XRobot](https://github.com/xrobot-org) 的架构设计（均为 Apache-2.0），
但**未链接其运行时**；借鉴范围的说明见 [docs/FUSION.md](docs/FUSION.md) §3。
