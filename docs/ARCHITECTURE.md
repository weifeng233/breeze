# Breeze 架构说明

> 本文档描述 Zig 重写后的 Breeze 框架内核。C 版本的算法库（`include/`、`src/`）仍然保留，
> 作为算法层迁移的参考与对照，迁移计划见文末。

## 1. 为什么换 Zig，以及放弃了什么

| 旧设想 | 现状 | 原因 |
|---|---|---|
| C89，兼容 8051 | **放弃** | Zig 的目标架构表里没有 mcs51/8051。ARM（`thumb`/`arm`/`aarch64`）与 RISC-V（`riscv32`/`riscv64`）都是官方目标。 |
| 仅头文件，复制即用 | **放弃** | 预处理器带来的重复定义问题（C 版本里 `globals.c` 与 `src/*.c` 定义同一批符号、`breeze.h` 从未整体编译通过）在 Zig 里不存在：`@import` 天然单一定义。 |
| Protothread 式协程 | **换模型** | Zig 的 `switch` 分支不 fall-through，也没有 `goto`，`case __LINE__` 那种 Duff 设备式续点无法表达。Zig 0.16 也没有 `async`/`await` 语言特性；新的 `std.Io` 是宿主平台 I/O 接口（线程池 / io_uring / kqueue），裸机没有可用后端。 |
| 阻塞式 HAL（`delay`、单次定时器） | **换契约** | 单例定时器无法表达 N 个并发超时；协作式调度下阻塞调用会拖垮整个循环。 |

替代方案是 **comptime 生成的状态机**：任务写成编译期指令序列，编译器生成状态、跳转表与恢复点。
它拿回了 Protothread 的全部收益，并且更好：

* 没有 `__LINE__` 魔法，同一行两个等待点不会冲突，文件超过 65535 行也不会失效；
* 没有 `-Wimplicit-fallthrough` 告警（C 版本必须在**每一个**等待点压制它）；
* 状态只有一个索引，`@sizeOf` 编译期可知。

## 2. 内核模型

```
应用任务  ──▶  Scheduler(Hal, 任务表)  ──▶  HAL（now / criticalEnter / criticalExit）
                    │
                    ├── Program(Ctx, 指令表)   顺序逻辑与超时
                    └── EventFlags             ISR → 任务的唯一通道
```

### 调度语义

1. **单一调度上下文**：`main` 超循环，或已存在 RTOS 里的一个任务。无抢占、无上下文切换。
2. **任务表是编译期的**：`inline for` 把它展开成直接调用，没有函数指针间接跳转，RAM 里也没有任务表。
3. **时间来自 `Hal.now()`**：ISR 只负责推进时基，调度器**绝不**在中断上下文运行。
4. **固定栅格**：周期任务按 `next_run += period` 调度，平均频率精确、抖动不累积；落后超过一整个周期就重新对齐，并把这次落后**计数**而不是爆发式补跑。
5. **`.pending` 优先**：任务返回 `.pending` 表示序列未完成，下一轮立即再轮询；返回 `.finished` 才回到周期栅格。

### 为什么任务上下文必须是静态的

`ctx` 指针必须是编译期可知的（即指向容器级变量）。这不是测试的权宜之计，而是内核的语义要求：
没有栈可以保存，任何需要跨越挂起点的东西都必须活在被调用帧之外。内核用编译期检查把这条规则变成错误，
而不是让它变成运行期悬垂指针。

### 中断规则

ISR 只允许做三件事：

1. 推进 `now()` 读取的时基（`hal.tickIsr()`）；
2. 往无锁单生产者环里塞字节（`RxRing.pushFromIsr`）；
3. 置事件标志（`EventFlags.setFromIsr`）。

ISR **不得**调用调度器、分配内存、格式化字符串。清除标志是任务的责任，且必须走
`EventFlags.clear`（内部关中断），否则会与 ISR 的置位竞争而丢事件。

## 3. 指令集

`Program` 由编译期指令表构成，`poll` 每次推进直到挂起或结束：

| 指令 | 语义 |
|---|---|
| `call` | 执行动作后前进 |
| `call_until` | 反复执行动作直到返回 true —— 轮询驱动的原语 |
| `wait_ms` | 从**首次到达**该指令起等待固定毫秒 |
| `wait_event` | 等到事件掩码出现 |
| `wait_for` | 等到谓词为真 |
| `wait_event_timeout` | 等事件或超时，结果记入 `timed_out` |
| `branch_event` / `branch_timeout` | 按上一次定时等待的结果跳转 |
| `branch_if` | 谓词为真则跳转 —— 重试/放弃逻辑靠它表达 |
| `jump` | 无条件跳转；跳到 0 即循环任务 |
| `finish` | 提前结束，程序计数器复位 |

越界的跳转目标在**编译期**报错；只由 `jump` 组成的病态程序由每轮指令预算兜底，变成可观测的停滞而不是死循环。

## 4. 实测数据

在 Zig 0.16.0 上实测（`zig build ci` 全绿：29 个单元测试 + 3 个目标编译）。

**每任务开销（编译期精确）**

| 项 | 大小 |
|---|---|
| `Program` 状态 | 12 字节，**与指令条数无关** |
| `TaskState`（周期任务） | 24 字节 |
| 每任务额外栈 | **0**（共用主栈；对照 RTOS 每任务 256 B–2 KB） |

**固件占用**（同一份内核 + 两个任务的完整示例，`-OReleaseSmall`）

| 目标 | Flash | RAM |
|---|---|---|
| Cortex-M0 (`thumb-freestanding`) | 600 B | 60 B |
| Cortex-M4 | 594 B | 60 B |
| RISC-V32 (`riscv32-freestanding`) | 960 B* | 60 B |

\* 其中 208 B 是 `.eh_frame` 展开信息，去掉 `-funwind-tables` 即可省下。

60 B RAM = 事件标志 4 + 两个任务状态 48 + 计数 8，与设计预期完全一致。

**时序（虚拟时钟仿真）**

* 1000 ms 内 20 ms 周期任务恰好触发 50 次，**抖动 0 tick**；
* 1000 ms 内 10 ms / 50 ms 两个任务分别 100 / 20 次，互不干扰；
* 50 ms 超时在恰好第 51 tick 走超时分支；
* 400 ms 内 20 ms 控制任务恰好 20 次；
* 循环卡死 100 ms（十个周期）后只补跑一次并记录 `resyncs=1`、`max_late=90`。

## 5. 目录结构

```
build.zig                 构建、测试、交叉编译、格式检查
src/breeze.zig            公开 API 根（`@import("breeze")`）
src/kernel/
  tick.zig                回绕安全的毫秒时基运算
  events.zig              ISR → 任务的事件标志
  program.zig             comptime 状态机（指令集与 Program）
  scheduler.zig           任务表、周期栅格、健康统计
  hal.zig                 HAL 契约 + 编译期一致性检查
src/hal/
  host.zig                虚拟时钟后端（测试 / 仿真）
  cortex_m.zig            Cortex-M：SysTick、PRIMASK、WFI、SPSC 环
  riscv.zig               RISC-V：mtime/mtimecmp、mstatus、WFI、SPSC 环
examples/
  scheduler_demo.zig      主机仿真演示（`zig build demo`）
  firmware_cortex_m.zig   Cortex-M 固件骨架（也是交叉编译检查）
  firmware_riscv.zig      RISC-V 固件骨架（同上）
```

## 6. 构建

```bash
zig build test           # 29 个单元测试
zig build demo           # 主机虚拟时钟演示
zig build check-targets  # 交叉编译 Cortex-M0 / M4 / RISC-V32
zig build ci             # 格式检查 + 测试 + 目标编译
```

交叉编译无需安装任何工具链，`build.zig` 里已声明目标。

## 7. 测试策略

内核在主机上以**虚拟时钟**运行，所以：

* 不 sleep、不起线程，十秒钟的固件场景在微秒内跑完；
* 完全可复现，不会因 CI 机器负载而闪断；
* 断言的不是"大概齐"，而是精确 tick：`expectEqual(@as(Tick, 980), fire_ticks[49])`。

`Hal` 是 comptime 类型参数而不是 vtable，所以主机与目标共用同一份调度器代码，没有任何间接层；
只有 `src/hal/*.zig` 三个文件不同。

## 8. 算法层迁移计划（下一步）

现有 C 算法库（filter / control / image / math，约 50 个头文件）**尚未迁移**。迁移时的硬性约束：

1. **分配器参数化**：现有代码有 100+ 处 `malloc/calloc/free`（`hough_transform`、`histogram`、
   `interpolation` 等），其中 4 个头文件连 `stdlib.h` 都没包含。Zig 版本必须把 allocator 作为参数传入，
   并提供"调用者提供缓冲区"的变体，否则嵌入式目标无法使用。
2. **错误用 error union**：取代 `BreezeErrorCode` 枚举 + 手工传播。
3. **保持纯函数**：算法层不感知调度，只被任务调用。这一点现有设计已经满足，迁移时不要引入内核依赖。
4. **顺序建议**：math（无依赖）→ filter → control → image（重内存，最后做）。
   每迁移一个模块，就在 `src/` 下建立对应目录并补 `zig test`，与 C 版本做数值对照。

## 9. 已知限制

* 无优先级抢占；需要硬实时请把本内核放进一个 RTOS 任务里，由 RTOS 处理硬实时部分。
* 无动态任务创建、无堆、无栈保护。
* `Program` 的等待点不能嵌套在另一层 `switch` 里（与 C 版 Protothread 相同的约束，但这里由
  指令表结构天然规避）。
* 单个任务不让出会独占循环；用 `worstLateness()` / `totalResyncs()` 检测。
