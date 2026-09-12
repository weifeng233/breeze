# Breeze 架构说明

> 本文档描述 Zig 重写后的 Breeze 框架内核：设计决策、调度语义、中断规则与实测数据。
>
> 相关文档：[FUSION.md](FUSION.md)（融合方案）· [LIBXR-XROBOT.md](LIBXR-XROBOT.md)（调研记录）
>
> C 版本的算法库（`include/`、`src/`）仍然保留，作为算法层迁移的参考与对照。
> **它当前无法编译**，原因与迁移计划见第 8 节。

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
应用层    App(模块声明)             编译期依赖校验、任务表生成
   │
   ├── Program(Ctx, 指令表)         顺序逻辑与超时
   ├── Channel(T, N)                ISR → 任务的无锁元素通道
   ├── Topic("name", T)             编译期主题 + 遥测帧
   └── EventFlags                   ISR → 任务的事件标志
   │
内核层    Scheduler(Hal, 任务表)     周期栅格、健康统计
   │
HAL       now / criticalEnter / criticalExit   （comptime 类型参数，非 vtable）
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
2. 往无锁单生产者环里塞数据（`Channel.pushFromIsr`）；
3. 置事件标志（`EventFlags.setFromIsr`）。

ISR **不得**调用调度器、分配内存、格式化字符串。清除标志是任务的责任，且必须走
`EventFlags.clear`（内部关中断），否则会与 ISR 的置位竞争而丢事件。

### 关于共享内存：为什么不用 `@atomicLoad`

单核 MCU 上，ISR 与调度器不会同时执行；32 位对齐访问不会撕裂；每个共享字只有一个写者。
因此 `volatile` 语义已经足够——而**原子访问在 ARMv6-M 上会导致链接失败**：

实测（Zig 0.16.0，`thumb-freestanding-eabi -mcpu cortex_m0plus`），任何内存序的
`@atomicLoad`/`@atomicStore` 都会产生对 `__atomic_load_4` / `__atomic_store_4` 的调用，
因为 ARMv6-M 没有 `LDREX`/`STREX`，LLVM 不认为 32 位原子是无锁的。这些符号不在嵌入式常用的
picolibc 归档里，而许多工程用 `-fno-compiler-rt` 编译 Zig 对象。

所以 `src/kernel/shared.zig` 集中定义 volatile 访问纪律，内核里没有一处原子操作。
完整推导与实测表见 [FUSION.md](FUSION.md) §5.1。

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

越界的跳转目标在**编译期**报错；只由 `jump` 组成的病态程序由每轮指令预算兜底，
变成可观测的停滞而不是死循环。

## 4. 实测数据

环境：Zig 0.16.0，`-OReleaseSmall -fstrip -fno-unwind-tables -fno-compiler-rt`。
`zig build ci` 全绿。测试与目标的数量以 [README](../README.md) 的状态行为准，
本文件不再重复——重复过的数字已经漂移过三次，CI 现在会校验 README 里那一个。

**每任务开销（编译期精确）**

| 项 | 大小 |
|---|---|
| `Program` 状态 | 12 字节，**与指令条数无关** |
| `TaskState`（周期任务） | 28 字节 |
| 每任务额外栈 | **0**（共用主栈；对照 RTOS 每任务 256 B–2 KB） |

**协调原语（编译期精确）**

| 类型 | 大小 | 说明 |
|---|---|---|
| `Join` | 4 字节 | 只数分支，分支数不限 |
| `Join.Of(T, cap)` | `cap` + `cap×sizeof(T)` + 1 | 每分支一个状态字节，`cap ≤ 32` |
| `Join.Of(u16, 2)` | 8 字节 | 融合固件 `Boot` 用的就是这个 |
| `Limiter` | 12 字节 | 窗口 + 游标 + 总数 |

**固件占用**

| 固件 | Flash | RAM |
|---|---|---|
| `firmware_cortex_m.zig` → Cortex-M0 | 628 B | 184 B |
| `firmware_cortex_m.zig` → Cortex-M4F | 672 B | 184 B |
| `firmware_riscv.zig` → RISC-V32 | 868 B | 200 B |
| `smartcar/firmware.zig` → CYT2BL3 (CM4F) | 2036 B | 496 B |
| `smartcar/firmware.zig` → CYT4BB7 CM0+ | 2024 B | 496 B |
| `smartcar/firmware.zig` → CYT4BB7 CM7F | 2038 B | 496 B |

这张表在 2026-09 重测时发现**已经漂移**：`TaskState` 记的是 24（实际 28），Cortex-M0 骨架
记的是 596/136（当时实际 616/184），融合固件记的是 1434/280（当时实际 1556/364）——
上一次内核改动之后就没有再对过。同一次重测还发现 `tools/elfsize.ps1` 把 `.data` 只算作
flash，漏掉了它在 RAM 里的那份拷贝（融合固件少报 96 B），工具已修。

现在这张表里的三个骨架/融合固件数字**由 CI 校验**：`pwsh tools/sizes.ps1` 按上面的命令
重新编译、测量并与 README 的「实测开销」表逐项比对。也就是说 README 是这几个数字的唯一
来源，本表只是引用——如果两处不一致，以 CI 报错为准（它会指出是哪一行）。

Cortex-M0 骨架的 184 B RAM 中，76 B 是示例演示用的 64 字节 UART 接收环及其 12 字节通道头部，
56 B 是两个任务的 `TaskState`，28 B 是任务上下文，其余 24 B 是事件标志与调度器自身状态。

复现方式：`tools/elfsize.ps1` 直接读 ELF 段表报告占用（Zig 0.16 的 `zig objdump` 尚是占位实现，
只打印 `TODO dump elf file`）。

```bash
zig build-obj -target thumb-freestanding-eabi -mcpu cortex_m0 \
  -OReleaseSmall -fstrip -fno-unwind-tables -fno-compiler-rt \
  --dep breeze "-Mroot=examples/firmware_cortex_m.zig" "-Mbreeze=src/breeze.zig" \
  -femit-bin=kernel_cm0.o
pwsh tools/elfsize.ps1 kernel_cm0.o
```

**时序（虚拟时钟仿真，由单元测试断言）**

* 1000 ms 内 20 ms 周期任务恰好触发 50 次，**抖动 0 tick**；
* 1000 ms 内 10 ms / 50 ms 两个任务分别 100 / 20 次，互不干扰；
* 50 ms 超时在恰好第 51 tick 走超时分支；
* 400 ms 内 20 ms 控制任务恰好 20 次；
* 循环卡死 100 ms（十个周期）后只补跑一次并记录 `resyncs=1`、`max_late=90`。

## 5. 目录结构

```
build.zig                 构建、测试、交叉编译、格式检查
build.zig.zon             包清单
LICENSE                   MIT
src/breeze.zig            公开 API 根（`@import("breeze")`）
src/app.zig               模块组合：Manifest / App / AppWithHardware
src/kernel/
  tick.zig                回绕安全的毫秒时基运算
  events.zig              ISR → 任务的事件标志
  program.zig             comptime 状态机（指令集与 Program）
  scheduler.zig           任务表、周期栅格、健康统计
  hal.zig                 HAL 契约 + 编译期一致性检查
  shared.zig              ISR 共享内存的 volatile 访问纪律
  chan.zig                调用者提供存储的无锁 SPSC 通道
  topic.zig               编译期主题 + 遥测帧 + CRC
src/hal/
  host.zig                虚拟时钟后端（测试 / 仿真）
  cortex_m.zig            Cortex-M：SysTick、PRIMASK、WFI、RxRing
  riscv.zig               RISC-V：mtime/mtimecmp、mstatus、WFI、RxRing
examples/
  scheduler_demo.zig      主机仿真演示（`zig build demo`）
  smartcar/               四模块融合固件（三款智能车芯片验证），按模板的粒度分文件：
    topics.zig              模块之间的通信契约（3 个 topic）
    board.zig               板级：内核 HAL + 模块 I/O 两个面、缓冲区
    app.zig                 装配：模块实例、模块表、调度器（对 Io/Hal 泛型）
    firmware.zig            目标根：中断向量 + 入口（check-targets 编的就是它）
    app_test.zig            主机测试：用假 I/O 跑真实的模块图
    modules/                每个模块一个文件，对 I/O 面泛型
  firmware_cortex_m.zig   Cortex-M 接线示例（也是交叉编译检查）
  firmware_riscv.zig      RISC-V 接线示例（同上）
tools/
  elfsize.ps1             读 ELF 段表报告 flash / RAM 占用
  sizes.ps1               重新编译三个镜像、测量并与 README 的占用表比对（CI 用）
docs/
  ARCHITECTURE.md         本文档
  FUSION.md               融合方案
  LIBXR-XROBOT.md         LibXR/XRobot 调研记录
include/ src/             旧 C 算法库（迁移参考，当前不可编译）
applications/             基于旧 C 算法库的应用示例（同上）
```

`hal.RxRing` 是 `Channel(u8, N)` 加了自带缓冲的便捷形式——框架里只有一处 SPSC 实现，
索引算术、满/空判定与 volatile 纪律都在 `chan.zig`。

## 6. 构建

```bash
zig build test           # 单元测试（数量见 README 状态行）
zig build demo           # 主机虚拟时钟演示
zig build check-targets  # 交叉编译所有目标
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

### 内联汇编的验证陷阱

Zig 惰性分析：**未被引用的 `inline fn` 根本不会被汇编**。这一点在本项目里踩过——`cortex_m.zig`
里的 `criticalEnterSaved` / `criticalExitRestore` 一度没有任何调用者，于是其中的汇编从未被汇编过。
更糟的是，用 `comptime { _ = &fn; }` 取地址**也不能**强制生成代码：把 `msr primask` 故意改成
错误助记符后，构建依然通过。

真正有效的做法是 `export fn` 包装——导出函数必须被生成。现在两个函数都有 C ABI 包装，
并且验证方式是"故意写错助记符，构建必须失败"。

同样的道理适用于 HAL 后端里的任何内联汇编：**只有被导出或被实际调用的代码才算验证过**。

注意链接器仍会回收它：`--gc-sections` 下，没有任何调用者的导出函数会从最终镜像里消失。
所以"汇编被验证过"和"函数出现在固件里"是两件事。`board.zig`（见
[FUSION.md](FUSION.md) §7）里 `breeze_board_wfi` 就是这样——它在对象文件里被汇编和校验，
然后被链接器删掉，因为调度器调用的是 `BoardHal.idle`，而那条 `wfi` 已经内联进调度循环了。

### `test` 块里的 `@import` 也会被解析

Zig 在 `build-obj`（不是跑测试）时**也会解析 `test` 块中的 `@import`**。这一点是用
"故意 import 一个不存在的文件"实测确认的：

```zig
pub fn value() u32 { return 7; }

test "imports a file that does not exist" {
    const missing = @import("does_not_exist.zig");
    _ = missing;
}
```

对上面的文件执行 `zig build-obj`，会报 `unable to load 'does_not_exist.zig'`。

后果：内核的几个文件在测试夹具里 import `../hal/host.zig`，所以用 `tools/vendor.ps1`
把内核复制到别的仓库时**必须把 `hal/host.zig` 一起带上**，否则被 vendor 的树无法编译——
即使固件根本不会引用它。这在 S0 阶段实际踩到过，`VENDORED.md` 里也写了原因。

## 8. 算法层迁移计划

现有 C 算法库（共 35 个头文件，其中 filter / control / image / math 27 个）**尚未迁移**。

### 8.1 它当前不可编译

这不是推测，是实测结论。任何 `#include "breeze/breeze.h"` 的文件都编译失败：

```text
include/breeze/comm/comm_buffer.h:30:3: error: conflicting types for 'BreezeCommBuffer'
include/breeze/math/interpolation.h:241:13: error: static declaration of
        'BreezeSplineInterpolation_Free' follows non-static declaration
```

`breeze.h` 聚合了 63 个头文件，其中两个有上述缺陷，因此 `examples/*.c`（5 个）与
`applications/` 下的两个应用**全部无法编译**——尽管它们各自的 README 曾写着
`gcc xxx.c -o xxx -lm` 可以直接构建。这些 README 已按实际情况更正。

唯一能构建并通过的是 `tests/comm/test_comm_interface.c`（链接
`include/breeze/core/globals.c`）：11/11 测试、42/42 断言通过。

### 8.2 迁移时的硬性约束

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
* **临界区不可嵌套**。内核自身从不嵌套（只有 `events.zig` 两处平铺的 enter/exit），
  但 HAL 契约是 `fn () void`，没有地方保存上一层的中断状态。Cortex-M 后端的
  `cpsid i` / `cpsie i` 是无条件置位/清位，因此嵌套时内层 `criticalExit` 会提前开中断。
  需要嵌套的调用方请改用 `hal.criticalEnterSaved()` / `hal.criticalExitRestore(saved)`。
  主机后端用深度计数器建模，两个后端在这一点上行为不同。
* `Program` 只表达"步骤序列 + 条件跳转"，**无法表达数据依赖的循环边界**（例如"直到数组耗尽"）。
  这类逻辑仍需写成 `call_until` 谓词或普通函数。
* 单个任务不让出会独占循环；用 `worstLateness()` / `totalResyncs()` 检测。
* 遥测帧的 CRC 多项式未与 LibXR 源码核对，见 [FUSION.md](FUSION.md) §9。



