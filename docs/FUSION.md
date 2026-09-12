# 融合方案：Breeze × LibXR/XRobot × Smartcar-Template

> 本文档回答一个问题：把 XRobot/LibXR 的设计经验、Breeze 的确定性内核、以及
> Smartcar-Template 的生成器工作流结合成一个方案，应该长什么样。
>
> 相关文档：[ARCHITECTURE.md](ARCHITECTURE.md)（内核设计）· [LIBXR-XROBOT.md](LIBXR-XROBOT.md)（调研记录）
>
> 文中所有"实测"结论都是在 Zig 0.16.0 + 本机复现的，不是推断。XRobot/LibXR 的事实来自
> [LIBXR-XROBOT.md](LIBXR-XROBOT.md)（对官方文档与 Doxygen 源码浏览器逐页核对）。

---

## 1. 三个项目各是什么

| | **Breeze** | **LibXR / XRobot** | **Smartcar-Template** |
|---|---|---|---|
| 定位 | 裸机确定性任务内核 | 机器人模块化框架 | 智能车竞赛模板生成器 |
| 语言 | Zig（freestanding） | C++20，必须编译 | C 或 Zig（`zig cc`） |
| 调度 | comptime 任务表 + 周期栅格 | **无自己的抢占调度**（`MonitorAll` 是 1000ms 监督循环） | cyt2bl3 有槽位调度器；另两芯片没有 |
| 内存 | 零堆、零任务栈 | 运行期 `SPSCQueue`/`Callback::Create` 会堆分配 | 无约束 |
| 规模 | 融合固件 1948 B flash / 436 B RAM（实测） | 完整框架 | 完整 SDK + 逐飞库 |
| 许可 | **MIT** | Apache-2.0 | GPL-3.0（逐飞库约束） |
| 目标 | Cortex-M / RISC-V | STM32/ESP32/HPM/CH32/MSPM0/Linux/… | CYT2BL3 / CYT4BB7 / RT1064 |

**三者不是竞争关系。** LibXR 解决"模块怎么组织和复用"，Breeze 解决"时间怎么确定性分配"，
Smartcar-Template 解决"工程怎么生成到学生手里"。三者恰好互补。

许可上需要注意：Breeze 采用 MIT，而逐飞库是 GPL-3.0。**把 Breeze 源码直接放进
Smartcar-Template 生成的工程里分发，需要按 GPL 兼容方式处理**，见 §9。

---

## 2. 关键发现：设计哲学已经高度重合

读完 LibXR 的设计文档后最值得注意的是——**它和 Breeze 独立收敛到了同一组结论**：

| LibXR 的明确主张 | Breeze 的对应实现 |
|---|---|
| "嵌入式系统中，运行时的内存分配是设计缺陷" | 零堆；`Channel` 用调用者提供的缓冲 |
| "一切回调/中断都必须是无阻塞的" | ISR 只能做三件事：推进时基、塞字节、置标志 |
| "上下文（thread/isr）必须在回调中显式传递" | `setFromIsr` vs `raise`，命名即契约 |
| "无锁数据结构与 ISR 驱动的数据流" | `Channel` 是无锁 SPSC；ISR 只交接，任务展开 |
| "接口中不应出现任何平台相关类型" | HAL 只有 3 个函数，且是 comptime 类型参数而非 vtable |
| "任何 I/O 操作都必须绑定确定的完成行为" | ← **这条 Breeze 原来没有，是本方案的补强点** |

这不是巧合：两边面对的都是"单核 MCU 上，中断和主循环共享状态"这一个物理事实。
**结论是：可以放心地把 LibXR 的架构当作已被验证的设计输入，而不必引入它的运行时。**

---

## 3. 融合的核心判断

### 3.1 取架构，不取运行时

LibXR 的 C++20 运行时（模板、RBTree、`std::atomic`、构造期堆分配）会和 Breeze 的
立身之本直接冲突。而且它已经预期要编译进来：

- `SPSCQueue` 在构造函数里 `::operator new[]`，不接受调用者缓冲；
- `Callback::Create` **会堆分配且没人释放**（官方文档自己承认）；
- Topic 靠运行时字符串 + RBTree 注册表解析。

**所以方案是：把 LibXR 的架构模式用 Zig 的 comptime 重新实现，而不是链入 LibXR。**
本方案的前三项（`chan.zig` / `topic.zig` / `app.zig`）就是这三条的具体落地，已实现并测试。

### 3.2 Zig comptime 在每一处都优于运行期注册表

| LibXR 做法 | Breeze 做法 | 收益 |
|---|---|---|
| `Topic::FindOrCreate(name, domain)` 查 RBTree | `Topic("name", T)` 在编译期算 CRC32 | 零 RAM、零查找、零锁；拼错的名字编译不过 |
| YAML manifest 写在 C++ 注释里，Python 解析 | `Manifest` 是声明式结构体，`App` 编译期校验 | 没有需要同步的第二份真相 |
| 生成 `XRobotMain` 装配模块 | `App(decls)` 生成任务表 | 无需 codegen 步骤 |
| `Callback::Create` 堆分配 | 指令表里存 `*const fn` | 无分配、无生命周期问题 |

**这不是"重造轮子"，而是"同一设计在更强的类型系统里实现"。**

### 3.3 反过来，Breeze 补上 XRobot 最缺的一块

LibXR 文档明确说它没有抢占调度器；`ApplicationManager::MonitorAll()` 默认 1000 ms 睡一次，
是监督循环而非控制循环。对智能车来说这远远不够——车需要 1 kHz 控制环、50 Hz 摄像头、
10 Hz 遥测同时跑且时序可推理。**这正是 Breeze 的存在理由。**

---

## 4. 已落地的融合（代码）

三项新增模块均已实现、测试并通过三款目标芯片的交叉编译。

### 4.1 `Channel` —— 调用者提供存储的无锁 SPSC

`src/kernel/chan.zig`。对应 LibXR 的 `SPSCQueue` / `Pipe`，但：

```zig
var uart_rx_buf: [64]u8 = undefined;            // RAM 成本写在声明处
var uart_rx = breeze.CountedChannel(u8, 63).init(&uart_rx_buf);
```

- 容量必须是 `2^n - 1`，索引回绕用掩码而非取模（M0 无硬件除法）；
- 满时**返回 false 而不是覆盖**；`CountedChannel` 记录丢弃计数供遥测上报；
- 用 `shared.zig` 的 volatile 纪律而非 `@atomicLoad`（原因见 §5）。

### 4.2 `Topic` —— 编译期主题 + LibXR 兼容的遥测帧

`src/kernel/topic.zig`。线格式与 LibXR 文档一致，因此**现有上位机工具可以直接解析**：

```text
偏移  长度  字段
   0    1  前缀 = 0x5A
   1    3  负载长度，小端 24 位
   4    4  主题名 CRC32，小端
   8    6  时间戳，微秒，小端 48 位
  14    1  版本 = 0x01
  15    1  0..14 字节的 CRC8
  16    N  负载（裸结构体字节）
16+N    1  0..15+N 字节的 CRC8
```

```zig
const Attitude = breeze.Topic("attitude", extern struct { roll: f32, pitch: f32, yaw: f32 });
// Attitude.id 在编译期折叠为 CRC32("attitude")，无注册表
```

> **兼容性注意**：字段布局来自 LibXR 文档，但**文档没有给出 CRC 多项式**。本实现用
> CRC-32/ISO-HDLC（`0xEDB88320`）与 CRC-8/ATM（`0x07`）。若要与真实 LibXR 上位机逐字节互通，
> 需对照 LibXR 源码复核这两处；测试已把当前行为钉死，改动必然是显式的。

### 4.3 `App` —— 编译期模块组合

`src/app.zig`。对应 XRobot 的 manifest + 生成 `XRobotMain`：

```zig
const App = breeze.AppWithHardware(.{
    .{ .module = Boot,    .state = &boot_state,    .config = .{ .timeout_ms = 250 } },
    .{ .module = Imu,     .state = &imu_state,     .config = .{ .alpha = 0.3 } },
    .{ .module = Chassis, .state = &chassis_state, .config = .{ .counts_per_meter = 1850.0 } },
    .{ .module = Uplink,  .state = &uplink_state },
}, &.{ "i2c0", "uart0", "encoder_l", "encoder_r", "motor_l", "motor_r" });
```

编译期检查：
- `depends` 中的每个名字都必须被别的模块 `provides`；
- `hardware` 中的每个名字都必须在应用声明的硬件表里；
- 同一名字不能被两个模块同时 `provides`；
- 实例名不能重复；
- `config` 字面量里的字段必须真实存在（拼错的 `.bais = 5` 会编译失败）。

**`depends` 与 `hardware` 必须分开**——这是实现过程中被编译器逼出来的结论：一开始只有一个
`requires` 列表，写示例时立刻报错"Imu 需要 i2c0，但没有模块提供它"。把外设包装成假模块只为
通过检查是纯粹的浪费。XRobot 用 `depends` / `required_hardware` 两条线划了同样的界线。

---

## 5. 融合过程中发现并修复的真实缺陷

这一节是本次融合最有价值的部分：**为了对接 Smartcar 的真实构建参数，暴露出 Breeze 一个会导致链接失败的错误。**

### 5.1 ARMv6-M 上原子操作会引入 libatomic

Smartcar 的 `build_zig.ps1` 用 `-fno-compiler-rt` 编译 Zig 对象。而 CYT4BB7 有一颗 **Cortex-M0+** 核。
实测（Zig 0.16.0，`thumb-freestanding-eabi -mcpu cortex_m0plus`）：

| 写法 | 外部依赖 |
|---|---|
| `@atomicLoad(.monotonic)` | `__atomic_load_4` |
| `@atomicStore(.monotonic)` | `__atomic_store_4` |
| `.acquire` / `.release` | 同上 |
| `.seq_cst` | 同上 |
| **`volatile` 访问** | **无** |

ARMv6-M 没有 `LDREX`/`STREX`，LLVM 因此不认为任何 32 位原子是无锁的，改为发库调用。
`-fno-compiler-rt` 下这些符号无处可寻 → **链接失败**。

进一步实测确认 `tools/picolibc/{cm0p,cm4f,cm7}/lib/libc.a` **都不含 `__atomic_load_4`**
（含 `__aeabi_memclr4`/`__aeabi_fadd` 等，但不含原子助手）。

**修复**：新增 `src/kernel/shared.zig`，改用 volatile 访问纪律，并写明为什么这是充分的：
单核上 ISR 与调度器不会同时执行；32 位对齐访问不会撕裂；每个共享字只有一个写者。
修复后四个目标对象的未定义符号全为空。

> 顺带一提：`generic_rv32` 同样需要原子助手而 `baseline_rv32` 不需要——也就是说
> "能不能链接"取决于 `-mcpu`，这种脆弱性本身就是不该依赖原子的理由。

### 5.2 其它在融合中修掉的

- **CRC-8 实现错误**：在 `u8` 上直接 `<< 1`，丢位行为与 Zig 语义不符，多项式除法算错。
  校验值测试（`"123456789"` → `0xF4`）当场抓出。
- **主机测试的时序语义**：`runFor` 原本先推进时钟再跑循环，导致首次激活落在 t=1、
  产生 1 ms 虚假延迟、10 ms 任务在 1 秒内跑 101 次。改为"先跑后推进"，与真实 systick 一致。

---

## 6. 实测数据

### 6.1 融合固件（`examples/firmware_smartcar.zig`：4 个模块 + 3 个 topic + 2 个 channel）

| 目标 | Flash | RAM | 外部依赖 |
|---|---|---|---|
| CYT2BL3 (Cortex-M4F) | 1948 B | 436 B | `__aeabi_memclr4` |
| CYT4BB7 CM0+ | 1932 B | 436 B | `__aeabi_memclr4` + 软浮点 3 处 + 64 位整数 1 处 |
| CYT4BB7 CM7F / RT1064 | 1952 B | 436 B | `__aeabi_memclr4` |

全部由 picolibc 提供，**无一来自 libatomic**。（尺寸与依赖列在 2026-09 重新测量过；
办法是把产物当字节串搜符号名，能查出"引用了什么"，查不出别的。RAM 一列含 `.data`
在 RAM 中的拷贝，见 ARCHITECTURE §4 对 `tools/elfsize.ps1` 口径修正的说明。）

### 6.2 内核本体

| 项 | 数值 |
|---|---|
| `Program` 状态 | 12 B，与指令条数无关 |
| `TaskState` | 28 B |
| `Join` / `Limiter` | 4 B / 12 B |
| `Join.Of(T, cap)` | `cap` + `cap×sizeof(T)` + 1 B |
| 每任务额外栈 | 0 |
| `firmware_cortex_m.zig`（Cortex-M0） | 628 B flash / 184 B RAM |
| `firmware_riscv.zig`（RISC-V32） | 868 B flash / 200 B RAM |
| 调度抖动 | 0 tick |

Cortex-M0 骨架的 184 B RAM 中 76 B 是示例演示用的 UART 接收环，56 B 是两个 `TaskState`，
28 B 是任务上下文，其余 24 B 是事件标志与调度器自身状态。**以上数值与 README 的
「实测开销」表是同一批测量**；两张表曾经各自漂移过，改动内核后请一并重测。

### 6.3 测试

`zig build ci`：单元测试与目标交叉编译全部通过（含三款 Smartcar 芯片，
每颗都有具名目标）。数量见 [README](../README.md) 状态行。

---

## 7. 落地到 Smartcar-Template

### 7.1 现状与问题

Smartcar-Template 的 Zig 路径已经存在，但三款芯片严重不对称：

| 芯片 | Zig 文件数 | 调度器 | 驱动层 |
|---|---|---|---|
| cyt2bl3 | 20 | 有（槽位式） | 完整（gpio/uart/spi/i2c/adc/pwm/…） |
| cyt4bb7 | 1 | **无** | 无，只有 `extern fn` 裸声明 |
| rt1064 | 1 | **无** | 无 |

而 cyt2bl3 那个 `scheduler.zig` 本身有几个问题：

1. **忙等**：`while (self.started) { ... }` 没有任何空闲处理，无任务就绪时 100% 占 CPU；
2. **周期漂移**：`last_run = current_tick` 导致误差累积，而 Breeze 用 `next_run += period` 固定栅格；
3. **无事件**：只有 `period_ms`，无法表达"等串口中断"；
4. **RAM 浪费**：`[16]?Task` 常驻约 384 B，Breeze 同功能 48 B；
5. **双重循环**：`zig_loop()` 调用永不返回的 `scheduler.run()`，外面 `main_zig.c` 的 `for(;;)` 形同虚设；
6. `TaskState` 枚举被赋值但从未被读取。

### 7.2 替换方案

把 `project/zig/scheduler.zig` 换成 Breeze，`user.zig` 改成模块声明：

```
project/zig/
  breeze/             ← 新增：Breeze 内核（chan/topic/app/hal）
  modules/            ← 新增：每个模块一个文件，含 Manifest
    imu.zig
    chassis.zig
    uplink.zig
    boot.zig
  board.zig           ← 新增：HAL 实现（tick 来自 PIT，临界区来自 zf_common_interrupt）
  user.zig            ← 消失：被 App 声明取代
```

改动面比看上去小：`build_zig.ps1` 只需把入口从 `user.zig` 换成 `main.zig`，
`main_zig.c` 与 `zig_bridge.h` 的 `zig_setup()` / `zig_loop()` 契约**完全不变**，
`Makefile` 的 `zig` target 与链接行也**完全不变**。

HAL 适配只有三个必需函数加一个可选钩子。注意 `criticalEnter`/`criticalExit` 必须成对，
且**内核自身不嵌套临界区**（见 ARCHITECTURE.md §9），所以保存一个 PRIMASK 副本即可：

```zig
var saved_primask: u32 = 0;

pub const BoardHal = struct {
    pub fn now() Tick {
        return sf.pit.getMillis();
    }
    pub fn criticalEnter() void {
        saved_primask = sf.interrupt.globalDisable();
    }
    pub fn criticalExit() void {
        sf.interrupt.globalEnable(saved_primask);
    }
    /// 可选：调度器空转时调用。
    pub fn idle() void {
        // WFI，或留空
    }
};
```

### 7.3 对生成器的要求

`docs/index.html` 的模板矩阵**不需要新增维度**：Zig 路径本就存在，只是内容换了。
但 manifest 生成时应把 `App` 的模块图导出，便于未来扩展。

### 7.4 分阶段

| 阶段 | 内容 | 验收 | 状态 |
|---|---|---|---|
| **S0** | 把 Breeze 作为 `project/zig/breeze/` 引入 cyt2bl3，替换 scheduler | 现有 LED/串口示例行为不变；RAM 下降可量化 | ✅ **已完成**，见 §7.5 |
| **S1** | 用 `App` 重写 cyt2bl3 示例为 3 个模块 | 编译通过；`App.describe()` 输出正确模块图 | ✅ **已完成**，见 §7.8 |
| **S2** | 补 cyt4bb7 / rt1064 的 HAL 与 Zig 驱动层 | 三芯片 Zig 路径对称 | 待做 |
| **S3** | 遥测：接一个 topic 到串口，上位机用 LibXR 帧格式解析 | 上位机能看到实时曲线 | 待做 |
| **S4** | 把 XRobot 式模块清单导出为文档，供学生阅读 | 生成 README 中的模块表 | 待做 |

### 7.5 S0 实测结果

分支 `feat/breeze-kernel-s0`。同一份固件、同一组任务、`-OReleaseSmall`：

| | 原槽位调度器 | Breeze | 变化 |
|---|---|---|---|
| Zig 对象 RAM | 404 B | **72 B** | −332 B（−82%） |
| Zig 对象 Flash | 944 B | 811 B | −133 B |
| 整机 `.bss` | 3624 B | **3288 B** | −336 B |
| 整机 Flash | 153 952 B | 153 776 B | −176 B |
| Zig 对象中的 `wfi` | 0 处 | 2 处 | 空闲不再忙等 |

Flash 那一列对 Breeze 偏保守，但只在**对象文件**那一行：Breeze 还额外带了两个诊断导出
（`zig_worst_lateness`、`zig_grid_resyncs`），原版本没有对应功能。整机那一行不受影响——
S1 期间实测确认，链接脚本只 `KEEP` 了 `.intvec` / `.init` / `.fini`，这两个无人调用的
导出会被 `--gc-sections` 回收，最终镜像里根本不存在（用 `zig objdump` 与二进制串搜索
都查不到）。这条已在 §7.9 一并记录。

原调度器占的 400 B 就是 `[16]?Task` 常驻数组。Breeze 三个任务的调度状态名义上是 84 B
（`TaskState` 24 B × 3 + 事件标志 4 + 两个计数器 8），实际只发出 72 B，
因为 `TaskState.runs` 在固件里只写不读，被优化器删掉了（3 × 4 = 12 B，正好是差额）。

**验证深度**：`scripts/build_zig.ps1` 出对象 → 完整 `make` 链接出
`cyt2bl3.elf/.hex/.bin` → 最终镜像里有 `zig_setup`/`zig_loop`/`pit0_isr_callback`/
`zig_millis`、没有 `global_scheduler`。

**C 侧零改动**：`main_zig.c`、`zig_bridge.h`、`cm4_isr.c` 一行未动，
导出契约完全不变。

### 7.6 S0 过程中发现的模板既有问题

都在改之前就存在，与本方案无关：

| 问题 | 影响 | 处理 |
|---|---|---|
| `coremark_port.c` include 了不存在的 `main.h` | **整机无法链接**（`'main.h' file not found`） | 已修：改为 `coremark.h`（`ee_u32` 的真正来源，且 `-I../../../tools` 已在包含路径上） |
| 同样的 include 缺陷存在于 cyt4bb7 / rt1064 的副本 | 那两颗芯片同样无法链接 | **未改**：本环境无法构建验证，只报告 |
| `make` 试图重建 picolibc，但 `picolibc-src` 已被 commit ca13264 删除 | 无法重建；因为 `build_picolibc.ps1` 的 mtime 比预编译的 `libc.a` 新，每次都会触发 | 未改（属设计决策）；验证时用 `make -o <libc.a>` 绕过，纯净树同样失败，已确认与本次改动无关 |

### 7.7 供后续复用的机制

Breeze 侧新增 `tools/vendor.ps1`：把内核复制进任意仓库，并写出 `VENDORED.md`
（记录来源 commit 与每个文件的 sha256）。`-Check` 模式检测两边是否漂移。
这样 S2/S3 往另两颗芯片复制时不需要手工挑文件。

有一个 Zig 行为值得记下：**`test` 块里的 `@import` 在 `build-obj` 时也会被解析**。
内核的几个文件在测试夹具里 import `../hal/host.zig`，所以 vendor 时必须把它一起带上，
否则被 vendor 的树无法编译。这条是用"故意 import 一个不存在的文件"实测确认的。

### 7.8 S1 实测结果

分支 `feat/app-modules-s1`。三个任务改写成 `modules/{blink,status,telemetry}.zig`，
装配表变成 `app.zig` 里的一份 `AppWithHardware` 声明。

**模块图**（编译期算好，`describe()` 打出来的字面就是这些）：

```
application: 3 module(s)
  [0] blink0 (Blink)  period=200ms
        Toggles the status LED and counts every edge
        hardware: led
        provides: blink
  [1] status0 (Status)  period=1000ms
        Prints a one-line link report once a second
        depends: blink
        hardware: uart0
        publishes: status
  [2] telemetry0 (Telemetry)  period=100ms
        Emits a sample frame for the host to plot
        hardware: uart0
        publishes: telemetry
```

**整机占用**（`-OReleaseSmall`，用新增的 `tools/elfsize.ps1` 按 section 统计）：

| | S0（手写任务表） | S1（`describe` 关） | S1（`describe` 开） |
|---|---|---|---|
| `.text` | — | 145 040 | 147 312 |
| `.bss` | 3 288 | 3 296 | 3 296 |
| Flash 合计 | 153 952 | **152 128** | 154 400 |
| BIN | 153 952 | 153 792 | 156 064 |

两个结论：

1. **换成模块化装配本身不增加开销**，反而比 S0 少 160 B。`Blink` 的边沿计数占 4 B，
   但省掉了任务表里冗余的东西。
2. **开机打印模块图值约 2.3 KB flash**（`.text` 增量 147 312 − 145 040 = 2 272 B，
   全部来自 `std.fmt` 的整数格式化）。对一颗 152 KB 的镜像这不是小数目，因此
   `app.zig` 里留了 `print_manifest_at_boot` 开关。它是 `comptime` 分支，关掉时
   那段代码**根本不会被分析**，代价是精确的零而不是"很小"。默认开启，因为教学价值
   高于 2.3 KB。

**主机测试**：`zig test project/zig/app_test.zig` → **69 passed**。其中 8 个是 S1 新写的，
跑的是**固件同一份模块代码**——模块把 I/O 面作为 comptime 类型参数接收，换成 `FakeIo`
就能在主机上执行。验证内容包括：三者的实际激活次数（5 / 1 / 10）、`describe()` 的逐行
输出、以及"依赖确实在传数据"——`Status` 在 t=0 那一轮读到 `Blink` 刚产生的第 1 个边沿，
证明 `Config` 里那个指针不是装饰。

这也正是 Breeze 相对 XRobot 的关键差别：XRobot 需要一份 YAML 加一个 Python 生成器才能
拿到模块图，Breeze 的模块图就是源码，而且能在主机上直接跑。

### 7.9 S1 暴露的框架缺口

`App` 原有的 `coerceConfig` 无法表达"必填的 Config 字段"：它用 `var cfg: Config = .{}`
构造，于是每个字段都必须有默认值。而 `Status` 恰恰需要一个没有默认值的
`blink: *Blink.State` 来把接线变成编译期强制。

已改为 `undefined` 构造 + 显式检查无默认值字段，漏填时给出点名报错：

```
Status.Config field 'blink' is required and was not set in the instance's `config` literal
```

这是 S1 第一次真正使用 `App` 才发现的缺口——之前 `app.zig` 的测试全都用带默认值的
Config，覆盖不到。Breeze 侧补了两个测试锁住它。

另一个改动在模板侧：`$(ZIG_OUT)` 原来只依赖 `project/zig/*.zig`，看不到新加的
`modules/` 子目录，改了模块不会触发重新编译、固件会静默保留旧代码。三个 Makefile
都已改成 `project/zig/*.zig project/zig/*/*.zig`。

### 7.10 `--gc-sections` 会吃掉无调用者的导出

S1 验证镜像内容时发现的：`zig_worst_lateness` 和 `zig_grid_resyncs` 是 `pub export fn`，
但没有任何 C 侧代码调用它们，于是链接时被回收，镜像里既没有符号也没有代码。

这修正了 §7.5 的一处不准确表述。它和 S0 记录过的 `breeze_board_wfi` 是同一个现象，
但方向相反，值得一起记住：

* **`export fn` 保证被汇编**——Zig 的惰性分析不会跳过它，所以内联汇编里的笔误能在
  编译期暴露。这是 S0 用 `export` 替换 `comptime { _ = &f; }` 的原因。
* **`export fn` 不保证进镜像**——只要没人调用、链接脚本又没 `KEEP`，它就只是一段
  被丢弃的死代码。

所以"想确认某段代码真的被编译过"用 `export`，"想确认它真的在固件里"必须查最终镜像。
这两件事不能互相替代。

顺带记下另一个字符串相关的坑：`describe()` 的输出在镜像里是**碎片化**的，
`depends:` 与 ` blink` 分属格式串和 manifest 数组，并不连续。想从二进制里确认
模块图在不在，要么搜各个片段，要么直接信主机测试对**渲染结果**的断言——后者才是
权威检查。

---

## 8. 明确不做的部分

诚实地列出边界，比罗列能力更重要：

| 不采纳 | 原因 |
|---|---|
| 链入 LibXR 运行时 | 会引入 C++20、构造期堆分配、RBTree 注册表，摧毁 Breeze 的 60 B 内核与主机可测性 |
| `Operation::BLOCK`（信号量阻塞） | Breeze 无阻塞语义。等价能力由 `Program` 的 `wait_event_timeout` 提供，且**不需要信号量、不需要线程** |
| `in_isr` 运行时显式传参 | LibXR 需要它是因为它的回调可能在两种上下文被调用。Breeze 通过**接口分离**（`setFromIsr` / `raise`）在编译期解决同一问题，比运行时标志更强 |
| 运行期 topic 注册表 | comptime topic 覆盖同样需求，且零 RAM |
| 抢占式调度 | 明确不做，保持时序可静态推理。需要硬实时就把 Breeze 放进一个 RTOS 任务里 |

### 关于 `Operation` 的取舍

LibXR 的 `Operation` 模型（发起时绑定完成行为：CALLBACK / BLOCK / POLLING / NONE）
是它最值得学的一条，但 Breeze 的实现方式不同：

| 完成方式 | LibXR | Breeze |
|---|---|---|
| 忽略结果 | `Operation()` | 不关心返回值即可 |
| 回调 | `Operation(Callback&)`，会堆分配 | 指令表里的 `*const fn`，零分配 |
| 轮询 | `OperationPollingStatus&` | `Program` 的 `call_until` |
| 阻塞等待 | `Operation(Semaphore&, timeout)` | **`wait_event_timeout` 指令**——顺序代码，无栈无锁 |

最后一行是 Breeze 相对 LibXR 的实质优势：它把"等一个异步操作完成"从**阻塞原语**
变成了**可组合的顺序代码**，这正是 protothread 类方案的价值所在。

---

## 9. 风险与限制

| 风险 | 影响 | 对策 |
|---|---|---|
| Zig 与 C++ 不能混用同一套模块 | 只能二选一，不能"都要" | Smartcar 的 `language` 轴本就是互斥的；本方案是把 Zig 路径做强，不是合并 |
| 学生 Zig 学习曲线 | 中 | Zig 语法比 C++20 模板少得多；且 `App` 的声明式写法比手写状态机更接近自然语言 |
| `Program` 无法表达任意控制流 | 中 | 数据依赖的循环边界仍需手写；已在 ARCHITECTURE.md §9 明说 |
| **MIT 与 GPL-3.0 的许可冲突** | 高 | 逐飞库是 GPL-3.0。Breeze 是 MIT，两者**不冲突但不可单向合并**：MIT 代码可以放进 GPL 工程（GPL 兼容），但反过来不行。若要把 Breeze 源码直接 vendored 进 Smartcar-Template 生成的工程并整体分发，该分发物整体按 GPL-3.0 处理，需保留逐飞库声明。另一种做法是生成器只引用 Breeze 的发布版而不 vendored 源码 |
| 三芯片 SDK 差异 | 高 | S2 阶段的工作量主要在这里，与 Breeze 无关 |
| CRC 多项式未与 LibXR 源码核对 | 低 | 若不需上位机互通则无影响；需要时按 §4.2 复核 |

---

## 10. 结论

三个项目的正确关系是：

- **Smartcar-Template** 负责把工程交付到学生手里（生成器、三芯片 SDK、逐飞库、OpenOCD）；
- **Breeze** 负责让这些工程的时间行为确定、可测、可推理（comptime 任务表、零栈协程化顺序逻辑、主机虚拟时钟测试）；
- **LibXR/XRobot** 提供已经被验证过的架构答案（无锁 ISR 数据流、显式上下文、完成行为绑定、模块清单、遥测线格式），**以设计输入而非依赖的形式采纳**。

融合的产物不是"更大的框架"，而是：**一个 1.4 KB 的固件，里面有可组合的模块、
可推理的时序、可被上位机解析的遥测，并且整套内核能在开发机上用虚拟时钟跑完。**

---

## 附：本次融合产出的代码

| 文件 | 内容 |
|---|---|
| `src/kernel/shared.zig` | ISR 共享内存访问纪律（修复 ARMv6-M 链接问题） |
| `src/kernel/chan.zig` | 调用者提供存储的无锁 SPSC channel |
| `src/kernel/topic.zig` | comptime 主题 + LibXR 兼容遥测帧 + CRC |
| `src/app.zig` | 编译期模块组合与依赖校验 |
| `examples/firmware_smartcar.zig` | 四模块融合固件示例（三芯片编译验证） |
| `tools/elfsize.ps1` | flash/RAM 占用测量工具 |
| [LIBXR-XROBOT.md](LIBXR-XROBOT.md) | LibXR/XRobot 技术调研报告 |

---

## 11. 与 zig-orch 的关系

`zig-orch` 是同一作者的另一个协作式编排内核，面向"主循环 + 偶发耗时任务"
场景（帧预算、片轮转、三级优先级、动态 spawn、可取消句柄、mquickjs 的 VM
抢占接缝）。两边解决的问题相邻，因此评估过是否合并。结论是**在接缝处融合，
不合并内核**。

### 取了什么

`Join`（等待 N 个分支归位）与 `Limiter`（限制在飞窗口）搬进了
`src/kernel/orchestrate.zig`。两者都不依赖调度器——它们只是"带规则的计数器"，
所以可以从任务、ISR 或主机测试里用，而 Breeze 的调度器不需要为它们长出一个
特性。它们与 `Program` 的配合是自然的：序列通过 `call_until` 等一个 `Join`。

`zig-orch` 的 ROADMAP 里，"`Join` 没有结果槽"是最高优先级的缺口。Breeze 这边补上了：
`Join.Of(T, cap)` 给每个分支一个**具名**槽位，分支从 ISR 里汇报时带上自己产出的值，
等待者按名字取回。槽位编号在编译期固定，因此每个分支写自己的那个字——没有共享计数器，
也就没有读-改-写；这是它与 `Join` 的关键区别，也是并发汇报用不着临界区的原因。
上限 32 个槽位：再多就不该用槽位，而该用 `Limiter` 加一个自己的数组。

顺带补上的还有一个 `Join` 做不到的动作：**把"还没汇报的分支"标记为失败**。
融合固件的 `Boot` 就是这样实现超时的——超时不是等一个事件标志，而是把仍在
`.running` 的槽位逐个 `fail`，然后照常走重试分支。计数式的 `Join` 无法表达这件事，
因为它不知道该失败哪一个。

第三个是**超预算检测**，但不是照搬。`zig-orch` 在 `Hooks` 里声明了
`on_overrun` 却从未调用它，`defers` 计数器也只增不读——是个死钩子。Breeze
这边把它真正接上了：任务可声明 `.max_late_ms`，`TaskState.overruns` 计数，
`setOverrunHook` 提供推送通知，另有 `observesOverruns()` 用来区分"没有超预算"
和"根本没在观察"。

### 没取什么，以及为什么

| 不合并 | 原因 |
|---|---|
| 运行期 slot 池 + epoch 句柄 | Breeze 的 comptime 任务表是承重墙：`App`/`Manifest` 的模块图、零 RAM 任务表、声明顺序的确定性都建在它上面。整体替换等于用 zig-orch 取代 Breeze，不是融合 |
| 帧预算 + 片轮转调度 | 与"固定周期栅格、时序可静态推理"冲突；两者优化目标不同 |
| 三级优先级 | 同上。Breeze 用声明顺序表达依赖优先级，且是可静态审阅的 |
| `Channel` | Breeze 已有一份，且在 ARMv6-M 上能链接（见下） |
| `Tick = u64` | Breeze 用 u32 + 有符号差值比较，构造上抗回绕，在 32 位目标上还省一半空间 |

### 一个共同的硬约束：ARMv6-M 上没有原子 RMW

两边都独立撞上了同一件事，所以值得记在这里。在
`thumb-freestanding-eabi -mcpu cortex_m0plus` 上实测：

| `std.atomic.Value(u32)` 操作 | 结果 |
|---|---|
| `load` / `store` | 内联，无依赖 |
| `swap` / `fetchOr` / `fetchAdd` / `cmpxchg` | `__atomic_exchange_4` / `_fetch_or_4` / `_fetch_add_4` / `_compare_exchange_4` 库调用 |

而 `thumb-freestanding` 不提供这些符号——**即使 Zig 自带的 compiler_rt 也不提供**，
实测链接失败（`-fno-compiler-rt` 只是让它更早失败）。所以问题只在
read-modify-write 上，不是"原子操作都不能用"。

Breeze 的答案是 `src/kernel/shared.zig` 的 volatile 纪律（`Scheduler` 与
`EventFlags` 在 M0+ 上的对象里 `__atomic` 引用数为零）。zig-orch 在评估后采用了
同一手法：`Channel` 改为 volatile（每个索引只有一个写者），`takeDeferred` 的
读-清改为临界区。

顺带发现 `zig-orch` 的 `zig build check-freestanding` 是空转的：它用
`addObject` 编 `orch.zig`，而 Zig 对泛型惰性分析、没有调用点就一行都不分析
（产物 516 B），且对象不解析符号。即使改成 `addExecutable`，Zig 对没有下游
消费者的 artifact 会传 `-fno-emit-bin`，**链接根本不执行**。要让它真的有牙齿，
必须安装产物以强制 emit。这一点是实测出来的：把回归代码放回去之后，那一步
依然报 success。


## 附：致谢与许可

- **Breeze** 采用 [MIT 许可](../LICENSE)。
- 本项目采纳了 [LibXR](https://github.com/Jiu-xiao/libxr) 与
  [XRobot](https://github.com/xrobot-org) 的**架构设计思想**（Apache-2.0）。
  架构与设计思路不受版权保护，但来源应予说明，故在此致谢。
  Breeze **未包含、未链接 LibXR 的任何源代码**，因此不构成 Apache-2.0 的再分发义务。
- `include/`、`src/` 下的 C 算法库为同一 MIT 许可下的历史代码。
- 若把 Breeze vendored 进包含逐飞库（GPL-3.0）的工程分发，请按 §9 处理许可。
