# LibXR / XRobot 技术调研报告

> **本文档是外部资料的核对记录，不是 Breeze 的设计文档。**
> 相关文档：[ARCHITECTURE.md](ARCHITECTURE.md)（内核设计）· [FUSION.md](FUSION.md)（融合方案）
>
> **用途**：记录 LibXR / XRobot 的实际机制，为 [FUSION.md](FUSION.md) §3 的
> "取架构、不取运行时"判断提供依据。Breeze **未包含也未链接** LibXR 的任何源代码，
> 本文档只在设计层面引用其公开文档与开源实现。
>
> **状态**：调研于 2026-09，针对 LibXR `55c20b4`、XRobot `15bff04`、CodeGen `17b9f9e`。
> 上游是活跃项目，接口可能已变化；引用具体 API 前请复核。
>
> **为什么正文是英文**：本报告大量逐字引用上游的 API 名、源码片段与文档原文，
> 翻译会破坏引用的准确性。因此正文保留英文。未核对与文档缺失之处集中列在第 12 节，
> 不做推测性补齐。

Research target: the XRobot / LibXR embedded robotics framework.
Primary sources: <https://xrobot-org.github.io/> (Chinese + English), the generated Doxygen
API reference at <https://jiu-xiao.github.io/libxr/>, and raw GitHub sources.

**Provenance convention used throughout:**

- **[DOC]** — stated on a documentation page (URL given).
- **[SRC]** — read from actual source in the Doxygen reference or raw GitHub (URL given).
- **[INFERENCE]** — my own reasoning, not stated anywhere.

All fetched web content was treated as untrusted data. Where a page was thin, absent, or a
fetch failed, that is stated explicitly rather than filled in from general knowledge.

---

## 0. Project structure and identity

- **LibXR** is the C++ embedded framework (core, middleware, drivers, OS abstraction).
  Repo: <https://github.com/Jiu-xiao/libxr>. Licensed **Apache-2.0**.
- **XRobot** is the Python project/module manager + CLI toolchain.
  Repo: <https://github.com/xrobot-org/XRobot> (default branch `XRobot2.0`). Licensed **Apache-2.0**.
- **LibXR_CppCodeGenerator** is the Python code generator.
  Repo: <https://github.com/Jiu-xiao/LibXR_CppCodeGenerator>. Licensed **Apache-2.0**.

Stated lineage **[DOC]** (<https://xrobot-org.github.io/docs/about>): XRobot was formerly
`qdu-rm-mcu`, created 2019-01-19 by [@qsheeeeen], used as the electronics/control framework of
Qingdao University's RoboMaster team. Named current contributors: **@Jiu-xiao** (framework
designer; STM32 / CH32 / Linux driver development), **@molqzone** (MSPM0, DAPLink),
**@CaFeZn** (HPM series), **@llLeo306** (control & real-time systems).

---

## 1. The Operation model

Source of truth: `src/core/rw/operation.hpp`, read verbatim at
<https://jiu-xiao.github.io/libxr/operation_8hpp_source.html>.

### 1.1 What an Operation is

**[SRC]** A `Operation<T>` is a small tagged union describing *how the completer should report
back*. The whole class is ~200 lines. Verbatim:

```cpp
template <typename Args>
class Operation
{
 public:
  using Callback = LibXR::Callback<Args>;

  enum class OperationType : uint8_t
  {
    CALLBACK,
    BLOCK,
    POLLING,
    NONE
  };

  enum class OperationPollingStatus : uint8_t
  {
    READY,
    RUNNING,
    DONE,
    ERROR
  };

  Operation() : data{nullptr}, type(OperationType::NONE) {}

  Operation(Semaphore &sem, uint32_t timeout = UINT32_MAX)
      : data{.sem_info = {&sem, timeout}}, type(OperationType::BLOCK)
  {
  }

  Operation(Callback &callback)
      : data{.callback = &callback}, type(OperationType::CALLBACK)
  {
  }

  Operation(OperationPollingStatus &status)
      : data{.status = &status}, type(OperationType::POLLING)
  {
  }
  // copy ctor / move ctor / copy assign / move assign all defined
```

### 1.2 The exact enum / variant names

- `OperationType::CALLBACK` — completion delivered via callback
- `OperationType::BLOCK` — completion signalled via semaphore
- `OperationType::POLLING` — completion written into a status variable
- `OperationType::NONE` — completion ignored (this is also the default-constructed state)

- `OperationPollingStatus::READY`
- `OperationPollingStatus::RUNNING`
- `OperationPollingStatus::DONE`
- `OperationPollingStatus::ERROR`

Storage is a real C++ `union` **[SRC]**:

```cpp
  union
  {
    Callback* callback;
    struct
    {
      Semaphore* sem;
      uint32_t timeout;
    } sem_info;
    OperationPollingStatus* status;
  } data;

  OperationType type;
```

### 1.3 How completion is bound at initiation time

**[SRC]** Binding is by **constructor overload**, chosen at the call site — there is no
setter and no runtime mode switch after construction. You pick the constructor, pass the
`Operation` by reference into the port, and the port/ISR calls `UpdateStatus`.

**Note the timeout asymmetry:** the BLOCK constructor stores a `timeout` in `sem_info`, but
`UpdateStatus` never reads it. **[SRC]** The timeout is consumed by the *port*, not by
`Operation`.

### 1.4 How a driver reports completion

**[SRC]** Via one template method:

```cpp
  template <typename Status>
  void UpdateStatus(bool in_isr, Status&& status)
  {
    switch (type)
    {
      case OperationType::CALLBACK:
        data.callback->Run(in_isr, std::forward<Status>(status));
        break;
      case OperationType::BLOCK:
        // BLOCK waits are signaled by semaphore only; the owning port keeps the
        // final ErrorCode in its block_result_ handoff state.
        // BLOCK 只通过信号量唤醒；最终 ErrorCode 由端口侧 block_result_ 交接。
        data.sem_info.sem->PostFromCallback(in_isr);
        break;
      case OperationType::POLLING:
        *data.status = (status == ErrorCode::OK) ? OperationPollingStatus::DONE
                                                 : OperationPollingStatus::ERROR;
        break;
      case OperationType::NONE:
        break;
    }
  }

  void MarkAsRunning()
  {
    if (type == OperationType::POLLING)
    {
      *data.status = OperationPollingStatus::RUNNING;
    }
  }
```

**Three critical behaviours to design against:**

1. **BLOCK does not carry a result.** The semaphore is only a wakeup. The actual `ErrorCode`
   is kept by the *port* in its `block_result_` field. `Operation` itself never stores it.
   This is stated in the source comment above and confirmed by the docs **[DOC]**
   (<https://xrobot-org.github.io/docs/basic_coding/core/core-op>): "当前完成值本身不会通过
   `Operation` 内部保存给阻塞等待者，具体最终 `ErrorCode` 由拥有该 `Operation` 的端口侧
   handoff 状态保存."
2. **POLLING hard-codes `ErrorCode::OK` as the success test.** The docs are explicit that this
   makes POLLING only naturally correct for `Operation<ErrorCode>`: "当前实现直接按
   `status == ErrorCode::OK` 判断成功并置为 `DONE`... 若把它推广到其它 `T`，并不能自动得到一套
   独立于 `ErrorCode` 的通用成功判定语义." **[DOC]**
3. **`UpdateStatus` is a template**, so the status type is unconstrained at the call site —
   type correctness relies on the caller passing something convertible to `T`.

### 1.5 Type aliases and function-pointer contracts

**[SRC]**

```cpp
typedef Operation<ErrorCode> ReadOperation;
typedef Operation<ErrorCode> WriteOperation;

typedef ErrorCode (*WriteFun)(WritePort& port, bool in_isr);
typedef ErrorCode (*ReadFun)(ReadPort& port, bool in_isr);

typedef struct
{
  RawData data;
  ReadOperation op;
} ReadInfoBlock;

typedef struct
{
  ConstRawData data;
  WriteOperation op;
} WriteInfoBlock;
```

**This is the single most important integration fact.** The platform driver hooks
(`ReadFun` / `WriteFun`) do **not** receive the buffer or the `Operation` as arguments. They
receive only the *port* and an `in_isr` flag. The payload + operation live inside the port
(`ReadPort::info_` of type `ReadInfoBlock`). A new platform backend must therefore reach back
onto the port object to find out what was requested.

### 1.6 Doc code examples (verbatim)

From <https://xrobot-org.github.io/docs/basic_coding/core/core-op>:

```cpp
// 默认构造：类型为 NONE
Operation();
// 构造阻塞操作
Operation(Semaphore &sem, uint32_t timeout = UINT32_MAX);
// 构造回调操作（T 为回调参数类型）
Operation(Callback<T> &cb);
// 构造轮询操作
Operation(OperationPollingStatus &status);
```

```cpp
Semaphore sem;
WriteOperation op_block(sem, 100);
write_port(data, op_block);
```

```cpp
auto cb = Callback<ErrorCode>::Create([](bool in_isr, int context, ErrorCode ec) {
  // 回调处理逻辑
}, 123);  // 绑定 context 为 123
ReadOperation op_cb(cb);
read_port(buffer, op_cb);
```

```cpp
auto status = LibXR::ReadOperation::OperationPollingStatus::READY;
ReadOperation op_poll(status);
read_port(buffer, op_poll);
// 后续通过 status 查询是否完成
if (status == LibXR::ReadOperation::OperationPollingStatus::DONE) {
  // 成功完成
} else if (status == LibXR::ReadOperation::OperationPollingStatus::ERROR) {
  // 完成但发生错误
}
```

### 1.7 `AsyncBlockWait` — the BLOCK handoff helper

**[SRC]** Also defined in `operation.hpp`, class doc: *"Shared BLOCK waiter handoff for
synchronous driver operations."*

```cpp
class AsyncBlockWait
{
 public:
  // Keep the waiter state 32-bit wide so STM32 builds stay within the
  // project-wide atomic shim boundary.
  enum class State : uint32_t
  {
    IDLE = 0,
    PENDING = 1,
    CLAIMED = 2,
    DETACHED = 3,
  };

  void Start(Semaphore& sem);
  void Cancel();
  ErrorCode Wait(uint32_t timeout);
  bool TryPost(bool in_isr, ErrorCode ec);

 private:
  Semaphore* sem_ = nullptr;
  std::atomic<State> state_{State::IDLE};
  ErrorCode result_ = ErrorCode::OK;
};
```

Semantics **[SRC]** + **[DOC]**: on timeout the waiter is atomically moved to `DETACHED` so a
late completion cannot deliver a result to a caller that already returned; `TryPost` then sees
`DETACHED` and returns `false` after cleaning the in-flight state back to `IDLE`. This is the
concrete reason BLOCK does not attribute the `ErrorCode` to `Operation` itself.

---

## 2. Callback mechanism

Source: `src/core/libxr_cb.hpp` at <https://jiu-xiao.github.io/libxr/libxr__cb_8hpp_source.html>.

### 2.1 Signature

**[SRC]** The callback function signature is exactly:

```cpp
void(bool in_isr, ArgType arg, Args... args)
```

where `ArgType` is the **bound** first argument and `Args...` are the call-site arguments.

### 2.2 How type safety is achieved

**[SRC]** Via a C++20 `concept` that requires convertibility to an *exact* function pointer:

```cpp
template <typename CallableType, typename BoundArgType, typename... CallbackArgs>
concept CallbackFunctionCompatible = requires(CallableType callable) {
  static_cast<void (*)(bool, BoundArgType, CallbackArgs...)>(callable);
};
```

Doxygen's one-line description: *"可转换为精确回调函数指针的可调用对象"* (a callable
convertible to an exact callback function pointer). Practically this means only non-capturing
lambdas, plain functions and static member functions qualify — capturing lambdas do not.
**[DOC]** confirms: "需要**可转换为函数指针**（例如普通函数、静态成员函数、无捕获 lambda 等）".

### 2.3 The blocks

**[SRC]**

```cpp
template <typename... Args>
struct CallbackBlockHeader
{
  using InvokeFunType = void (*)(void*, bool, Args...);
  InvokeFunType run_fun_ = nullptr;
};

template <typename ArgType, typename... Args>
class CallbackBlock : public CallbackBlockHeader<Args...>
{
 public:
  using FunctionType = void (*)(bool, ArgType, Args...);

  CallbackBlock(FunctionType fun, ArgType&& arg);
  CallbackBlock(const CallbackBlock& other) = delete;
  CallbackBlock& operator=(const CallbackBlock& other) = delete;

  static void InvokeThunk(void* cb_block, bool in_isr, Args... args)
  {
    auto* cb = static_cast<CallbackBlock<ArgType, Args...>*>(cb_block);
    cb->Invoke(in_isr, std::forward<Args>(args)...);
  }

 protected:
  void Invoke(bool in_isr, Args... args)
  {
    fun_(in_isr, arg_, std::forward<Args>(args)...);
  }

  FunctionType fun_;
  ArgType arg_;
};
```

Type erasure is the `CallbackBlockHeader<Args...>*` base pointer + `run_fun_` thunk; the bound
argument is stored by value.

### 2.4 The `in_isr` convention

**[SRC]** `in_isr` is the **first parameter of every callback**, always `bool`, and is threaded
through the entire stack:

- `Callback::Run(bool in_isr, PassArgs&&... args)` — the caller states its own context.
- `CallbackBlock::Invoke(bool in_isr, Args...)` forwards it to the user function.
- `CallbackBlockHeader::InvokeFunType = void (*)(void*, bool, Args...)` — it is part of the
  type-erased ABI, not an optional extra.
- `Operation::UpdateStatus(bool in_isr, Status&&)` passes it into `cb.Run(in_isr, status)`.
- `Semaphore::PostFromCallback(bool in_isr)` takes it.
- `Topic::PublishFromCallback(Data&, bool in_isr)` / `PublishFromCallback(Data&, MicrosecondTimestamp, bool in_isr)`.
- `ReadPort::operator()(RawData, ReadOperation&, bool in_isr = false)`.

It is a *caller-supplied declaration of context*, not something the framework detects.
**[INFERENCE]** The framework does not verify it; passing `false` from an ISR would silently
take non-ISR paths.

### 2.5 Callback wrapper and factories

**[SRC]**

```cpp
template <typename... Args>
class Callback
{
  static void FunctionDefault(void*, bool, Args...) {}
  inline static CallbackBlockHeader<Args...> empty_cb_block_ = {&FunctionDefault};

 public:
  template <typename BoundArgType, typename CallableType>
    requires CallbackFunctionCompatible<CallableType, BoundArgType, Args...>
  [[nodiscard]] static Callback Create(CallableType fun, BoundArgType arg)
  {
    using FunctionType = typename CallbackBlock<BoundArgType, Args...>::FunctionType;
    auto cb_block = new CallbackBlock<BoundArgType, Args...>(
        static_cast<FunctionType>(fun), std::move(arg));
    return Callback(cb_block);
  }

  template <typename BoundArgType, typename CallableType>
    requires CallbackFunctionCompatible<CallableType, BoundArgType, Args...>
  [[nodiscard]] static Callback CreateGuarded(CallableType fun, BoundArgType arg)
  {
    using FunctionType = typename CallbackBlock<BoundArgType, Args...>::FunctionType;
    auto cb_block = new GuardedCallbackBlock<BoundArgType, Args...>(
        static_cast<FunctionType>(fun), std::move(arg));
    return Callback(cb_block);
  }

  Callback() : cb_block_(&empty_cb_block_) {}
  Callback(const Callback&) = default;
  Callback& operator=(const Callback&) = default;

  template <typename... PassArgs>
  void Run(bool in_isr, PassArgs&&... args) const
  {
    cb_block_->run_fun_(cb_block_, in_isr, std::forward<PassArgs>(args)...);
  }

  bool Empty() const { return cb_block_ == &empty_cb_block_; }

 private:
  explicit Callback(CallbackBlockHeader<Args...>* cb_block)
      : cb_block_((cb_block != nullptr) ? cb_block : &empty_cb_block_)
  {
  }

  CallbackBlockHeader<Args...>* cb_block_ = &empty_cb_block_;
};
```

**Allocation caveat [DOC]** (<https://xrobot-org.github.io/docs/basic_coding/core/core-callback>):
"当前 `Create` 的实现会 `new CallbackBlock<BoundArgType, Args...>`，因此**包含动态内存分配**；
同时 `Callback` 本身不管理释放." — **`Create` heap-allocates and nothing frees it.** `Callback`
is a shallow-copying non-owning handle; `Empty()` compares against a static empty block, and
`Run` on an empty callback is a safe no-op.

### 2.6 The recursion-flattening behaviour (trampoline)

This is the "recursion-flattening" the request refers to. **[SRC]** verbatim:

```cpp
template <typename ArgType, typename... Args>
class GuardedCallbackBlock : public CallbackBlock<ArgType, Args...>
{
 public:
  GuardedCallbackBlock(typename CallbackBlock<ArgType, Args...>::FunctionType fun,
                       ArgType&& arg)
      : CallbackBlock<ArgType, Args...>(fun, std::move(arg))
  {
    this->run_fun_ = &InvokeThunk;
  }

  static void InvokeThunk(void* cb_block, bool in_isr, Args... args)
  {
    auto* cb = static_cast<GuardedCallbackBlock<ArgType, Args...>*>(cb_block);

    if (!cb->running_)
    {
      cb->running_ = true;
      auto cur_args = std::tuple<std::decay_t<Args>...>{std::forward<Args>(args)...};
      do
      {
        cb->pending_ = false;
        std::apply([&](auto&... a) { cb->Invoke(in_isr, a...); }, cur_args);
        if (cb->pending_)
        {
          cur_args = std::move(cb->pending_args_);
        }
      } while (cb->pending_);
      cb->running_ = false;
      return;
    }

    // 重入时只保留最新一组参数，把递归调用压平成串行重放。
    // On reentry, keep only the latest argument pack so recursive callback chains are
    // flattened into serialized replay.
    cb->pending_args_ = std::tuple<std::decay_t<Args>...>{std::forward<Args>(args)...};
    cb->pending_ = true;
  }

 private:
  bool running_ = false;
  bool pending_ = false;
  std::tuple<std::decay_t<Args>...> pending_args_{};
};
```

**Mechanism:** on re-entry while `running_` is true, no new stack frame is created. Instead a
single pending request is latched (later re-entries *overwrite* the earlier pending args —
only the newest survives). When the outermost invocation finishes its body, the `do/while`
loop replays the pending call at the same stack level. This is the "trampoline flattening"
(trampoline 扁平化) the docs describe, and it prevents unbounded stack growth in cyclical
callback chains such as A → B → C → A. **[DOC]** says the guard exists specifically to suppress
"回调链形成环时的栈递归增长". Args are stored via `std::decay_t` by value so they must be
copyable.

Important **[DOC]**: the guard is *opt-in*. "只有 `CreateGuarded(...)` 路径才会启用
trampoline 扁平化的重入保护；普通 `Create(...)` 只创建基础 `CallbackBlock`."

### 2.7 Doc example (verbatim)

```cpp
void OnEvent(bool in_isr, int context, const char* msg) {
  printf("ISR=%d context=%d msg=%s\n", in_isr, context, msg);
}
auto cb = LibXR::Callback<const char*>::Create(OnEvent, 42);
cb.Run(false, "Hello");
```

Output: `ISR=0 context=42 msg=Hello`

---

## 3. Pipe / queue

### 3.1 `SPSCQueueBase` — the byte-queue core

Source: `src/structure/queue/spsc_queue_base.hpp`, read at
<https://jiu-xiao.github.io/libxr/spsc__queue__base_8hpp_source.html>.

**Answers to the specific questions:**

| Question | Answer | Evidence |
|---|---|---|
| SPSC or MPSC? | **SPSC** (single-producer, single-consumer) | Class name + doc "单生产者单消费者字节队列内核"; Doxygen on `SPSCQueue`: "单生产者单消费者无锁队列" **[SRC]** |
| Lock-free? | **Yes.** Two `std::atomic<size_t>` indices, no mutex, no spinlock | **[SRC]** |
| Uses a semaphore/event? | **Not in the queue itself.** No semaphore member exists. Blocking is a higher-layer concern (`Operation::BLOCK` + `Semaphore`) | **[SRC]** |
| Buffer ownership | **Owns a heap allocation.** `::operator new[]` in the constructor, `::operator delete[]` in the destructor | **[SRC]** |
| Capacity semantics | Stores `capacity_`; ring has `capacity_ + 1` physical slots — one slot is sacrificed to distinguish full from empty | **[SRC]** |

Verbatim key facts:

```cpp
class alignas(LibXR::CONCURRENCY_ALIGNMENT) SPSCQueueBase
{
 public:
  using IndexType = size_t;

  SPSCQueueBase(size_t element_size, size_t capacity);
  SPSCQueueBase(size_t element_size, size_t element_align, size_t capacity);
  ~SPSCQueueBase();

  ErrorCode PushBytes(const void* value);
  ErrorCode PopBytes(void* value = nullptr);
  ErrorCode PeekBytes(void* value);
  ErrorCode PushBatchBytes(const void* data, size_t count);
  ErrorCode PopBatchBytes(void* data, size_t count);
  ErrorCode PeekBatchBytes(void* data, size_t count);

  template <typename Writer>
  ErrorCode PushBytesWithWriter(size_t count, Writer&& writer);
  template <typename Reader>
  ErrorCode PopBytesWithReader(size_t count, Reader&& reader);

  void Reset();
  size_t Size() const;
  size_t EmptySize() const { return capacity_ - Size(); }
  size_t MaxSize() const { return capacity_; }

 private:
  void InitStorage()
  {
    ASSERT(element_size_ > 0);
    ASSERT(payload_alloc_align_ > 0);
    ASSERT(capacity_ > 0);
    ASSERT(capacity_ <= std::numeric_limits<size_t>::max() - 1);
    ASSERT((payload_alloc_align_ & (payload_alloc_align_ - 1)) == 0);

    const size_t payload_bytes = MultiplyChecked(payload_stride_, RingCapacity());
    payloads_ = static_cast<std::byte*>(
        ::operator new[](payload_bytes, std::align_val_t(payload_alloc_align_)));
  }

  size_t RingCapacity() const { return capacity_ + 1; }
  IndexType Increment(IndexType index) const { return (index + 1) % RingCapacity(); }

  std::byte* payloads_;

  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<IndexType> head_;  // next to dequeue
  alignas(LibXR::CONCURRENCY_ALIGNMENT) std::atomic<IndexType> tail_;  // next to enqueue
};
```

Memory ordering **[SRC]**: producer loads `tail_` relaxed, loads `head_` acquire, stores
`tail_` release. Consumer loads `head_` relaxed, loads `tail_` acquire, stores `head_` release.
This is the textbook correct SPSC ring discipline. Copy and move are explicitly deleted
("禁止拷贝构造 / 禁止移动构造").

### 3.2 `SPSCQueue<T>` — the typed wrapper

Source: `src/structure/queue/spsc_queue.hpp` at
<https://jiu-xiao.github.io/libxr/spsc__queue_8hpp_source.html>.

```cpp
template <typename Data>
class SPSCQueue final : public QueueTypedBase<SPSCQueue<Data>, Data>, public SPSCQueueBase
{
 public:
  static_assert(alignof(Data) <= alignof(std::max_align_t),
                "SPSCQueue does not support over-aligned payload types");

  using ValueType = Data;
  using QueueTypedBase<SPSCQueue<Data>, Data>::Pop;
  using QueueTypedBase<SPSCQueue<Data>, Data>::Push;

  explicit SPSCQueue(size_t length) : SPSCQueueBase(sizeof(Data), alignof(Data), length) {}
  ~SPSCQueue() = default;

  ErrorCode Peek(Data& item);
  ErrorCode PushBatch(const Data* data, size_t size);
  ErrorCode PopBatch(Data* data, size_t size);
  ErrorCode PeekBatch(Data* data, size_t size);

  template <typename Writer> ErrorCode PushWithWriter(Writer&& writer);
  template <typename Writer> ErrorCode PushWithWriter(size_t size, Writer&& writer);
  template <typename Reader> ErrorCode PopWithReader(Reader&& reader);
  template <typename Reader> ErrorCode PopWithReader(size_t size, Reader&& reader);

  void Reset();
};
```

And the CRTP base **[SRC]** (`queue_typed_base.hpp`):

```cpp
template <typename Derived, typename Data>
class QueueTypedBase
{
 public:
  using ValueType = Data;

  ErrorCode Push(const Data& item)
  {
    return static_cast<Derived*>(this)->PushBytes(&item);
  }

  ErrorCode Pop(Data& item) { return static_cast<Derived*>(this)->PopBytes(&item); }

  ErrorCode Pop() { return static_cast<Derived*>(this)->PopBytes(nullptr); }
};
```

**Ownership / lifetime caveat [SRC]:** the typed layer is a *raw byte copy* of `T`. The
`PushWithWriter`/`PopWithReader` batch forms `static_assert` that `Data` is
`std::is_trivially_copyable_v` **and** `std::is_trivially_destructible_v`. The single-element
`Push`/`Pop` do not carry those asserts but still only `FastCopy` bytes — **[INFERENCE]** so a
non-trivially-copyable `T` would be copied bitwise with its destructor never run. Docs confirm
the intent: "队列内部不管理 `T` 的复杂生命周期" **[DOC]**
(<https://xrobot-org.github.io/docs/basic_coding/structure/spsc_queue>).

Batch API contract **[SRC]**: `PushBytes` is `Memory::FastCopy` in a loop then one release
store. On overflow it returns `ErrorCode::FULL`; on underflow `ErrorCode::EMPTY`; null pointer
`ErrorCode::PTR_NULL`. Batch operations are all-or-nothing (capacity is checked up front).

### 3.3 `Pipe`

Source: `src/core/libxr_pipe.hpp` at <https://jiu-xiao.github.io/libxr/libxr__pipe_8hpp_source.html>.

```cpp
class Pipe
{
 public:
  Pipe(size_t buffer_size) : read_port_(0), write_port_(1, buffer_size)
  {
    // 绑定回调并共享同一数据队列。
    // Bind callbacks and share the same data queue.
    read_port_.read_fun_ = ReadFun;
    write_port_.write_fun_ = WriteFun;
    read_port_.queue_data_ = write_port_.queue_data_;
  }

  ~Pipe() {}
  Pipe(const Pipe&) = delete;
  Pipe& operator=(const Pipe&) = delete;

  ReadPort& GetReadPort() { return read_port_; }
  WritePort& GetWritePort() { return write_port_; }

 private:
  static ErrorCode ReadFun(ReadPort&, bool) { return ErrorCode::PENDING; }

  static ErrorCode WriteFun(WritePort& port, bool in_isr)
  {
    auto* pipe = LibXR::ContainerOf(&port, &Pipe::write_port_);
    WriteInfoBlock info;
    if (port.queue_info_->Pop(info) != ErrorCode::OK)
    {
      ASSERT(false);
      return ErrorCode::EMPTY;
    }

    // 推动读端从共享队列中取数。
    // Drive the reader to consume from the shared queue.
    pipe->read_port_.ProcessPendingReads(in_isr);

    return ErrorCode::OK;
  }

  ReadPort read_port_;
  WritePort write_port_;
};
```

**Key structural facts [SRC] + [DOC]:**

- Internal shape is exactly `ReadPort(0)` and `WritePort(1, buffer_size)`. The read port owns
  **no** private data queue; the write port owns a 1-slot metadata queue plus a byte queue of
  `buffer_size`.
- The two ports **share one byte queue** by pointer assignment (`read_port_.queue_data_ =
  write_port_.queue_data_`). There is exactly one copy of the payload — into that shared queue.
- The read side is **passive**: `ReadFun` is a stub that returns `ErrorCode::PENDING`, i.e. it
  never completes anything itself. Progress happens only when the write side calls
  `ProcessPendingReads(in_isr)`.
- `Pipe` is non-copyable, non-movable, and exposes no `Size()` / `Reset()` of its own — you go
  through `GetReadPort()` / `GetWritePort()`.

**[DOC] caveat** (<https://xrobot-org.github.io/docs/basic_coding/core/core-pipe>): `WriteFun`
pops a `WriteInfoBlock` from `queue_info_`; if that pop fails the implementation asserts and
returns `ErrorCode::EMPTY`. The docs note this means `Pipe` depends on `WritePort`'s specific
enqueue/completion semantics, not a generic write-callback protocol.

### 3.4 Where the semaphore actually is

**[SRC]** To be explicit, since the question asks: the queue is lock-free and semaphore-free.
The semaphore in the I/O path belongs to `Operation::BLOCK` (`data.sem_info.sem`) and to
`ReadPort`'s BLOCK handoff state. `ReadPort` carries
`std::atomic<BusyState> busy_` with states
`IDLE = 0, PENDING = 1, CLEARING = 2, BLOCK_CLAIMED = 3, BLOCK_DETACHED = 4, EVENT = UINT32_MAX`
and an `ErrorCode block_result_` field for the BLOCK result handoff. **[SRC]**
(<https://jiu-xiao.github.io/libxr/read__port_8hpp_source.html>)

---

## 4. Message / Event / Topic middleware

### 4.1 What abstractions exist

**[DOC]** The middleware index (<https://xrobot-org.github.io/docs/basic_coding/middleware>)
lists: Application framework, Logger, **Event**, **Message** (Topic / Packet-Server /
LinuxSharedTopic), Database (flash KV), RamFS, Terminal.

**[DOC]** The message page (<https://xrobot-org.github.io/docs/basic_coding/middleware/message>)
narrows the current mainline to three public paths: `Topic` (in-process, strongly typed
pub/sub), `Topic::Server` / `Packet` (byte-stream framing over serial/bus/network), and
`LinuxSharedTopic` (host-side shared-memory cross-process). It also states a breaking change:
"`Topic` 当前主线的核心契约是'强类型分发'，而不是旧版本常见的'Topic 自带 latest cache'模型."

### 4.2 `Topic` — real API surface

Source: `src/middleware/message/topic.hpp` at
<https://jiu-xiao.github.io/libxr/topic_8hpp_source.html>. **[SRC]** verbatim:

```cpp
template <typename Data>
concept TopicPayload =
    !std::is_reference_v<Data> && !std::is_const_v<Data> && !std::is_volatile_v<Data> &&
    std::is_object_v<Data> && std::is_default_constructible_v<Data> &&
    std::is_copy_assignable_v<Data> && std::is_trivially_destructible_v<Data>;
```

The topic handle is a node in a red-black tree:

```cpp
typedef RBTree<uint32_t>::Node<Block>* TopicHandle;

struct Block
{
  std::atomic<LockState> busy;
  LockFreeList subers;
  TypeID::ID payload_type_id;
  uint32_t payload_size;
  uint32_t payload_alignment;
  uint32_t crc32;
  Mutex* mutex;
};

enum class LockState : uint32_t { UNLOCKED = 0, LOCKED = 1, USE_MUTEX = UINT32_MAX };
```

Message views:

```cpp
template <typename Data>
struct MessageView
{
  MicrosecondTimestamp timestamp;
  Data* data;
};

struct RawMessageView
{
  MicrosecondTimestamp timestamp;
  ConstRawData payload;
};

template <typename Data>
struct Message
{
  MicrosecondTimestamp timestamp;
  Data data;
};
```

**Type identity is the triple `payload_type_id + payload_size + payload_alignment` [DOC]** —
"类型约束由 `payload_type_id + payload_size + payload_alignment` 共同定义". The subscription
check is an assert-triple **[SRC]**:

```cpp
  template <typename Data>
  static void CheckSubscriberType(Topic topic)
  {
    CheckTopicPayload<Data>();
    ASSERT(topic.block_ != nullptr);
    ASSERT(topic.block_->data_.payload_type_id == TypeID::GetID<Data>());
    ASSERT(topic.block_->data_.payload_size == sizeof(Data));
    ASSERT(topic.block_->data_.payload_alignment == alignof(Data));
  }
```

`TypeID::GetID<T>()` returns "每种类型返回一个进程内唯一的静态地址（`const void*`）" — i.e. the
address of a per-type static, deliberately avoiding RTTI/`typeid` **[DOC]**
(<https://xrobot-org.github.io/docs/basic_coding/core/core-rawdata>).

### 4.3 Compile-time or runtime registration?

**Both, at different layers — this is the nuanced answer:**

- **Topic *type* checking is compile-time.** `CreateTopic<T>()` is a template that fills the
  runtime contract from `TypeID::GetID<Data>()`, `sizeof(Data)`, `alignof(Data)`. The
  `TopicPayload` concept and `CheckSubscriberType` asserts are compile-time/enabled-assert
  mechanisms. **[SRC]**
- **Topics themselves are registered at *runtime*, by name.** `Topic::Find(name, domain)`,
  `Topic::FindOrCreate<Data>(name, domain, multi_publisher)`, `Topic::WaitTopic(name, timeout,
  domain)` all take a `const char*` name and resolve through a static `RBTree<uint32_t>` keyed
  by CRC32 of the name. **[SRC]**
- **Subscribers are registered at runtime** onto a per-topic `LockFreeList`:
  `RegisterCallback(Callback& cb)`, `SyncSubscriber`, `ASyncSubscriber`, `QueuedSubscriber`.
  **[SRC]**

So: *typed* but *dynamically discovered*. **[INFERENCE]** This is what makes the code generator
able to wire modules by name string without them knowing about each other at compile time.

### 4.4 Publish / subscribe semantics

**[SRC]** Publication is *synchronous fan-out inside the publish call*:

```cpp
  template <typename Data>
  void Publish(Data& data)
  {
    PublishTyped(data, NowTimestamp(), false, false);
  }

  template <typename Data>
  void Publish(Data& data, MicrosecondTimestamp timestamp)
  {
    PublishTyped(data, timestamp, false, false);
  }

  template <typename Data>
  void PublishFromCallback(Data& data, bool in_isr)
  {
    PublishTyped(data, NowTimestamp(), true, in_isr);
  }

  template <typename Data>
  void PublishFromCallback(Data& data, MicrosecondTimestamp timestamp, bool in_isr)
  {
    PublishTyped(data, timestamp, true, in_isr);
  }
```

`PublishTyped` locks, asserts the publish contract, calls
`DispatchSubscribers(block_, timestamp, &data, from_callback, in_isr)`, then unlocks.
Locking picks between an atomic fast path and a `Mutex` **[SRC]** (via `LockState`:
`UNLOCKED` / `LOCKED` / `USE_MUTEX`). Docs phrase it **[DOC]**: "`multi_publisher = false` 时
优先走轻量原子快路径；`multi_publisher = true` 时改用 `Mutex` 串行化."

**No buffering [DOC]**: "`Topic` 本身只负责本次发布的分发，不保存 latest payload 副本" and
explicitly "不保存最近一次消息缓存" / "不提供 `DumpData()`". The four subscriber kinds trade
differently — `SyncSubscriber` writes into a caller-provided object, `ASyncSubscriber` needs an
explicit `StartWaiting()` per message, `QueuedSubscriber` pushes into an **`SPSCQueue`** (and
silently **drops** on full, never blocking the publisher), `Callback` runs on every publish.

Subscriber type enum **[SRC]**: `enum class SuberType : uint8_t { SYNC, ASYNC, QUEUE, CALLBACK };`

Callback subscription signatures **[DOC]** — four accepted shapes:
`T` / `T&` / `const T&` (typed payload), `MessageView<T>` (timestamp + data pointer),
`RawMessageView` / `ConstRawData` (raw payload view). The bound `void*` is the user argument.

### 4.5 Packet / Server — the wire format

**[DOC]** (<https://xrobot-org.github.io/docs/basic_coding/middleware/message/message-packet-server>).
This is fully specified and directly relevant to any integration:

Header is fixed at 16 bytes, plus a trailing CRC8:

| Field | Bytes | Meaning |
|---|---|---|
| `prefix` | 1 | fixed `0x5A` |
| `data_len_raw` | 3 | little-endian 24-bit payload length |
| `topic_name_crc32` | 4 | CRC32 key of the topic name |
| `timestamp_us_raw` | 6 | little-endian 48-bit microsecond timestamp |
| `version` | 1 | protocol version, mainline is `0x01` |
| `pack_header_crc8` | 1 | header CRC8 |
| `payload` | N | payload bytes |
| trailing `crc8` | 1 | whole-packet CRC8 |

Confirmed in source **[SRC]**:

```cpp
  static constexpr uint8_t PACKET_PREFIX = 0x5A;
  static constexpr uint8_t PACKET_VERSION = 0x01;
  static constexpr size_t PACK_BASE_SIZE = 17;
```

API: `PackData(const Data&, PackedData<Data>&)` / `PackData(..., MicrosecondTimestamp)`,
`PackRaw(ConstRawData, RawData, [timestamp])`, `Server::Register(topic)`,
`Server::ParseData(ConstRawData)`, `Server::ParseDataFromCallback(ConstRawData, bool in_isr)`.
`PackRaw` returns `ErrorCode::PTR_NULL` / `SIZE_ERR` / `NO_BUFF` on the documented failure
modes. `Server(buffer_length)` registration asserts
`payload_size + PACK_BASE_SIZE` fits the buffer and `payload_alignment <= CACHE_LINE_SIZE`.

### 4.6 `Event` — the separate, ID-keyed mechanism

Source: `src/middleware/event.hpp` at <https://jiu-xiao.github.io/libxr/event_8hpp_source.html>.

**[SRC]**

```cpp
class Event
{
 public:
  using Callback = LibXR::Callback<uint32_t>;
  using CallbackList = LockFreeList*;

  Event();

  void Register(uint32_t event, const Callback& cb);
  void Active(uint32_t event);
  void ActiveFromCallback(CallbackList list, uint32_t event, bool in_isr = true);
  CallbackList GetList(uint32_t event);
  void Bind(Event& sources, uint32_t source_event, uint32_t target_event);

 private:
  struct Block
  {
    uint32_t event;
    Callback cb;
  };

  RBTree<uint32_t> rbt_;
};
```

**`Event` and `Topic` are entirely separate subsystems.** `Event` is keyed by a numeric
`uint32_t` event ID; `Topic` is keyed by a name string → CRC32 with a typed payload contract.
**[INFERENCE]** `Event` is the cheaper, untyped, ID-based notification path; `Topic` is the
typed data path.

The ISR story **[DOC]**: `GetList(event)` must be called in a normal context to obtain and
cache the callback-list pointer; that pointer is then used from the ISR via
`ActiveFromCallback(list, event, true)`. Doxygen restates it: *"获取指定事件的回调链表指针
（必须在非中断上下文中调用）"*. This is because the underlying `RBTree` lookup is not
ISR-safe. Internally: `RBTree<uint32_t>` maps event ID → `Block`, and each `Block`'s callbacks
live in a `LockFreeList`. `Bind` bridges one event's activation to another event ID.

Doc example:

```cpp
int counter = 0;
LibXR::Event evt;
auto cb = LibXR::Event::Callback::Create(
    [](bool, int *cnt, uint32_t event) {
        (*cnt)++;
        ASSERT(event == 0x10);
    },
    &counter);
evt.Register(0x10, cb);
evt.Active(0x10);
```

```cpp
evt_dst.Bind(evt_src, 0xA, 0xB);  // 当 evt_src 的事件 0xA 触发时，会激活 evt_dst 的事件 0xB
```

### 4.7 Relation to the Operation model

**[SRC] + [INFERENCE]** There is **no structural coupling**. `Topic` uses `LockFreeList`,
`RBTree`, `Mutex`, `Semaphore`, `Thread` — but never `Operation`. `Operation` is exclusively
the I/O-completion abstraction for `ReadPort`/`WritePort`-based drivers. The only shared
plumbing is `Callback<Args...>` and the `in_isr` convention, which both use. If you need "give
me the next message like an operation", the idiomatic LibXR equivalent is
`ASyncSubscriber::StartWaiting()` / `Available()` / `GetData()` or `SyncSubscriber::Wait(timeout)`,
not an `Operation`.

---

## 5. Time

Source: `src/core/libxr_time.hpp` at <https://jiu-xiao.github.io/libxr/libxr__time_8hpp_source.html>
and `src/driver/timebase.hpp` at <https://jiu-xiao.github.io/libxr/timebase_8hpp_source.html>.

### 5.1 Timestamp types

**[SRC]** Two independent types, not one templated clock:

- `MicrosecondTimestamp` — wraps `uint64_t microsecond_`, `operator uint64_t() const`.
- `MillisecondTimestamp` — wraps `uint32_t millisecond_`, `operator uint32_t() const`.

Note the **width asymmetry**: microseconds are 64-bit, milliseconds are **32-bit**.

Each has a nested `Duration`:

```cpp
class Duration
{
 public:
  Duration(uint64_t diff) : diff_(diff) {}
  operator uint64_t() const { return diff_; }
  [[nodiscard]] double ToSecond() const { return static_cast<double>(diff_) / 1000000.0; }
  [[nodiscard]] float ToSecondf() const { return static_cast<float>(diff_) / 1000000.0f; }
  [[nodiscard]] uint64_t ToMicrosecond() const { return diff_; }
  [[nodiscard]] uint32_t ToMillisecond() const { return diff_ / 1000u; }
 private:
  uint64_t diff_ = 0;
};
```

### 5.2 Resolution and wraparound handling

Resolution is **1 µs** for `MicrosecondTimestamp`, **1 ms** for `MillisecondTimestamp`.

**[SRC]** The subtraction operators implement explicit wraparound arithmetic against a
configurable timebase ceiling (a new default-constructed `MicrosecondTimestamp` is zero):

```cpp
  [[nodiscard]] Duration operator-(const MicrosecondTimestamp& old_timestamp) const
  {
    uint64_t elapsed = 0;
    const uint64_t max_valid = Detail::TimebaseMaxValidUs();

    if (microsecond_ >= old_timestamp.microsecond_)
    {
      elapsed = microsecond_ - old_timestamp.microsecond_;
    }
    else
    {
      elapsed = microsecond_ + (max_valid - old_timestamp.microsecond_) + 1ULL;
    }

    ASSERT(elapsed <= max_valid);

    return Duration(elapsed);
  }
```

The millisecond version is identical with `uint32_t` / `TimebaseMaxValidMs()`.

The ceiling storage **[SRC]** (replacing the older external globals, as the docs note):

```cpp
namespace Detail
{
[[nodiscard]] inline uint64_t& TimebaseMaxValidUsStorage() noexcept
{
  static uint64_t value = UINT64_MAX;
  return value;
}

[[nodiscard]] inline uint32_t& TimebaseMaxValidMsStorage() noexcept
{
  static uint32_t value = UINT32_MAX;
  return value;
}

[[nodiscard]] inline uint64_t TimebaseMaxValidUs() noexcept { return TimebaseMaxValidUsStorage(); }
[[nodiscard]] inline uint32_t TimebaseMaxValidMs() noexcept { return TimebaseMaxValidMsStorage(); }

inline void ConfigureTimebaseWrapRange(uint64_t max_valid_us, uint32_t max_valid_ms) noexcept
{
  TimebaseMaxValidUsStorage() = max_valid_us;
  TimebaseMaxValidMsStorage() = max_valid_ms;
}
}  // namespace Detail
```

Defaults are `UINT64_MAX` / `UINT32_MAX`, i.e. **no wraparound correction unless a platform
configures it**. **[INFERENCE]** A platform with e.g. a 16-bit or 32-bit hardware timer counter
must call `ConfigureWrapRange` or elapsed-time math after a counter wrap will assert/misbehave.

### 5.3 The monotonic clock source — `Timebase`

**[SRC]** `Timebase` is a pure static interface; a *platform backend* provides the definitions:

```cpp
class Timebase
{
 public:
  Timebase() = default;
  Timebase(const Timebase&) = delete;
  Timebase& operator=(const Timebase&) = delete;

  static MicrosecondTimestamp GetMicroseconds();
  static MillisecondTimestamp GetMilliseconds();

  [[nodiscard]] static bool IsReady() noexcept { return ready_; }

  static inline void DelayMicroseconds(uint32_t us)
  {
    if (us == 0u)
    {
      return;
    }

    const uint64_t START = static_cast<uint64_t>(Timebase::GetMicroseconds());
    while ((static_cast<uint64_t>(Timebase::GetMicroseconds()) - START) < us)
    {
      // busy-wait
    }
  }

 protected:
  static void SetReady(bool ready = true) noexcept { ready_ = ready; }

  static void ConfigureWrapRange(uint64_t max_valid_us, uint32_t max_valid_ms) noexcept
  {
    Detail::ConfigureTimebaseWrapRange(max_valid_us, max_valid_ms);
  }

  [[nodiscard]] static uint64_t GetConfiguredWrapRangeUs() noexcept
  {
    return Detail::TimebaseMaxValidUs();
  }

  [[nodiscard]] static uint32_t GetConfiguredWrapRangeMs() noexcept
  {
    return Detail::TimebaseMaxValidMs();
  }

 private:
  static inline bool ready_ = false;
};
```

Two decisive facts:

1. **`GetMicroseconds()` is declared but not defined here.** Doxygen's cross-reference shows
   the definition living in `ch32_timebase.cpp` **[SRC]** — i.e. **each platform backend
   supplies its own definition**. That is the port mechanism for the clock.
2. **`DelayMicroseconds` is a busy-wait**, not a sleep.

`IsReady()` / `SetReady()` is the handshake a platform uses to announce its timebase is up.
`PlatformInit(uint32_t timer_pri = 2, uint32_t timer_stack_depth = 65536)` (Linux signature)
is the documented platform bring-up entry **[SRC]**
(<https://jiu-xiao.github.io/libxr/libxr__system_8hpp_source.html>); the STM32 generated code
calls `PlatformInit(2, 1024)` **[DOC]**.

### 5.4 Timer abstraction

Two distinct things exist; do not conflate them.

**(a) `LibXR::Timer` — software periodic task scheduler [DOC]**
(<https://xrobot-org.github.io/docs/basic_coding/system/timer>):

| Method | Purpose |
|---|---|
| `template <typename Arg> static TimerHandle CreateTask(void (*fun)(Arg), Arg arg, uint32_t cycle)` | create periodic task, cycle in **ms** |
| `static void Start(TimerHandle handle)` | start |
| `static void Stop(TimerHandle handle)` | stop |
| `static void SetCycle(TimerHandle handle, uint32_t cycle)` | change period |
| `static void Add(TimerHandle handle)` | add to schedule; first `Add` in a multithreaded build creates the management thread |
| `static void Refresh()` | run one scheduling pass |
| `static void RefreshTimerInIdle()` | bare-metal hook; called automatically by `Thread` sleep / `Mutex` / `Semaphore` waits |

Stated accuracy is **1 ms**, built on `Thread::SleepUntil`. Two modes: multithreaded/RTOS uses
a dedicated management thread polling at 1 ms; bare-metal/single-threaded piggybacks the
refresh on existing wait paths. Each task is a `ControlBlock` held in a `List`. The docs note
that porting only requires `Thread` + `Timebase` to work — the Timer body needs no changes.

**(b) `STM32TimerTimebase` — a hardware-timer-backed Timebase [DOC]**
(<https://xrobot-org.github.io/docs/code_gen/stm32>), constructed as
`STM32TimerTimebase timebase(&htim2);`. Doxygen also lists `STM32Timebase`,
`LinuxTimebase`, `ESP32Timebase`, `HPMTimebase`, `MSPM0Timebase`,
`WebAsmTimebase`, `WebotsTimebase` **[SRC]** — one per platform backend.

---

## 6. CodeGenerator

**Scope note:** this section is thinner than the rest because the XRobot/CodeGenerator
sub-pages beyond the top-level ones were being fetched by a delegated subagent that was
stopped before it reported. What follows is what I verified directly.

### 6.1 Two separate generators

There are **two distinct code-generation toolchains**, and they solve different problems:

**(A) LibXR CodeGenerator** — C++ peripheral init from an MCU project.
**[DOC]** (<https://xrobot-org.github.io/docs/code_gen>):
"CodeGenerator 的核心作用，是根据 SDK 或工程描述文件生成 LibXR 侧的初始化与适配代码。
例如依靠 STM32CubeMX 的 IOC 文件，生成对应的 C++ 外设初始化代码与 `libxr_config.yaml`
配置骨架." Coverage: STM32 (GPIO, UART, SPI, I2C, ADC, DAC, CAN, PWM, Flash, Cache,
Watchdog, Timebase) and XRobot integration.

**(B) XRobot** — module dependency management + `main` generation. See §7.

### 6.2 Input format — concrete answer

**The input is STM32CubeMX `.ioc`, parsed into YAML.** The pipeline is explicit **[DOC]**
(<https://xrobot-org.github.io/docs/code_gen/stm32>):

1. initialise/update the `libxr` submodule
2. find the `.ioc` file and parse it to `.config.yaml`
3. generate `app_main.cpp`
4. patch `CMakeLists.txt` to integrate LibXR

Entry point: `xr_cubemx_cfg -d .` run from the CubeMX project root.

Generated file layout **[DOC]** verbatim:

```
.
├── .config.yaml                      # 解析生成的配置文件
├── User/
│   │── app_main.cpp                  # 主入口初始化代码
│   │   app_main.h                    # 主入口初始化代码的头文件
│   │── libxr_config.yaml             # LibXR 配置文件
│   └── flash_map.hpp                 # FLASH 地址映射表
├── cmake/LibXR.CMake                 # LibXR 构建配置
├── CMakeLists.txt                    # 自动集成 LibXR
└── Middlewares/Third_Party/LibXR     # Git 子模块：LibXR 本体
```

So the config files are **`.config.yaml`** (generated from `.ioc`, feeds the generator) and
**`libxr_config.yaml`** (LibXR-side configuration). Both are YAML.

Sub-commands **[DOC]**:

| Tool | Function |
|---|---|
| `xr_cubemx_generate` | CubeMX script-mode generation only |
| `xr_cubemx_cfg` | full pipeline (parse + generate + cmake) |
| `xr_parse_ioc` | parse `.ioc` → `.config.yaml` |
| `xr_gen_code_stm32` | generate `app_main.cpp` from YAML |
| `xr_stm32_flash` | generate STM32 flash layout table |
| `xr_stm32_cmake` | patch CMake to integrate LibXR |
| `xr_stm32_toolchain_switch` | switch toolchain/standard library |

Flags for `xr_cubemx_cfg`: `-d` (project root), `-t` (terminal peripheral, e.g. `usart1`),
`--xrobot`, `--commit`, `--git-source`, `--git-mirrors`. `xr_stm32_toolchain_switch` accepts
`gcc`, `clang -g`, `clang --newlib`, `clang --picolibc`.

Project requirements **[DOC]**: must be a STM32CubeMX **CMake** export; must have an `.ioc`
file; FreeRTOS must have `configUSE_MUTEXES` enabled.

### 6.3 What it emits — verbatim generated `app_main.cpp`

**[DOC]** (<https://xrobot-org.github.io/docs/code_gen/stm32>), abridged to the substantive part:

```cpp
#include "app_main.h"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_dac.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
......
using namespace LibXR;
/* User Code Begin 1 */
/* User Code End 1 */
/* External HAL Declarations */
extern ADC_HandleTypeDef hadc1;
extern CAN_HandleTypeDef hcan1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
......
/* DMA Resources */
static uint16_t adc1_buf[64];
static uint8_t spi1_tx_buf[32];
static uint8_t spi1_rx_buf[32];
static uint8_t usart1_tx_buf[128];
static uint8_t usart1_rx_buf[128];
static uint8_t i2c1_buf[32];
......
extern "C" void app_main(void) {
  /* User Code Begin 2 */
    /* User Code End 2 */
  STM32TimerTimebase timebase(&htim2);
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;
  /* GPIO Configuration */
  STM32GPIO USER_KEY(USER_KEY_GPIO_Port, USER_KEY_Pin, EXTI0_IRQn);
  STM32GPIO LED_B(LED_B_GPIO_Port, LED_B_Pin);
  STM32PWM pwm_tim1_ch1(&htim1, TIM_CHANNEL_1, false);
  STM32SPI spi1(&hspi1, spi1_rx_buf, spi1_tx_buf, 3);
  STM32UART usart1(&huart1,
              usart1_rx_buf, usart1_tx_buf, 5);
  STM32I2C i2c1(&hi2c1, i2c1_buf, 3);
  STM32CAN can1(&hcan1, 5);
  /* User Code Begin 3 */
  while (1) {
      LibXR::Thread::Sleep(1000);
  }
  /* User Code End 3 */
}
```

Observations worth noting for integration **[INFERENCE]**:
- Buffer sizes are **hard-coded per-peripheral constants** (`usart1_rx_buf[128]`,
  `adc1_buf[64]`, `spi1_tx_buf[32]`) and are passed positionally into the driver constructors.
- Peripheral objects are **function-local statics inside `app_main`**, which is why the docs
  warn: "此函数应当永不返回，一旦函数返回所有外设对象（如 usart1）将会析构并释放资源，此时访问
  将会导致崩溃" **[DOC]**.
- `User Code Begin/End` fences are preserved across regeneration.
- CMSIS peripheral names (`huart1`, `hspi1`, `EXTI0_IRQn`) leak directly into the generated
  C++, so CubeMX is the single source of truth for pin/IRQ config.

### 6.4 XRobot integration mode (`--xrobot`)

**[DOC]** (<https://xrobot-org.github.io/docs/code_gen/code-gen-xrobot-inter>). Invocation:

```
xr_gen_code_stm32 -i ./.config.yaml -o ./User/app_main.cpp --xrobot
[INFO] Detected FreeRTOS configuration
[INFO] FlashLayout is generated and injected, MCU: STM32G431KBU6
[INFO] Flash layout map written to: ./User/flash_map.hpp
[INFO] Successfully generated: ./User
[INFO] Generated header file: app_main.h
```

Emits an extra hardware-container construction into `app_main.cpp`:

```cpp
  LibXR::HardwareContainer peripherals{
    LibXR::Entry<LibXR::PowerManager>{power_manager, {"power_manager"}},
    ...
  };
  XRobotMain(peripherals);
```

And appends peripheral aliases into `libxr_config.yaml`:

```yaml
device_aliases:
  power_manager:
    type: PowerManager
    aliases:
    - power_manager
  LED:
    type: GPIO
    aliases:
    - LED
    - led_red
  ...
```

**This is the seam between the two generators** **[INFERENCE]**: CodeGenerator produces the
`HardwareContainer` + `libxr_config.yaml` aliases; XRobot's `xrobot_gen_main` produces
`XRobotMain(hw)`, which receives that container.

### 6.5 GPIO generation shape (representative peripheral)

**[DOC]** (<https://xrobot-org.github.io/docs/code_gen/stm32/stm32-code-gen-gpio>):

```cpp
// GPIO配置为普通输入输出引脚
STM32GPIO gpioA0(GPIOA, GPIO_PIN_0);
// GPIO配置为外部中断引脚
STM32GPIO gpioA1(GPIOA, GPIO_PIN_1, EXTI1_IRQn);
```

The page states the generator's job is to translate the CubeMX role into an instance shape; it
explicitly does **not** redefine interrupt priority, pull-up/down strategy or runtime callback
logic. It also names the implementation file: `GeneratorCodeSTM32.py`.

---

## 7. Module system

### 7.1 The MANIFEST — exact declaration syntax

This is the key mechanism. **[DOC]**
(<https://xrobot-org.github.io/docs/proj_man/proj-man-create-mod>), verbatim:

> 头文件中的模块描述信息位于 `/* === MODULE MANIFEST === */` 注释块中，格式如下：

```
/* === MODULE MANIFEST V2 ===
module_description: IMU interface module
constructor_args: []
template_args: []
required_hardware:
  - imu
  - scl
  - sda
depends: []
=== END MANIFEST === */
```

So: **the module declaration is a YAML block embedded in a C++ block comment inside the module
header**. Not macros, not a separate file. The doc states this is the core source of module
metadata: "MANIFEST 是模块元信息的核心来源，**生成主函数、文档、依赖树等都基于此内容**."

The parser reads it and prints fields that map 1:1 to the YAML keys:

```
=== Module: MySensor.hpp ===
Description       : IMU interface module
Constructor Args  :
Required Hardware : imu, scl, sda
Depends           : None
```

MANIFEST keys: `module_description`, `constructor_args`, `template_args`,
`required_hardware`, `depends`.

### 7.2 Module directory layout

**[DOC]** `xrobot_create_mod MySensor --desc "IMU interface module" --hw imu scl sda` produces:

```
Modules/
└── MySensor/
    ├── MySensor.hpp        # 含 MANIFEST 的模块头文件
    ├── README.md           # 自动生成的模块说明文档
    ├── CMakeLists.txt      # 构建配置
    └── .github/workflows/build.yml # GitHub 自动测试
```

More create flags **[DOC]**:

```
xrobot_create_mod PIDController \
   --desc "A generic PID controller" \
   --hw input output \
   --constructor kp=1.0 ki=0.2 kd=0.0 \
   --template T=float \
   --depends MySensor
```

`--constructor` and `--template` write into both the MANIFEST and the README; all fields
support automatic type inference (int, float, bool).

### 7.3 Project directory convention

**[DOC]** (<https://xrobot-org.github.io/docs/proj_man>):

```
YourProject/
├── Modules/               # 存放模块仓库
│   └── modules.yaml       # 仓库列表
│   └── sources.yaml       # （可选）模块源索引
├── User/                  # 用户配置与生成输出
│   ├── xrobot.yaml        # 构造参数配置
│   └── xrobot_main.hpp    # 自动生成主函数
```

### 7.4 How modules get wired together

**Two files and a CMake glob.**

**(a) `Modules/modules.yaml`** — the repo list; each line a full module name **[DOC]**:

```
- xrobot-org/BlinkLED
- your-namespace/YourModule@dev
```

**(b) `User/xrobot.yaml`** — the instance configuration. Complete verbatim example **[DOC]**:

```yaml
global_settings:
  monitor_sleep_ms: 1000
modules:
- name: BlinkLED
  constructor_args:
    blink_cycle: 250
- id: MySensor_0
  name: MySensor
  constructor_args: {}
```

With template args and an explicit instance id **[DOC]**:

```yaml
- name: PID
  constructor_args:
    kp: 1.0
    ki: 0.2
  template_args:
    T: float
  id: pid_left
```

Keys: `global_settings.monitor_sleep_ms`, `modules[].name`, `modules[].id`,
`modules[].constructor_args`, `modules[].template_args`. **`id` becomes the generated C++
variable name** — the doc shows `xrobot_add_mod MySensor` appending
`as id 'MySensor_0' to User/xrobot.yaml`.

**(c) Generated `User/xrobot_main.hpp`** — verbatim **[DOC]**
(<https://xrobot-org.github.io/docs/proj_man/proj-man-gen-main>):

```cpp
#include "app_framework.hpp"
#include "libxr.hpp"
// Module headers
#include "BlinkLED.hpp"
#include "MySensor.hpp"
static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;
  // Auto-generated module instantiations
  static BlinkLED blinkled(hw, appmgr, 250);
  static MySensor MySensor_0(hw, appmgr);
  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(1000);
  }
}
```

**The universal module constructor signature is therefore
`Module(LibXR::HardwareContainer& hw, ApplicationManager& appmgr, <constructor_args...>)`**
**[INFERENCE]** from the generated call sites. Template modules get
`static PID<float> pid_left(hw, appmgr, 1.0, 0.2);` **[DOC]**. Note the constructor args are
passed **positionally in MANIFEST order**, not by name, at the call site — the YAML names are
resolved to positions by the generator.

`xrobot_gen_main` flags: `--config User/xrobot.yaml` (reuse config, skip rescan),
`--output User/xrobot_main.hpp`, `--hw <name>` (rename the hardware container variable).

### 7.5 Module composition contract

`Application` is the base class **[DOC]**
(<https://xrobot-org.github.io/docs/basic_coding/middleware/app-framework>):

- **`HardwareContainer`** — device registry with multi-alias registration, typed lookup
  `Find<T>("alias")`, and `FindOrExit<T>({...})` which asserts on miss.
- **`Application`** — abstract module base; must implement `OnMonitor()`.
- **`ApplicationManager`** — scheduler; `Register(app)` and `MonitorAll()` which periodically
  calls every module's `OnMonitor()`.

```cpp
class MyApp : public Application { public:
  void OnMonitor() override {
    // 用户定义的周期任务逻辑
  }};
MyApp app;
LibXR::ApplicationManager manager;
manager.Register(app);
manager.MonitorAll();  // 周期调用所有模块的 OnMonitor()
```

Container construction **[DOC]**:

```cpp
HardwareContainer container(
  Entry<UART>{uart1, {"uart1", "console"}},
  Entry<GPIO>{gpio1, {"gpio1", "LED"}});
```

The doc explicitly states XRobot's generator emits this registration code for each module:
"XRobot 系统的自动生成工具会为每个模块自动生成注册代码，统一调用 `HardwareContainer` 与
`ApplicationManager`". Modules are stored in a `LockFreeList`.

**Dependency resolution [DOC]** is via the MANIFEST `depends:` field — "
`xrobot_init_mod` ... 递归解析模块仓库" and `depends` feeds the dependency tree. The exact
topological ordering algorithm is **not documented on the pages I read** — I am flagging this
as a gap rather than guessing.

### 7.6 The CMake wiring (source-verified)

**[SRC]** `CMakeLists.txt` (<https://raw.githubusercontent.com/Jiu-xiao/libxr/master/CMakeLists.txt>)
shows the actual composition mechanism — module directories are globbed and each one's
`CMakeLists.txt` is `include()`d, and modules publish their link dependencies through a global
property:

```cmake
if(XROBOT_MODULES_DIR)
  file(
    GLOB _xr_module_dirs CONFIGURE_DEPENDS
    RELATIVE ${XROBOT_MODULES_DIR}
    ${XROBOT_MODULES_DIR}/*
  )
  list(SORT _xr_module_dirs)

  foreach(_xr_mod IN LISTS _xr_module_dirs)
    if(IS_DIRECTORY "${XROBOT_MODULES_DIR}/${_xr_mod}")
      if(EXISTS "${XROBOT_MODULES_DIR}/${_xr_mod}/CMakeLists.txt")
        message(STATUS "[XRobot] Including module: ${_xr_mod}")
        include("${XROBOT_MODULES_DIR}/${_xr_mod}/CMakeLists.txt")
      endif()
    endif()
  endforeach()

  get_property(_xr_module_deps GLOBAL PROPERTY XR_MODULE_DEPS)
  if(_xr_module_deps)
    list(REMOVE_DUPLICATES _xr_module_deps)
    list(SORT _xr_module_deps)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${_xr_module_deps})
  endif()
endif()
```

**Design consequence [INFERENCE]:** each module's `CMakeLists.txt` is expected to append to the
global `XR_MODULE_DEPS` property. Modules that need a third-party library declare it there, and
LibXR links them all in one shot. Directory name order is sorted, but the ordering is only
cosmetic for CMake include order — actual dependency correctness is carried by the MANIFEST
`depends` and by static C++ instantiation order in `xrobot_main.hpp`.

### 7.7 CLI reference

**[DOC]** (<https://xrobot-org.github.io/docs/proj_man>):

| Command | Description |
|---|---|
| `xrobot_gen_main` | Generate main C++ entry source file |
| `xrobot_mod_parser` | Parse and show module manifest |
| `xrobot_create_mod` | Create a new module folder & header |
| `xrobot_init_mod` | Clone and recursively sync all module repos |
| `xrobot_setup` | One-click workspace setup & main function generate |
| `xrobot_add_mod` | Add repo or append module instance config |
| `xrobot_src_man` | Multi-source module repository management utility |

Install: `pipx install xrobot` / `pip install xrobot`; source install from
`https://github.com/xrobot-org/XRobot`. There is also a VS Code extension `XRobot.xrobot`
providing a GUI configuration entry point that runs the generation commands.

`sources.yaml` is the optional module-source index; `xrobot_setup` creates a default one with
the official source already included, and `xrobot_src_man` manages mirrors.

---

## 8. Platform abstraction

### 8.1 The driver interface shape

**[SRC]** The lowest layer is not a class but **two function-pointer typedefs** (see §1.5):

```cpp
typedef ErrorCode (*WriteFun)(WritePort& port, bool in_isr);
typedef ErrorCode (*ReadFun)(ReadPort& port, bool in_isr);
```

A port is bound to a backend by assignment **[DOC]**:

```cpp
ReadPort &operator=(ReadFun fun);
WritePort &operator=(WriteFun fun);
```

Above that sits a family of abstract peripheral base classes. The common pattern, verified for
UART **[DOC]** (<https://xrobot-org.github.io/docs/basic_coding/driver/uart>):

```cpp
enum class Parity : uint8_t {
  NO_PARITY = 0,  // 无校验
  EVEN = 1,       // 偶校验
  ODD = 2         // 奇校验
};

struct Configuration {
  uint32_t baudrate;  // 波特率
  Parity parity;      // 校验模式
  uint8_t data_bits;  // 数据位长度
  uint8_t stop_bits;  // 停止位长度
};

template <typename ReadPortType = ReadPort, typename WritePortType = WritePort>
UART(ReadPortType* read_port, WritePortType* write_port);

virtual ErrorCode SetConfig(Configuration config) = 0;

template <typename OperationType>
ErrorCode Write(ConstRawData data, OperationType&& op, bool in_isr = false);

template <typename OperationType>
ErrorCode Read(RawData data, OperationType&& op, bool in_isr = false);
```

**Crucially, `UART` does *not* inherit from `ReadPort`/`WritePort`.** It *holds pointers* to
them (`read_port_`, `write_port_`) and forwards. The docs are explicit **[DOC]**: "`UART` 基类
当前只是保存 `read_port_` / `write_port_` 指针并把 `Read()` / `Write()` 转发过去，不负责创建、
拥有或释放这些端口对象；端口生命周期仍由调用方或具体平台实现管理." This is **composition, not
inheritance** — a significant difference from what one might assume. **[INFERENCE]** It is what
lets `Pipe` and a real UART expose the same `Port` interface.

The driver overview **[DOC]** (<https://xrobot-org.github.io/docs/basic_coding/driver>) states
the common recipe, with the caveat that not every driver has all parts:

- `Configuration` struct + `SetConfig()`
- `Read()` / `Write()` for stream or transactional use
- `Enable()` / `Disable()` where the hardware model needs it
- `Callback` registration for interrupt/async completion

and warns that `ADC`, `DAC`, `PowerManager`, `Timebase`, `Flash` are **narrower, specialised
interfaces** that should not be read as instances of one uniform template.

The complete peripheral family **[DOC]**: GPIO, UART, I2C, SPI, CAN/FDCAN, ADC, DAC, PWM, Flash,
Power, Timebase, Watchdog, USB. Doxygen confirms matching `Configuration` structs, e.g.
`GPIO::Configuration`, `I2C::Configuration`, `SPI::Configuration`, `PWM::Configuration`,
`CAN::Configuration`, `FDCAN::Configuration`, `Watchdog::Configuration` **[SRC]**.

Other verified driver facts **[DOC]**:
- `ReadPort(size_t buffer_size = 128)` — default 128-byte RX queue.
- `WritePort(size_t queue_size = 3, size_t buffer_size = 128)` — default 3 pending writes.
- `ReadPort::operator()(RawData, ReadOperation&, bool in_isr = false)` returns
  `ErrorCode::BUSY` if a read is already pending.
- `data.size_ == 0` means "read completes when **any** data is available"; for writes it means
  "return success immediately".
- `ReadPort::ProcessPendingReads(bool in_isr)` is the pump the driver calls after depositing
  bytes into the software queue.
- `WritePort::Stream` is an RAII batch-write helper: `Stream(WritePort*, WriteOperation)`,
  `operator<<`, `Commit()`, auto-commit on destruction. It refuses partial appends rather than
  splitting a segment, and will not write at all while unlocked.

### 8.2 How a new platform is added

**[SRC]** The repository layout makes the mechanism unambiguous. There are **two orthogonal
axes**, and both exist as real directories on `master`:

**Axis 1 — OS/system backend:** `system/`
Verified contents (<https://api.github.com/repos/Jiu-xiao/libxr/contents/system>):
`freertos`, `linux`, `none`, `threadx`, `webasm`, `webots`.

**Axis 2 — silicon/vendor driver backend:** `driver/`
Verified contents (<https://api.github.com/repos/Jiu-xiao/libxr/contents/driver>):
`ch` (CH32), `esp` (ESP32), `hpm` (HPM), `linux`, `mspm0` (TI MSPM0), `st` (STM32),
`webasm`, `webots`.

**[SRC]** `CMakeLists.txt` selects them via two cache variables:

```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/system/${_xr_system}
...
if(_xr_driver)
  target_sources(${PROJECT_NAME} PRIVATE ${${PROJECT_NAME}_DRIVER_C_SOURCES})

  target_include_directories(
    ${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/driver/${_xr_driver}
  )
endif()
```

**[DOC]** `cmake/config.cmake` is the file that defines `_xr_system` / `_xr_driver` — I did
**not** read it, so the exact option names and accepted values are a gap. **[INFERENCE]** from
the directory names, the values are presumably `freertos|linux|none|threadx|webasm|webots`
and `ch|esp|hpm|linux|mspm0|st|webasm|webots`.

**What a backend must provide** — established from source:

1. **OS primitives.** `libxr_system.hpp` per backend defines the handle typedefs. Linux
   version **[SRC]** (<https://jiu-xiao.github.io/libxr/libxr__system_8hpp_source.html>):

   ```cpp
   typedef pthread_mutex_t libxr_mutex_handle;
   struct libxr_linux_futex_semaphore
   {
     std::atomic<uint32_t> count;
   };
   typedef libxr_linux_futex_semaphore* libxr_semaphore_handle;
   typedef pthread_t libxr_thread_handle;

   void PlatformInit(uint32_t timer_pri = 2, uint32_t timer_stack_depth = 65536);
   ```

2. **`Timebase::GetMicroseconds()` / `GetMilliseconds()` definitions** — declared in the
   shared header, defined per backend (e.g. in `ch32_timebase.cpp`). **[SRC]**
3. **Per-driver `Configuration` + `SetConfig` + `Read`/`Write` + `ReadFun`/`WriteFun` binding.**
4. **The class naming convention [SRC]** (from Doxygen): `<Vendor><Peripheral>` — e.g.
   `STM32UART`, `STM32GPIO`, `STM32SPI`, `STM32I2C`, `STM32CAN`, `STM32CANFD`, `STM32ADC`,
   `STM32DAC`, `STM32PWM`, `STM32Flash`, `STM32PowerManager`, `STM32Watchdog`, `STM32Timebase`,
   `STM32TimerTimebase`, `STM32USBDevice`; and `ESP32UART`, `ESP32GPIO`, `ESP32SPI`, `ESP32I2C`,
   `ESP32PWM`, `ESP32ADC`, `ESP32DAC`, `ESP32Timebase`, `ESP32Watchdog`, `ESP32WiFiClient`,
   `ESP32NVSFlashDatabase`; plus `CH32*`, `HPM*`, `MSPM0*`, `LinuxUART`, `LinuxGPIO`,
   `LinuxTimebase`, `LinuxPowerManager`, `LinuxBinaryFileFlash`, `WebAsmTimebase`,
   `WebotsTimebase`.

**Is there a documented porting guide? [GAP]** I found **no** dedicated "porting to a new
platform" documentation page. The `driver` overview page describes the *interface* conventions
but not the porting procedure. The `Timer` page hints that "移植到新平台时，仅需保证 Thread 及
Timebase 支持，无需修改 Timer 主体逻辑" **[DOC]**, and the `system` index page names the
backends **[DOC]**: "LibXR 对不同系统后端（如 `linux / freertos / threadx / none`）的统一抽象层".
**[INFERENCE]** Porting is evidently done by copying a `system/<x>` + `driver/<x>` pair and
implementing the typedefs + Timebase; but this is inferred from layout, **not documented**.

### 8.3 Bare-metal ("none")

**[SRC]** `system/none` exists. **[DOC]** The `Timer` page describes the bare-metal strategy:
"裸机/单线程 | RefreshTimerInIdle 自动调用 | Thread 延时/Mutex/信号量等待时自动刷新". The
`ASync` page is more explicit about the degradation **[DOC]**:
"在裸机等无线程实现里，`ASync` 当前就是**同步直调**：`AssignJob()` 会直接调用 `job.Run()`，不会再创建
后台工作线程."

---

## 9. Language, standard, RTOS dependence, scheduling

### 9.1 Language and standard

**C++ — and it is not header-only.** **[SRC]** `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.12)

project(xr LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
...
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
```

- **Standard: C++20, required.** Confirmed by both the `CMAKE_CXX_STANDARD 20` and the
  `target_compile_features(... cxx_std_20)` line.
- The project declares `C CXX ASM` — C is present for vendor HAL glue and assembly startup,
  not for LibXR itself. Every LibXR source I read is `.hpp`/`.cpp`.
- It is **compiled, not header-only**: `target_sources(...)` globs `src/*.cpp`,
  `src/core/*.cpp`, `src/system/*.cpp`, `src/driver/*.cpp`, `src/utils/*.cpp`,
  `src/structure/*.cpp`, plus `src/structure/*/*.cpp`, `src/middleware/*.cpp`,
  `src/middleware/*/*.cpp`, `src/middleware/*/*/*.cpp`.
- Default library type is `STATIC`, with `LIBXR_SHARED_BUILD` / `LIBXR_STATIC_BUILD` /
  `LIBXR_OBJECT_BUILD` options.
- Uses real C++20 concepts (`CallbackFunctionCompatible`, `TopicPayload`,
  `MemberObjectPointer`, `CommonOrdered`, `DCacheFunctionAcceptsVoidPtr`), `requires` clauses,
  `std::apply`, `[[nodiscard]]` — so a C++20 compiler is a hard requirement **[SRC]**.
- Third-party: **Eigen** is vendored under `lib/Eigen` and linked by default; `LIBXR_NO_EIGEN`
  disables it. Kinematics/math utils (Axis, Quaternion, RotationMatrix, Inertia, CenterOfMass)
  derive from `Eigen::Matrix` **[SRC]**.
- `LIBXR_SINGLE_CORE` build flag exists and affects `CONCURRENCY_ALIGNMENT` **[SRC]**. The
  docs note: "`CONCURRENCY_ALIGNMENT`: 并发结构使用的对齐粒度；单核与多核配置下可能不同." **[DOC]**
- Special file suffixes drive per-file optimisation **[SRC]**: `*_o3.cpp` compiles at `-O3`
  (MSVC `/O2`), `*_os.cpp` at `-Os` (MSVC `/O1`). Used for e.g. `libxr_mem_o3.cpp`,
  `crc_o3.cpp` hot paths.
- `LIBXR_DEBUG_BUILD` is defined automatically for `CMAKE_BUILD_TYPE=Debug`, enabling
  `ASSERT`/`ASSERT_FROM_CALLBACK`. A separate `LIBXR_DEV_ASSERT_BUILD` gates maintainer-only
  assertions.
- Build types option set includes `LIBXR_TEST_BUILD` → `add_subdirectory(test)`, and
  `LIBXR_NO_EIGEN`.

### 9.2 Is an RTOS required?

**No.** It runs on RTOSes, on Linux, on bare metal, and even in WebAssembly and the Webots
simulator. **[SRC]** `system/` contains `freertos`, `threadx`, `linux`, `none` (bare metal),
`webasm`, `webots`. **[DOC]** the driver overview states: "核心模块当前依赖 C++20 特性和 LibXR
基础组件，适用于裸机和各种 RTOS 平台."

The backend is absorbed at the `Thread` / `Mutex` / `Semaphore` / `Timer` layer, so upper
layers are portable **[DOC]**: "`ASync` 本身不依赖特定 OS，所有平台差异已由 `Thread` 与
`Semaphore` 层吸收."

### 9.3 Does it have its own scheduler?

**It has a cooperative periodic-task layer, not a preemptive scheduler.**

Three things provide "scheduling":

1. **`ApplicationManager::MonitorAll()`** — the generated main loop calls it and then
   `Thread::Sleep(1000)`. This is the module scheduling mechanism: cooperative, round-robin,
   calling each `Application::OnMonitor()`. Period is set by
   `global_settings.monitor_sleep_ms` in `xrobot.yaml` **[DOC]**. **[INFERENCE]** Given the
   generated code sleeps 1000 ms per iteration, this is a *supervisory* health/status loop, not
   a control loop.

2. **`LibXR::Timer`** — a 1 ms-resolution software periodic task scheduler with a management
   thread (or `RefreshTimerInIdle` on bare metal) **[DOC]** (§5.4).

3. **`LibXR::Thread`** — a thin OS thread wrapper, not a scheduler. **Linux backend [SRC]**:

   ```cpp
   class Thread
   {
    public:
     enum class Priority : uint8_t
     {
       IDLE, LOW, MEDIUM, HIGH, REALTIME, NUMBER,
     };

     Thread() {};
     Thread(libxr_thread_handle handle) : thread_handle_(handle) {};

     template <typename ArgType>
     void Create(ArgType arg, void (*function)(ArgType arg), const char* name,
                 size_t stack_depth, Thread::Priority priority);

     static Thread Current(void);
     static uint32_t GetTime();
     static void Sleep(uint32_t milliseconds);
     static void SleepUntil(MillisecondTimestamp& last_waskup_time, uint32_t time_to_sleep);
     static void Yield();
     ErrorCode Join();

     operator libxr_thread_handle() { return thread_handle_; }
   };
   ```

   The Linux implementation uses `pthread_create` with `SCHED_FIFO` and
   `sp.sched_priority = min_priority + static_cast<int>(priority)`, falling back to
   `SCHED_OTHER` with a logged warning if `SCHED_FIFO` has insufficient range, and falling back
   further to plain `pthread_create` with default attributes if attribute setup fails. **[SRC]**

**`ASync` is a job runner, not a scheduler [DOC]** (<https://xrobot-org.github.io/docs/basic_coding/system/async>):

```cpp
class ASync {
public:
  enum class Status : uint8_t { READY, BUSY, DONE };
  ASync(size_t stack_depth, Thread::Priority priority);
  using Job = LibXR::Callback<ASync*>;
  ErrorCode AssignJob(Job job);                       // 任务上下文提交
  void       AssignJobFromCallback(Job job, bool isr);// ISR/回调上下文提交
  Status     GetStatus();                             // 查询状态并自动复位
};
```

One dedicated worker thread + counting semaphore per `ASync` instance; jobs execute serially.
Returns `ErrorCode::BUSY` if a job is already running. The docs note the naming rationale:
"设计理念中 Callback 不允许阻塞/延时，但是此处复用了 Callback 的接口与数据结构，为防止混淆重命名为
`Job`." On bare metal it degrades to a direct synchronous `job.Run()`. **[DOC]**

Also relevant **[DOC]** (<https://xrobot-org.github.io/docs/basic_coding/system>): the backend
pages "描述的是 LibXR 对不同系统后端（如 `linux / freertos / threadx / none`）的统一抽象层，
而不是承诺所有后端都共享完全相同的实现策略" — i.e. **semantics genuinely differ per backend**
for `Mutex` / `Semaphore` / `Timer` / `Thread` (priority inheritance, polling waits, thread
stubs). Any integration must verify the specific backend's behaviour.

### 9.4 Discipline rules implied by the design

**[DOC]** From the `ASync` page: "设计理念中 Callback 不允许阻塞/延时" — **callbacks must not
block or delay.** This is a stated design rule of the framework. **[INFERENCE]** It is the
reason `ASync` and `Job` exist as a separate concept, and it is a constraint any integration
must honour inside `in_isr` / callback paths.

---

## 10. Licence

**All three projects are Apache License 2.0.** Verified by fetching the actual `LICENSE` files.

| Project | Licence | Verified at |
|---|---|---|
| **LibXR** (`Jiu-xiao/libxr`) | **Apache-2.0** | <https://raw.githubusercontent.com/Jiu-xiao/libxr/master/LICENSE> — HTTP 200, full Apache 2.0 text, 11357 bytes |
| **XRobot** (`xrobot-org/XRobot`) | **Apache-2.0** | <https://raw.githubusercontent.com/xrobot-org/XRobot/master/LICENSE> — HTTP 200; the API listing confirms a `LICENSE` file of 11358 bytes on branch `XRobot2.0` |
| **LibXR_CppCodeGenerator** (`Jiu-xiao/LibXR_CppCodeGenerator`) | **Apache-2.0** | <https://raw.githubusercontent.com/Jiu-xiao/LibXR_CppCodeGenerator/master/LICENSE> — HTTP 200 |

Notes:
- Each file is the **unmodified stock Apache 2.0 text**; the `APPENDIX` still contains the
  literal placeholder `Copyright [yyyy] [name of copyright owner]`, i.e. **no copyright holder
  or year is filled in** in the licence file itself.
- `https://raw.githubusercontent.com/xrobot-org/XRobot/main/LICENSE` **404s** — the XRobot
  default branch is `XRobot2.0`, not `main`. Use the `master` URL above or `XRobot2.0`.
- **[GAP]** Neither the docs site footer (which shows only `Copyright © 2026 XRobot`) nor the
  About page (<https://xrobot-org.github.io/docs/about>) states a licence. The About page lists
  contributors but contains no licence text. I read those pages directly and am reporting the
  absence rather than assuming.
- Apache-2.0 imposes the usual obligations: retain notices, state changes, include a copy of
  the licence, and it includes an express patent grant. **[INFERENCE]** — standard licence
  property, not a project-specific claim.

---

## 11. Summary: what an integrator must know

The five facts that most shape an integration:

1. **`ReadFun`/`WriteFun` receive only `(port, in_isr)`.** Not the buffer, not the `Operation`.
   Payload and operation live on the port (`ReadPort::info_`, `ReadInfoBlock`). Any new backend
   must read them off the port.
2. **`Operation::BLOCK` carries no result.** The semaphore is a bare wakeup; the `ErrorCode`
   lives in the port's `block_result_`. And `POLLING` hard-codes `ErrorCode::OK` as success, so
   it only generalises to `Operation<ErrorCode>`.
3. **`Callback::Create` heap-allocates and nothing frees it.** `Callback` is a shallow-copying
   non-owning handle. The `in_isr` bool is a caller-supplied claim about context, threaded
   through every callback signature in the framework.
4. **`SPSCQueue` is genuinely lock-free SPSC**, owns a heap buffer, sacrifices one slot to
   distinguish full/empty, and stores payloads as **raw bytes** — the typed batch APIs
   `static_assert` trivial copyability and trivial destructibility.
5. **The XRobot module declaration is a YAML block inside a C++ comment** —
   `/* === MODULE MANIFEST V2 === ... === END MANIFEST === */` — and modules are composed by a
   generated `XRobotMain(LibXR::HardwareContainer&)` whose module constructors all take
   `(hw, appmgr, <constructor_args...>)`.

**Language/standard:** C++20, required, compiled (not header-only), CMake ≥ 3.12, Eigen vendored
by default.
**RTOS:** optional — FreeRTOS, ThreadX, Linux, bare metal (`none`), WebAssembly and Webots are
all real backends.
**Scheduler:** no preemptive scheduler of its own; a cooperative `ApplicationManager::MonitorAll()`
loop plus a 1 ms software `Timer`.
**Licence:** Apache-2.0 across LibXR, XRobot and the CodeGenerator.

---

## 12. Explicit gaps and unverified items

Reported rather than guessed:

- **No dedicated platform-porting guide was found.** The porting procedure in §8.2 is inferred
  from the repository layout and `CMakeLists.txt`, not read from a documented procedure.
- **`cmake/config.cmake` was not read**, so the exact CMake option/variable names and their
  accepted values for selecting `_xr_system` / `_xr_driver` are unverified.
- **Module dependency topological ordering is undocumented** on the pages I read. `depends:`
  exists in the MANIFEST and `xrobot_init_mod` claims recursive resolution, but the ordering
  algorithm is not stated.
- **`sources.yaml` schema is not shown** on any page I read — only that it is an optional
  module-source index and that a default with the official source is created for you.
- **`.config.yaml` full schema is not documented.** Only `device_aliases` was shown verbatim.
  Per-peripheral YAML keys are on deeper `code_gen/stm32/*` pages I did not read (GPIO's page
  turned out to document only the generated C++ shape, not YAML keys).
- **A `Module.cpp` generated body is never shown** — I only ever saw the `.hpp` MANIFEST and the
  `xrobot_main.hpp` instantiation site. What a module's own translation unit contains is unknown.
- **Doc pages I did not read:** `concept` (设计思想), `perf`, `env_setup`, `adv_coding`,
  `con_guide`, `debug`, `xrusb`, `intro`, the `system/{thread,mutex,semaphore}` doc pages, most
  `driver/*` doc pages (I read UART only), `middleware/{logger,database,ramfs,terminal}`,
  `structure/*` except `spsc_queue`, and most `code_gen/stm32/*` sub-pages. Where those topics
  are covered above, the content came from **source**, not from those pages.
- **`system/none` and `system/freertos` source was not read**; the bare-metal degradation
  behaviour is taken from the Timer and ASync documentation pages.
- `https://raw.githubusercontent.com/xrobot-org/XRobot/main/LICENSE` **404s** (branches are
  `master` / `XRobot2.0`).
- The Doxygen landing page (`/libxr/`) is JavaScript-driven and returns no content via plain
  fetch; I worked around this via `doxygen_crawl.html` and direct `*_source.html` URLs.
