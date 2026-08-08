# `mujoco_simulation` 日志模块完整设计方案

## 1. 文档状态

**版本：V1 Frozen**

本文定义 `mujoco_simulation` 日志模块 V1 的冻结设计，包括公共 API、日志宏、compile-time filtering、runtime admission、lazy stream、默认 `simulate.log`、backend 状态机、并发同步、运行期 sink failure、ABI、文本安全、Easylogging++ 隔离、CMake 和测试架构。

本文档通过实现验证、Fake Backend 状态机测试、shared-library consumer 测试以及 TSan 后，才升级为：

```text
V1 Frozen
```

---

# 2. 总体目标

业务代码只使用：

```cpp
#include <mujoco_simulation/log/logging.hpp>

SIM_DEBUG << "Debug value: " << value;
SIM_INFO  << "Simulation initialized";
SIM_WARN  << "Physics deadline missed";
SIM_ERROR << "Failed to load model: " << model_path;
SIM_FATAL << "mjModel is null";
```

或者：

```cpp
SIM_LOG(DEBUG) << "...";
SIM_LOG(INFO)  << "...";
SIM_LOG(WARN)  << "...";
SIM_LOG(ERROR) << "...";
SIM_LOG(FATAL) << "...";
```

Application 不感知 Easylogging++。

整体架构：

```text
application
        │
        ▼
mujoco_simulation/log/logging.hpp
        │
        ├── compile-time gate
        ├── runtime admission
        ├── lazy stream
        └── SourceLocation
        │
        ▼
inline LogMessageStream
        │
        ▼
impl::commit_message()
        │
        ▼
Logger
        │
        ├── RuntimePolicy
        ├── BackendState
        ├── retry state
        ├── failure state machine
        └── synchronization
        │
        ▼
BackendAdapter
        │
        ▼
EasyloggingAdapter
        │
        ▼
Easylogging++ PRIVATE
```

---

# 3. 目录结构

```text
mujoco_simulation/
├── include/
│   └── mujoco_simulation/
│       ├── export.hpp
│       └── log/
│           └── logging.hpp
│
├── src/
│   └── log/
│       ├── logging.cpp
│       └── impl/
│           ├── logging_state.hpp
│           ├── logger.hpp
│           ├── logger.cpp
│           ├── backend_adapter.hpp
│           ├── easylogging_adapter.hpp
│           ├── easylogging_adapter.cpp
│           ├── clock_adapter.hpp
│           └── steady_clock_adapter.hpp
│
├── third_party/
│   └── easyloggingpp/
│       ├── easylogging++.h
│       └── easylogging++.cc
│
└── tests/
    └── log/
        ├── fake_backend.hpp
        ├── fake_clock.hpp
        ├── logging_macro_test.cpp
        ├── logging_state_machine_test.cpp
        ├── logging_failure_test.cpp
        └── logging_concurrency_test.cpp
```

公共入口只有：

```cpp
#include <mujoco_simulation/log/logging.hpp>
```

`src/log/impl/` 下所有类型均不安装、不导出。

---

# 4. Easylogging++ 依赖边界

Easylogging++ 是完全 PRIVATE backend。

Application 和其他业务模块不得直接出现：

```cpp
#include <easylogging++.h>

LOG(...)
CLOG(...)

el::Logger
el::Loggers
el::Configurations
el::Level
```

Application 只链接：

```cmake
find_package(mujoco_simulation CONFIG REQUIRED)

target_link_libraries(
  simulation_app
  PRIVATE
    mujoco_simulation::mujoco_simulation
)
```

---

# 5. 公共命名空间

公共 API：

```cpp
namespace mujoco_simulation::logging {
}
```

公共头中为宏实现保留：

```cpp
namespace mujoco_simulation::logging::impl {
}
```

不使用：

```cpp
namespace detail
```

`impl` 不是普通用户 API，但 installed header 所引用的 exported symbols 构成 macro-support ABI。

---

# 6. `Level`

```cpp
enum class Level : std::uint8_t
{
    Debug = 10,
    Info  = 20,
    Warn  = 30,
    Error = 40,
    Fatal = 50,
    Off   = 255
};
```

等级关系：

```text
Debug < Info < Warn < Error < Fatal
```

`Off` 只允许作为 runtime threshold：

```cpp
mujoco_simulation::logging::set_level(
    mujoco_simulation::logging::Level::Off);
```

不存在：

```cpp
SIM_LOG(OFF)
```

---

# 7. `SourceLocation`

```cpp
struct SourceLocation
{
    const char* file;
    const char* function;
    std::int32_t line;
};
```

日志宏捕获：

```cpp
{
    __FILE__,
    __func__,
    static_cast<std::int32_t>(__LINE__)
}
```

V1 只承诺标准 `__func__` 的简单函数名。

例如：

```text
[simulation.cpp:142 initialize]
```

不承诺输出：

```text
Simulation::initialize
```

---

# 8. `Policy`

```cpp
struct Policy
{
    Level level{
        Level::Info};

    bool console_enabled{
        true};

    bool file_enabled{
        true};

    std::string file_path{
        "simulate.log"};

    bool colored_console{
        true};

    bool show_source_location{
        true};
};
```

`Policy` 的语义固定为：

> Requested Configuration。

它描述 application 希望使用的配置，不描述当前 sink 是否仍然正常工作。

---

# 9. 默认配置

默认：

```text
Level                   Info
Console requested       true
File requested          true
File path               simulate.log
File mode               append
Colored console         true
Source location         true
```

正常环境中：

```cpp
SIM_INFO << "Application started";
```

同时输出到：

```text
stderr
./simulate.log
```

---

# 10. 公共 API

```cpp
MUJOCO_SIMULATION_PUBLIC
bool configure(
    const Policy& policy) noexcept;

MUJOCO_SIMULATION_PUBLIC
bool set_level(
    Level level) noexcept;

MUJOCO_SIMULATION_PUBLIC
Level level() noexcept;

MUJOCO_SIMULATION_PUBLIC
bool is_enabled(
    Level level) noexcept;

MUJOCO_SIMULATION_PUBLIC
void flush() noexcept;

MUJOCO_SIMULATION_PUBLIC
const char* to_string(
    Level level) noexcept;
```

API 命名规则：

| 类型                 | API            |
| ------------------ | -------------- |
| 完整配置操作             | `configure()`  |
| 简单 property setter | `set_level()`  |
| 简单 property query  | `level()`      |
| predicate          | `is_enabled()` |
| action             | `flush()`      |
| conversion         | `to_string()`  |

不增加：

```text
get_level()
get_config()
set_config()
ConfigureResult
last_error()
```

---

# 11. `to_string()` 契约

固定结果：

| 输入             | 返回值         |
| -------------- | ----------- |
| `Level::Debug` | `"Debug"`   |
| `Level::Info`  | `"Info"`    |
| `Level::Warn`  | `"Warn"`    |
| `Level::Error` | `"Error"`   |
| `Level::Fatal` | `"Fatal"`   |
| `Level::Off`   | `"Off"`     |
| 非法枚举值          | `"Unknown"` |

返回字符串具有 static storage duration、NUL terminated，调用方不得释放。

---

# 12. Public Level 输入校验

合法 threshold：

```text
Debug
Info
Warn
Error
Fatal
Off
```

合法 message severity：

```text
Debug
Info
Warn
Error
Fatal
```

因此：

```cpp
is_enabled(Level::Off);
```

固定返回：

```text
false
```

非法 enum 同样返回 false。

---

# 13. `commit_message()` 的 Severity 防御

即使调用者直接调用 macro-support ABI：

```cpp
impl::commit_message(
    Level::Off,
    ...);
```

或者传入非法 enum：

```cpp
impl::commit_message(
    static_cast<Level>(123),
    ...);
```

都必须：

```text
不触发 bootstrap
不进入 backend
不产生 normal record
```

并允许通过限频 infrastructure fallback 报告 internal contract violation。

---

# 14. `colored_console`

Console sink 固定写：

```text
stderr
```

所有 severity 都使用同一个 Console destination。

`colored_console == false`：

```text
stderr 永远不输出 mujoco_simulation 生成的 ANSI color sequence
```

`colored_console == true`：

```text
stderr 是 interactive TTY
    → 必须启用 severity color

stderr 不是 TTY
    → 不输出 ANSI color
```

目标 Linux 平台使用：

```text
isatty(STDERR_FILENO)
```

或等价机制判断。

具体颜色值不是兼容性契约。

---

# 15. File 永不包含颜色

无论：

```cpp
colored_console
```

取何值：

```text
File sink 永远不包含 ANSI color sequence
```

---

# 16. `show_source_location`

为 true 时：

```text
[file:line function] message
```

例如：

```text
[imu.cpp:82 initialize] IMU 'base_imu': sensor name must not be empty
```

为 false 时：

```text
IMU 'base_imu': sensor name must not be empty
```

整个 SourceLocation prefix 被省略。

Console 和 File 使用一致的 SourceLocation policy。

---

# 17. 输出 Record 的稳定契约

V1 保证每条成功输出的日志具有以下信息：

```text
timestamp
severity
thread identifier
optional SourceLocation
business message
exactly one physical line terminator
```

Severity 文本固定为：

```text
DEBUG
INFO
WARN
ERROR
FATAL
```

Timestamp 使用本地 wall-clock time，并至少提供毫秒精度。

Thread identifier 是实现定义的可读标识。

每条 record 最终恰好追加一个物理：

```text
'\n'
```

业务 message 和 SourceLocation 内部的 newline 已经在进入 backend 前转义，因此不能额外制造物理行。

---

# 18. 输出格式兼容性边界

默认可以采用：

```text
2026-08-08 21:15:12.125 [INFO] [12345] [simulation.cpp:142 initialize] Simulation initialized
```

但 timestamp、thread identifier 的具体分隔符和 lexical rendering：

> 不属于 Public C++ ABI，也不作为 V1 machine-readable protocol。

Application 不应通过固定字符位置解析日志。

V1 稳定保证的是 record 字段语义、severity 名称、单行边界、SourceLocation 开关和颜色行为。

---

# 19. 空消息

以下均表示 empty business message：

```text
message == nullptr
message_size == 0
```

以及：

```text
message != nullptr
message_size == 0
```

Enabled 状态下仍产生一条正常 record。

如果 SourceLocation 开启，则 record 仍包含 source prefix。

如果 SourceLocation 关闭，则仍包含 timestamp/severity/thread 和最终换行。

因此：

```cpp
SIM_INFO;
```

行为明确且合法，但业务代码不推荐主动使用。

---

# 20. File Sink 永远 Append

V1 没有 file mode 字段。

因此：

> 默认 bootstrap 和 explicit `configure()` 的所有 File sink 永远采用 append。

V1 永远不会通过日志配置 truncate 已存在文件。

---

# 21. 相对 File Path

相对路径在每次实际：

```text
automatic bootstrap attempt
automatic retry attempt
explicit configure attempt
```

开始时，按照当时的 process CWD 解析。

本次 attempt 必须保存解析后的稳定目标，并在本次：

```text
validation
preflight
backend configuration
```

中始终使用同一个 resolved path。

---

# 22. File 打开后的 `chdir()`

File sink 成功打开后：

```cpp
chdir(...);
```

不会改变已打开 File sink 的目标。

只有下一次重新打开 File sink 时才重新解析相对路径。

---

# 23. `file_enabled == false`

当：

```cpp
config.file_enabled == false;
```

时：

```text
file_path 完全忽略
```

不进行 empty、NUL、filesystem、parent 或 target-type 检查。

因此：

```cpp
Policy policy;

config.file_enabled = false;
config.file_path.clear();
```

合法。

---

# 24. `file_enabled == true`

此时要求：

```text
file_path 非空
file_path 不包含 embedded NUL
parent directory 存在
parent 是 directory
已存在目标在 preflight 时属于允许类型
路径可被 append/open
```

否则 explicit：

```cpp
configure(config)
```

返回 false。

---

# 25. POSIX Path 字节语义

Ubuntu/Linux V1 中：

```text
file_path 按 POSIX pathname bytes 处理
```

除：

```text
NUL
```

外不要求 UTF-8 合法，不执行 Unicode normalization。

新文件权限由：

```text
process umask
+
backend/open implementation
```

共同决定。

---

# 26. File Target 类型

Preflight 对已存在目标允许：

```text
regular file
symlink 最终解析到 regular file
```

拒绝已知：

```text
directory
FIFO
socket
character device
block device
dangling symlink
其他非 regular target
```

这样可以避免正常配置误把日志写向特殊对象。

---

# 27. File Type 检查的 TOCTOU 边界

V1 明确承认：

> 路径类型 preflight 不是对 hostile filesystem namespace 的安全边界。

Preflight 与最终 backend open 之间目标仍可能被其他进程替换。

Production Backend Adapter 应在可行时对实际打开对象进行再次验证，或者使用不会无界阻塞特殊文件的安全打开方式。

如果当前 Easylogging++ backend 无法对最终 descriptor/handle 做可靠验证，则 V1 只能保证：

```text
已知危险目标在 preflight 时被拒绝
```

而不能宣称：

```text
在攻击者并发替换 pathname 时绝对不可能打开特殊对象
```

因此默认日志目录假定属于非对抗性的可信部署环境。

---

# 28. Automatic Bootstrap 也使用相同 File 规则

File Path 的 empty/NUL/parent/type/append 规则不仅适用于：

```cpp
configure()
```

同样适用于 automatic bootstrap。

第一次可输出的日志会尝试一次默认 `Policy`。该尝试与显式 `configure()`、`set_level()` 使用同一策略临界区：如果显式操作已经生效，自动尝试不会覆盖它。默认尝试失败时丢弃该条日志，后续仅显式 `configure()` 可以恢复输出；不使用 retry、Faulted 或逐 sink 降级状态。

---

# 29. Preflight 副作用

Preflight 可以通过 non-truncating append/open probe 验证可写性。

目标不存在时：

> 允许 preflight 创建空 regular file。

如果后续 backend commit 失败：

> V1 不保证删除该空文件。

内部状态的 strong guarantee 不等于 filesystem 零副作用。

---

# 30. 日志宏公共形式

支持：

```cpp
SIM_LOG(DEBUG) << ...;
SIM_LOG(INFO)  << ...;
SIM_LOG(WARN)  << ...;
SIM_LOG(ERROR) << ...;
SIM_LOG(FATAL) << ...;
```

以及：

```cpp
SIM_DEBUG << ...;
SIM_INFO  << ...;
SIM_WARN  << ...;
SIM_ERROR << ...;
SIM_FATAL << ...;
```

二者严格映射同一个 severity path。

---

# 31. Severity Token

`SIM_LOG()` 只接受：

```text
DEBUG
INFO
WARN
ERROR
FATAL
```

不接受：

```text
WARNING
TRACE
OFF
runtime Level variable
```

---

# 32. Severity 宏污染防护

即使外部存在：

```cpp
#define ERROR 0
#define DEBUG 1
#define INFO  2
```

以下仍必须正确：

```cpp
SIM_LOG(ERROR) << "...";
SIM_LOG(DEBUG) << "...";
SIM_LOG(INFO)  << "...";
```

因此 dispatch 必须直接：

```cpp
#define SIM_LOG(level_token) \
    SIMULATE_LOG_##level_token
```

不得增加 severity forwarding expansion 层。

---

# 33. 快捷宏

```cpp
#define SIM_DEBUG \
    SIMULATE_LOG_DEBUG

#define SIM_INFO \
    SIMULATE_LOG_INFO

#define SIM_WARN \
    SIMULATE_LOG_WARN

#define SIM_ERROR \
    SIMULATE_LOG_ERROR

#define SIM_FATAL \
    SIMULATE_LOG_FATAL
```

---

# 34. Logging Macro 保留前缀

内部宏统一：

```text
SIMULATE_LOG_*
```

公共 header 对自己实际定义的：

```text
SIM_LOG
SIM_DEBUG
SIM_INFO
SIM_WARN
SIM_ERROR
SIM_FATAL
SIM_LOG_LEVEL_*
SIMULATE_LOG_*
```

执行 include 前 collision check。

唯一允许 application 在 header 前主动定义：

```text
SIM_LOG_COMPILED_LEVEL
```

Header 无法阻止 include 后重新定义，因此调用方不得在之后覆盖这些保留名称。

---

# 35. Compile-Time Level

```cpp
#define SIM_LOG_LEVEL_DEBUG 1
#define SIM_LOG_LEVEL_INFO  2
#define SIM_LOG_LEVEL_WARN  3
#define SIM_LOG_LEVEL_ERROR 4
#define SIM_LOG_LEVEL_FATAL 5
#define SIM_LOG_LEVEL_NONE  6
```

使用 1～6 而不是 0～5，是为了让：

```cpp
#define SIM_LOG_COMPILED_LEVEL INVALID_TOKEN
```

在 `#if` 中退化为 0 后被范围校验拒绝。

---

# 36. 默认 Compile Level

```cpp
#ifndef SIM_LOG_COMPILED_LEVEL

#ifdef NDEBUG

#define SIM_LOG_COMPILED_LEVEL \
    SIM_LOG_LEVEL_INFO

#else

#define SIM_LOG_COMPILED_LEVEL \
    SIM_LOG_LEVEL_DEBUG

#endif
#endif
```

---

# 37. Compile-Level Validation

```cpp
#if SIM_LOG_COMPILED_LEVEL < \
        SIM_LOG_LEVEL_DEBUG || \
    SIM_LOG_COMPILED_LEVEL > \
        SIM_LOG_LEVEL_NONE

#error "invalid SIM_LOG_COMPILED_LEVEL"

#endif
```

以下必须 compile fail：

```text
-1
99
INVALID_TOKEN
```

---

# 38. Lazy Stream

必须保证：

```cpp
SIM_DEBUG
    << expensive_function();
```

runtime disabled 时：

```text
expensive_function() 不执行
LogMessageStream 不构造
ostringstream 不构造
backend 不访问
```

不能用 NullStream 实现。

---

# 39. `LogMessageStream::Expression`

```cpp
class LogMessageStream::Expression final
{
public:
    void operator&(
        std::ostream&) const noexcept
    {
    }
};
```

Lazy macro 采用 conditional/Voidify 模式。

---

# 40. Lazy Macro

概念：

```cpp
#define SIMULATE_LOG(level)                                          \
    !::mujoco_simulation::logging::is_enabled(level) ? (void)0 :    \
        ::mujoco_simulation::logging::impl::LogMessageStream::Expression{} & \
        ::mujoco_simulation::logging::impl::LogMessageStream(       \
            level, {__FILE__, __func__, static_cast<std::int32_t>(__LINE__)}).stream()
```

宏必须通过 precedence、dangling-else 和 temporary-lifetime 测试。

---

# 41. `LogMessageStream`

`LogMessageStream` 完全 inline 存在于 `logging.hpp` 的 `impl` 宏支撑区块，
不构成直接调用契约。

`libmujoco_simulation.so` 不：

```text
构造 LogMessageStream
析构 LogMessageStream
读取其 layout
接收 LogMessageStream*
```

因此 LogMessageStream object layout 不跨 DSO。

---

# 42. Frontend Exception 契约

这是一个重要边界：

> `SIM_* << ...` 不是 `noexcept` 表达式。

当日志 enabled 时，以下 frontend 操作可能抛异常：

```text
std::ostringstream 构造
stream buffer allocation
std::string allocation
用户 operator<<
用户表达式本身
```

这些发生在 exported `noexcept` backend 边界之前，因此允许正常向调用方传播。

换言之：

```text
backend/core infrastructure
    不向 application 抛异常

enabled C++ stream frontend
    仍具有普通 C++ stream expression 的异常语义
```

如果未来要求“任何日志调用都绝不能改变业务控制流”，需要重新设计固定容量或无异常 frontend，不属于 V1。

---

# 43. User Expression Exception

例如：

```cpp
SIM_INFO
    << function_that_throws();
```

原异常正常传播。

日志框架不吞业务异常。

---

# 44. Stack Unwinding

`LogMessageStream` 构造时记录：

```cpp
std::uncaught_exceptions();
```

如果析构时发现正在异常展开：

```text
partial business record 丢弃
```

不得提交已经构造的 prefix。

---

# 45. Stream Error State

如果 stream 没有抛异常，但设置：

```text
failbit
badbit
```

业务 record 完整丢弃。

不输出额外诊断，避免日志前端在失败路径引入新的导出 ABI 或递归风险。

---

# 46. Macro-Support ABI

```cpp
namespace mujoco_simulation::logging::impl {

MUJOCO_SIMULATION_PUBLIC
void commit_message(
    Level level,
    SourceLocation location,
    const char* message,
    std::size_t message_size) noexcept;

}
```

该函数不是普通用户 API，但属于 installed consumer binary dependency。

---

# 47. Message Pointer 生命周期

`commit_message()` 是 synchronous consumption API。

```text
message
    指向至少 message_size 个可读字节
```

实现必须在函数返回前完成所需消费或复制。

不得：

```text
保存 raw pointer
函数返回后访问
异步队列 raw pointer
```

---

# 48. Message Pointer 参数

合法：

```text
non-null + size > 0
non-null + size == 0
nullptr  + size == 0
```

非法：

```text
nullptr + size > 0
```

非法组合：

```text
不 dereference
不 bootstrap
不 backend write
限频 contract fallback
return
```

---

# 49. Message Length

`message_size` 是业务消息唯一权威长度。

禁止：

```cpp
strlen(message)
```

用于确定业务消息长度。

因此 embedded NUL 必须完整处理。

---

# 50. Business Message Sanitization

进入 backend 前对业务 message 执行 length-aware sanitization。

| 原始字节   | 输出文本   |
| ------ | ------ |
| `0x00` | `\0`   |
| TAB    | `\t`   |
| LF     | `\n`   |
| CR     | `\r`   |
| ESC    | `\x1B` |
| 其他 C0  | `\xHH` |
| DEL    | `\x7F` |
| 其他字节   | 原样     |

输出中的 `\n` 是两个可见字符，不是换行。

---

# 51. SourceLocation Sanitization

`SourceLocation::file` 和：

```text
SourceLocation::function
```

同样必须进行控制字符 sanitization。

处理顺序：

```text
file
→ safe bounded read
→ basename
→ sanitize

function
→ safe bounded read
→ sanitize
```

这样 macro-support ABI 的直接调用者也无法利用：

```text
LF
CR
ESC
C0
```

制造多行日志或 terminal escape injection。

---

# 52. SourceLocation 输入上限

普通业务路径可以分配并处理正常长度的 `__FILE__` / `__func__`。

但 internal implementation 必须对异常超长的直接 ABI 输入设置合理的 checked-size 上限。

超过实现允许的最大安全长度时：

```text
business record 丢弃
infrastructure fallback
```

不得无界读取不可信 C string。

---

# 53. Checked Size Calculation

以下全部使用 checked arithmetic：

```text
sanitized message
sanitized file
sanitized function
line representation
source prefix
final record body
```

任何：

```text
size_t overflow
length_error
bad_alloc
```

都意味着：

```text
不提交 partial record
不提交 truncated record
fallback
return
```

---

# 54. Runtime 状态分层

最终状态严格分成：

```text
Requested Policy
Active Sinks
BackendState
Runtime Policy
Retry Deadline
Backend Generation
```

职责：

| 状态                   | 含义                    |
| -------------------- | --------------------- |
| `requested_config`   | Application 希望使用什么    |
| Active Sink bits     | 当前允许继续尝试写哪些 sink      |
| `BackendState`       | Backend 生命周期状态        |
| `runtime_policy`     | 无锁 admission snapshot |
| retry deadline       | 下一次自动恢复允许时间           |
| `backend_generation` | 当前物理 backend 配置代次     |

---

# 55. `BackendState`

```cpp
enum class BackendState : std::uint8_t
{
    Uninitialized,
    Attempting,
    Ready,
    RetryPending,
    Faulted
};
```

该类型完全 internal，不属于 ABI。

---

# 56. State 含义

| State           | Active Sink | Retry | 含义                              |
| --------------- | ----------: | ----: | ------------------------------- |
| `Uninitialized` |        none |   yes | 尚未 bootstrap                    |
| `Attempting`    |        none |    no | 某线程拥有 bootstrap/retry ownership |
| `Ready`         |    one/both |    no | 正常 backend                      |
| `Ready`         |        none |    no | 显式 no-sink 配置                   |
| `RetryPending`  |        none |   yes | 无 sink，等待 automatic retry       |
| `Faulted`       |        none |    no | Backend integrity 不可信           |

---

# 57. Runtime Policy

```cpp
using PolicyWord =
    std::uint32_t;

std::atomic<PolicyWord>
    runtime_policy;
```

建议布局：

```text
bits 0..7    Level
bit 8        active_console_sink
bit 9        active_file_sink
bit 10       show_source_location
bit 11       bootstrap_retry_required
```

这是：

> single atomic runtime policy snapshot。

不承诺它在所有平台 lock-free。

---

# 58. Initial Policy

初始：

```text
Level                 Info
Active Console        false
Active File           false
Show Source           true
Retry                 true
BackendState          Uninitialized
```

Backend 尚未建立，因此不能预先把 requested sink 当作 Active sink。

---

# 59. Backend 同步模型最终简化

V1 不再使用：

```cpp
std::shared_mutex
```

统一使用：

```cpp
std::mutex backend_mutex;
```

原因是 logging 本身不是高频 realtime primitive，而普通日志最终仍要执行 formatting/I/O。

串行 backend operation 可以换来更明确的：

```text
单进程 record atomicity
generation 同步
configure/write/flush/FATAL 锁序
运行期 sink failure update
```

---

# 60. `backend_mutex` 的职责

以下所有实际 backend 操作必须持有同一个：

```cpp
backend_mutex
```

包括：

```text
normal write
FATAL write + flush
flush
bootstrap
configure
reset
runtime sink deactivate
```

因此同一 process 内不会有两个 mujoco_simulation backend operations 并行执行。

---

# 61. 单进程 Record Atomicity

因为所有 `BackendAdapter::write()` 都由同一个 backend mutex 串行调用：

> 同一 process 内，由 `mujoco_simulation` 发出的两条日志不会在同一 sink 上发生字节级交错。

多线程日志形成一个：

```text
backend-mutex acquisition order
```

定义的提交总序。

不保证该顺序与：

```text
日志表达式开始时间
Source timestamp 生成时间
线程调度顺序
```

完全一致。

如果某条 record 同时成功写入 Console/File，则两 sink 上相对于其他成功双写 record 的提交顺序一致。

---

# 62. Mutex Fairness

`std::mutex` 仍不保证标准意义上的：

```text
FIFO fairness
bounded waiting
wait-free progress
```

因此 V1 只承诺：

```text
thread safety
deadlock-free under documented lock order
```

不承诺 configure/flush/FATAL 在任意恶意持续竞争情况下存在严格的最大等待时间。

删除 `shared_mutex` 后，至少不再存在持续 shared-reader admission 导致 writer 特有饥饿的问题。

---

# 63. 锁顺序

唯一允许的双锁顺序：

```text
config_mutex
    ↓
backend_mutex
```

禁止：

```text
backend_mutex
    ↓
config_mutex
```

普通 write 只持：

```text
backend_mutex
```

`set_level()` 只持：

```text
config_mutex
```

---

# 64. `backend_generation` 的锁归属

`backend_generation` 不再由 `config_mutex` 单独保护。

最终规则：

> `backend_generation` 由 `backend_mutex` 保护。

```cpp
std::uint64_t
    backend_generation{0};
```

Normal write 在持有 backend mutex 时读取 generation。

Configure/bootstrap/reset/sink-deactivation 在持有 backend mutex 时修改 generation。

Runtime failure handler 按：

```text
config_mutex
→ backend_mutex
→ compare generation
```

检查 stale failure。

这样普通 write 不需要在 backend lock 内获取 config mutex。

---

# 65. Generation 更新

只要物理 backend 配置被成功切换到一个新的 coherent generation，就递增：

```text
bootstrap success
retry success
explicit configure success
successful sink deactivate
Faulted reset success
```

即使 Faulted reset 成功、随后 candidate configure 失败，reset 仍改变了 backend generation，因此 generation 必须更新。

---

# 66. Runtime Publication 模型

以下字段只在 `config_mutex` 下访问：

```text
BackendState
requested_config
```

`backend_generation` 在 backend mutex 下访问。

无锁 reader 只读取：

```text
runtime_policy
retry_deadline_ns
```

---

# 67. Publication 顺序

在需要发布新 runtime state 时：

```text
1. 在对应锁内完成 backend/state mutation
2. 更新 BackendState / Requested Policy
3. retry_deadline_ns.store(..., relaxed)
4. runtime_policy.store(..., release)
```

`runtime_policy.store(release)`：

> 必须作为无锁 runtime state 的最后 publication。

---

# 68. 无锁 Reader 顺序

无锁入口首先：

```cpp
const auto policy =
    runtime_policy.load(
        std::memory_order_acquire);
```

只有：

```text
policy.retry == true
```

才读取：

```cpp
retry_deadline_ns.load(
    std::memory_order_relaxed);
```

最终 retry ownership 仍必须经过：

```text
config_mutex
+
state recheck
```

所以 admission snapshot 即使因竞争略旧，也不会造成重复 backend attempt。

---

# 69. Retry Clock

内部定义：

```cpp
using RetryTick =
    std::int64_t;
```

含义固定为：

> `steady_clock::time_since_epoch()` 转换后的纳秒数。

转换结果饱和到：

```text
[0, INT64_MAX]
```

Retry interval：

```text
1,000,000,000 ns
```

Deadline 使用 saturating addition：

```text
deadline =
    saturating_add(
        now,
        1'000'000'000)
```

避免整数溢出。

---

# 70. Clock Adapter

内部：

```cpp
class ClockAdapter
{
public:
    virtual ~ClockAdapter() = default;

    virtual RetryTick now_ns()
        noexcept = 0;
};
```

Production 使用：

```text
SteadyClockAdapter
```

测试使用：

```text
FakeClock
```

Retry test 不允许依赖真实：

```text
sleep_for(1s)
```

---

# 71. `is_enabled()`

正式语义：

> 当前 severity 是否值得进入日志构造路径。

它不等于：

> 当前一定存在可立即写入的 sink。

概念：

```cpp
if (!valid_message_level(level))
    return false;

policy = acquire_load();

if (!severity_enabled(level, policy))
    return false;

if (policy.has_active_sink())
    return true;

if (!policy.retry_required())
    return false;

deadline = relaxed_load();

return clock.now_ns() >= deadline;
```

---

# 72. Retry Backoff

Default bootstrap 所有 requested sinks 均失败后：

```text
BackendState = RetryPending
Active sinks = none
Retry = true
deadline = now + 1 second
```

Backoff 内：

```text
is_enabled() == false
```

因此不构造 RHS。

Deadline 到期后，下一条 severity-enabled 日志可以尝试获取 retry ownership。

---

# 73. `Attempting` 防惊群

第一个 retry owner 获得 `config_mutex` 后必须立即重新检查状态。

确认仍应 retry 时：

```text
BackendState = Attempting
Active sinks = none
Retry = false
```

随后立即：

```cpp
runtime_policy.store(
    attempting_policy,
    std::memory_order_release);
```

再执行慢速：

```text
path resolution
filesystem probe
backend lock
backend configure
```

之后进入的新日志会看到：

```text
no active sink
retry=false
```

因此不再执行 RHS。

---

# 74. 已经通过 Admission 的 In-Flight Log

在 Attempting publication 前已经通过：

```cpp
is_enabled()
```

的其他线程，仍可能：

```text
完成 RHS
构造 LogMessage
进入 commit_message()
```

这些日志允许在 commit 时发现：

```text
Attempting
```

后直接 drop。

V1 不试图撤销已经发生的 RHS。

---

# 75. Attempt 必须存在终结 Guard

一旦 publish：

```text
BackendState = Attempting
```

当前 owner 必须建立一个内部 no-throw scope guard。

该 guard 保证 owner 无论通过：

```text
正常返回
path failure
allocation failure
filesystem exception
backend mutex lock failure
adapter failure
unexpected exception
```

退出，都不能留下：

```text
Attempting
+
no active sink
+
retry=false
```

永久状态。

---

# 76. Attempt Terminal Rule

Attempt 必须最终进入：

```text
Ready
RetryPending
Faulted
```

其中：

```text
普通 path/resource/open failure
且 backend integrity known
    → RetryPending 或 partial Ready

backend mutation 后 integrity unknown
    → Faulted
```

Scope guard 默认终态：

```text
如果尚未发生 backend integrity risk
    → RetryPending

如果已经标记 backend integrity unknown
    → Faulted
```

Guard destructor 必须：

```text
noexcept
no allocation
```

---

# 77. `configure()` 的公共语义

正式定义：

> `configure()` 是完整的 Policy 切换操作。

返回 true 表示 candidate 的全部 Requested Configuration 已成功建立并发布。

返回 false 时，candidate 不会成为新的 Policy，已经激活的 Policy 和输出目标保持可用。

---

# 78. `configure(false)` 的保守解释

Application 可以把：

```cpp
configure(policy) == false
```

解释为：

```text
candidate 未生效
旧 Policy 与旧输出继续生效
```

Application 可以修正配置后再次调用：

```cpp
configure(...)
```

尝试恢复。

---

# 79. Configure Preparation Phase

在任何 backend mutation 前完成：

```text
Level validation
Policy deep copy
file-path validation
relative-path resolution
filesystem preflight
backend candidate strings
所有可能分配的 frontend/config resources
```

所有可预见的 throwing preparation 必须发生在 backend mutation 前。

---

# 80. Preparation Strong Guarantee

如果 Preparation 阶段失败：

```text
configure() == false

requested_config unchanged
runtime_policy unchanged
BackendState unchanged
backend_generation unchanged
backend unchanged
```

这里只允许已有的 filesystem preflight 副作用，例如创建空日志文件。

---

# 81. No-Throw Policy Publication

Backend commit 成功以后，不得再执行可能分配的：

```cpp
requested_config = candidate;
```

必须提前构造：

```cpp
Policy prepared_policy;
```

最终通过经过验证的 no-throw memberwise swap 提交。

支持 toolchain 必须验证：

```cpp
noexcept(
    std::declval<std::string&>().swap(
        std::declval<std::string&>()))
```

成立。

---

# 82. Backend Configure Result

Internal Backend Adapter 必须能表达：

```cpp
enum class BackendMutationStatus
{
    Success,
    FailureOldStatePreserved,
    FailureBackendFaulted
};
```

该枚举完全 internal，不是 Public `ConfigureResult`。

---

# 83. Explicit Configure Success

成功后：

```text
prepared requested_config no-throw publication
BackendState = Ready
Active sink mask = requested sink mask
Retry = false
backend_generation++
runtime_policy release publication
```

显式：

```text
Console=false
File=false
```

也是合法 Ready 状态：

```text
Active sinks=none
Retry=false
```

---

# 84. Configure Failure：Old State Preserved

如果 adapter 明确保证 backend 完全未改变：

```text
configure() = false
requested_config unchanged
BackendState unchanged
Active mask unchanged
generation unchanged
old backend remains usable
```

---

# 85. Configure Failure：Backend Faulted

如果 backend 已部分改变，且旧状态不能被可靠证明：

```text
configure() = false

requested_config 仍保留旧值

BackendState = Faulted
Active sinks = none
Retry = false

runtime_policy release publication
```

不再声称 old backend 可用。

---

# 86. Explicit Configure 不先 Bootstrap

如果 application 第一项日志操作是：

```cpp
configure(custom_config);
```

不得先建立默认：

```text
./simulate.log
```

显式 configure 直接负责建立 candidate backend。

---

# 87. Faulted Recovery

Faulted 下 ordinary logging：

```text
is_enabled() == false
```

只允许 explicit：

```cpp
configure(...)
```

尝试恢复。

恢复时首先：

```cpp
backend.reset_to_safe_baseline();
```

---

# 88. Faulted Reset Failure

如果 reset 失败：

```text
configure() = false
BackendState = Faulted
Active sinks = none
Retry = false
requested_config unchanged
```

---

# 89. Faulted Reset Success

Reset 成功意味着 backend 已进入 safe baseline：

```text
backend_generation++
```

随后继续 strict candidate configure。

---

# 90. Faulted Reset Success + Candidate Failure

如果 safe baseline 已建立，但 candidate configure 又返回：

```text
FailureOldStatePreserved
```

这里所谓 preserved state 是：

```text
safe baseline
```

而不是 Faulted 前的旧 backend。

最终：

```text
configure() = false

requested_config 仍保持旧值
BackendState = Faulted
Active sinks = none
Retry = false

safe baseline 可以保持
```

如果 candidate success：

```text
Faulted → Ready
generation 再次递增
```

---

# 91. Backend Adapter

Production 状态机不能直接散落 Easylogging++ 调用。

内部接口概念：

```cpp
class BackendAdapter
{
public:
    virtual ~BackendAdapter() = default;

    virtual ResetResult
    reset_to_safe_baseline() noexcept = 0;

    virtual ConfigureResult
    configure(
        const BackendConfig& config,
        ApplyMode mode) noexcept = 0;

    virtual WriteResult
    write(
        Level level,
        const char* record,
        std::size_t size) noexcept = 0;

    virtual FlushResult
    flush(
        SinkMask sinks) noexcept = 0;

    virtual BackendMutationStatus
    deactivate_sinks(
        SinkMask sinks) noexcept = 0;
};
```

这些类型全部 internal。

---

# 92. `deactivate_sinks()` 是必须能力

仅仅清除 Runtime Policy bit 不足以停用一个失败 sink。

例如 backend 仍物理配置：

```text
Console + File
```

而 policy 只改：

```text
File inactive
```

下一次全局 backend write 仍可能继续写 File。

因此 runtime failure handler 必须真正调用：

```cpp
deactivate_sinks(failed_sinks);
```

只有 backend mutation 成功后，才能发布新的 Active Sink bits。

---

# 93. Runtime Sink Deactivation Failure

如果已知 File sink 失败，但：

```cpp
deactivate_sinks(File)
```

失败，无论 adapter 返回：

```text
old backend preserved
backend integrity unknown
```

V1 都不能继续把 backend 视为一个与 Runtime Policy 一致的安全 Ready 状态。

因此最终统一：

```text
deactivation failure
    → Faulted
    → Active sinks none
    → Retry false
```

原因是 old backend 中仍包含已知故障 sink。

---

# 94. Backend Configure Mode

Automatic default bootstrap 使用：

```text
BestEffortDefault
```

允许：

```text
Console success
File failure
→ Console-only Ready
```

Explicit configure 使用：

```text
Strict
```

必须满足全部 requested sink，否则返回：

```text
FailureOldStatePreserved
或
FailureBackendFaulted
```

不能 silently degrade。

---

# 95. Runtime Write Result

至少表达：

```cpp
struct WriteResult
{
    bool global_integrity_failure;

    SinkMask written_sinks;
    SinkMask failed_sinks;
};
```

这样 core 可以区分：

```text
Console failure
File failure
last sink failure
global failure
```

---

# 96. Runtime Sink Failure 流程

Normal write：

```text
backend_mutex
→ capture backend_generation
→ adapter.write()
→ capture result
→ release backend_mutex
```

如果 result 有故障：

```text
fallback diagnostic
→ runtime failure handler
```

Failure handler：

```text
config_mutex
→ backend_mutex
→ compare captured generation
→ stale?
     yes → ignore
     no  → deactivate/update backend
→ publish final runtime state
```

---

# 97. Stale Failure

例如：

```text
Thread A
generation 10
File write failed

Thread B
configure success
generation 11

Thread A
later handles failure
```

Thread A 获得：

```text
config_mutex
→ backend_mutex
```

后发现：

```text
captured_generation != current_generation
```

必须：

```text
discard stale failure result
```

不得修改 generation 11 backend。

---

# 98. Runtime Partial Failure

Console + File Active，File failure：

```text
deactivate File backend operation
```

成功后：

```text
BackendState = Ready
Active Console = true
Active File = false
Retry = false
backend_generation++
```

V1 不自动不断尝试恢复单个失效 File。

恢复由 explicit：

```cpp
configure(...)
```

完成。

---

# 99. Last Active Sink Failure

如果最后一个 Active sink 失败，并且 requested config 仍要求 normal sink：

```text
成功物理停用失败 sink
→ BackendState = RetryPending
→ Active sinks = none
→ Retry = true
→ deadline = now + 1 second
→ generation++
```

---

# 100. Global Runtime Failure

如果 adapter 返回：

```text
global_integrity_failure = true
```

不再尝试普通 sink degradation。

最终：

```text
BackendState = Faulted
Active sinks = none
Retry = false
```

---

# 101. FATAL Backend Operation

FATAL 使用同一个：

```cpp
std::mutex backend_mutex;
```

在一个临界区执行：

```text
capture generation
authoritative policy check
sanitize
FATAL write
flush successfully-written sinks
capture combined failure result
release backend_mutex
```

日志系统仍不 abort/terminate。

---

# 102. FATAL Partial Write Failure

例如：

```text
Console write success
File write failure
```

FATAL 必须继续：

```text
flush Console
```

而不是因为 File failure 就完全跳过 flush。

最终 failure set 合并：

```text
write_failed_sinks
+
flush_failed_sinks
```

Backend lock 释放后再调用 generation-aware failure handler。

---

# 103. FATAL Global Write Failure

如果 write 返回 backend integrity unknown：

```text
不继续调用复杂 backend flush
```

释放 backend lock 后：

```text
Faulted failure handler
fallback
```

---

# 104. FATAL Flush Failure

如果 FATAL write 成功，但部分 flush 失败：

```text
记录 flush failed sink
释放 backend lock
再进行 failure state transition
```

不能在 backend lock 内尝试获取 config mutex。

---

# 105. `flush()`

`flush()` 不 bootstrap。

先 acquire load runtime policy。

如果没有 Active sink：

```text
no-op
```

存在 Active sink时：

```text
backend_mutex
→ authoritative policy reload
→ capture generation
→ adapter.flush(active mask)
→ capture failure
→ release backend_mutex
```

然后再调用 failure handler。

---

# 106. `flush()` 不等于 Durable Storage

`flush()` 只要求 backend 将其正常 user-space/library buffers 推送到相应 OS/file stream 层。

V1 不调用、不承诺：

```text
fsync()
fdatasync()
storage barrier
physical-media durability
```

因此：

> `flush()` 不保证突然断电后日志一定持久存在。

FATAL forced flush 同样不等于 durable storage。

---

# 107. `noexcept` 总规则

以下 exported/public core functions 均必须保证异常不逃逸：

```text
configure()
set_level()
level()
is_enabled()
flush()
to_string()
impl::commit_message()
```

Backend Adapter 所有方法也必须 `noexcept`。

---

# 108. Adapter Exception Rule

Easylogging++、filesystem wrapper 或其他 backend 调用如果抛异常：

```text
adapter 内部 catch
→ 转换为明确 result
```

绝不能让第三方异常穿透 BackendAdapter API。

---

# 109. `configure()` Unexpected Exception

Preparation 阶段任何 unexpected exception：

```text
false
旧内部/backend 状态保持
```

Backend mutation 开始后的 unexpected failure：

```text
如果能够证明未改变 backend
    → false + old state

否则
    → false + Faulted
```

不得穿透 `noexcept`。

---

# 110. `set_level()` Exception

如果 mutex acquisition 或其他 internal operation 异常：

```text
return false
requested level unchanged
runtime policy unchanged
```

---

# 111. `commit_message()` Exception

`commit_message()` 最外层必须有 catch-all。

异常：

```text
不逃逸
不 terminate
fallback best effort
```

如果已经 publish Attempting，则 Attempt Finalizer 必须先完成 terminal transition。

---

# 112. Backend Lock Failure During Attempting

如果已经：

```text
Attempting published
```

但获取 backend mutex 时出现异常：

```text
backend 尚未 mutation
```

Attempt Finalizer 将状态转：

```text
RetryPending
```

并设置新的 retry deadline。

不得永久停留 Attempting。

---

# 113. Infrastructure Fallback

Fallback 不通过正常 logging backend。

建议直接使用：

```text
POSIX write(STDERR_FILENO, ...)
```

Fallback 不服从：

```text
Level::Off
normal sink disable policy
BackendState::Faulted
```

---

# 115. Fallback 输入安全

Fallback 不直接输出未经处理的任意：

```text
SourceLocation.file
SourceLocation.function
business message
backend text
```

使用固定容量 stack buffer，例如：

```text
512 bytes
```

SourceLocation 使用：

```text
bounded read
basename
ASCII control escaping
hard truncation
```

不得为了 fallback 再进行无界 heap allocation。

---

# 116. Fallback Rate Limiting

Internal failure class 建议固定为：

```text
Format
PointerContract
Preparation
Bootstrap
Configure
Write
Flush
SinkTransition
BackendFaulted
```

每类拥有独立 thread-safe atomic rate limiter。

默认：

```text
每类约最多 1 条 / 秒
```

---

# 117. Fallback Lock 规则

复杂 fallback 不得在持有：

```text
config_mutex
backend_mutex
```

时执行。

正常做法：

```text
capture small failure code
release logging lock
fallback
```

如果 fallback 本身失败：

```text
静默吞掉
```

不得递归、throw、abort。

---

# 118. `set_level()`

`set_level()`：

```text
config_mutex
→ requested_config.level = new level
→ load current Runtime Policy
→ 仅替换 Level bits
→ runtime_policy.store(release)
```

它绝不能根据 Requested Policy 重建 Active Sink bits。

例如：

```text
Requested File = true
Active File = false
```

调用：

```cpp
set_level(Level::Error);
```

后 Active File 必须仍为 false。

---

# 119. Submission Decision Point

Normal write 获取 backend mutex 后：

```cpp
runtime_policy.load(
    std::memory_order_acquire);
```

这一 authoritative load 定义为：

> runtime submission decision point。

如果 `set_level()` 在它之前 publication：

```text
新 Level 生效
```

如果在它之后：

```text
当前 in-flight record 可以完成
```

---

# 120. RHS 副作用

可能发生：

```text
first is_enabled() = true
RHS executed
later authoritative check = false
record dropped
```

因此日志 RHS 不得承担业务正确性依赖副作用。

---

# 121. BackendState 状态图

```text
                    ┌───────────────────┐
                    │   Uninitialized   │
                    └─────────┬─────────┘
                              │ auto owner
                              ▼
                    ┌───────────────────┐
          ┌────────►│    Attempting     │◄───────────┐
          │         └──┬────────────┬───┘            │
          │            │            │                │
 retry due│      success/degrade    │retryable fail │
          │            │            │                │
 ┌────────┴──────┐     ▼            ▼                │
 │ RetryPending  │  ┌───────┐   ┌──────────────┐    │
 └───────────────┘  │ Ready │   │ RetryPending │────┘
                    └───┬───┘   └──────────────┘
                        │
       partial sink loss│ → Ready
          last sink loss│ → RetryPending
 global/integrity failure
                        ▼
                   ┌─────────┐
                   │ Faulted │
                   └────┬────┘
                        │
             explicit configure
                        │
              reset + apply success
                        ▼
                      Ready
```

---

# 122. Explicit Configure 状态转换

Explicit configure 可从：

```text
Uninitialized
Ready
RetryPending
Faulted
```

进入。

成功：

```text
→ Ready
```

Preparation failure：

```text
→ 原状态自环
```

Backend `FailureOldStatePreserved`：

```text
→ 原状态自环
```

但 Faulted recovery 特例：

```text
reset success
candidate FailureOldStatePreserved
→ Faulted
```

Backend integrity unknown：

```text
→ Faulted
```

Explicit configure 在其他线程处于 Attempting 时会等待 `config_mutex`，随后基于 Attempt 最终状态继续执行，不得并发修改 backend。

---

# 123. Easylogging++ 初始化

Easylogging++ 所要求的：

```cpp
INITIALIZE_EASYLOGGINGPP
```

只在 production backend implementation TU 出现一次。

它仅负责 Easylogging++ 自身 storage，不代表 `mujoco_simulation` runtime bootstrap。

---

# 124. Easylogging++ 编译定义

必须一致：

```cmake
target_compile_definitions(
  mujoco_simulation
  PRIVATE
    ELPP_THREAD_SAFE
    ELPP_DISABLE_DEFAULT_CRASH_HANDLING
    ELPP_NO_DEFAULT_LOG_FILE
)
```

---

# 125. FATAL 不终止进程

Production Backend 初始化必须启用 Easylogging++ 的：

```cpp
el::LoggingFlag::
    DisableApplicationAbortOnFatalLog
```

因此：

```cpp
SIM_FATAL << "...";
```

不会主动：

```text
abort
terminate
exit
```

业务需要退出时自行：

```cpp
SIM_FATAL << "mjModel is null";
std::terminate();
```

---

# 126. 禁止默认 Easylogging 文件

必须保证不会生成：

```text
myeasylog.log
```

默认 File 完全由 mujoco_simulation 明确配置为：

```text
simulate.log
```

---

# 127. 单进程生命周期

V1 正式支持：

> `main()` 正常运行阶段中的日志。

V1 不支持依赖跨 Translation Unit static initialization/destruction 顺序的日志。

因此禁止依赖：

```text
global/static constructor 中 SIM_*
global/static destructor 中 SIM_*
```

---

# 128. Internal Singleton 生命周期

Production `Logger`、BackendAdapter 和 ClockAdapter 推荐通过 function-local holder 创建，并 intentionally leak 至 process termination：

```text
创建一次
正常运行阶段持续有效
不参与 static destruction ordering
```

目的不是支持 static destructor logging，而是减少内部 teardown 顺序风险。

---

# 129. Shared Library Unload

如果 `libmujoco_simulation.so` 被动态卸载：

```text
dlclose()
```

之后不得再调用：

```text
SIM_*
mujoco_simulation::logging::*
```

V1 不提供 unload-safe logger lifetime protocol。

---

# 130. Signal Handler

Logging API：

```text
不是 async-signal-safe
```

禁止在 POSIX signal handler 中调用：

```cpp
SIM_ERROR << "...";
flush();
configure(...);
```

Signal handler 需要诊断时使用适合 signal-safe 环境的底层机制。

---

# 131. `fork()` 限制

多线程 process 调用：

```text
fork()
```

后，在 child：

```text
exec()
```

之前不保证 logging 可用。

原因是 inherited mutex/backend 状态可能来自 fork 时的其他线程。

V1 不注册 `pthread_atfork()` 处理。

---

# 132. Rotation

V1 不提供：

```text
rotation
retention
compression
reopen
rotation signal
```

`simulate.log` 始终 append。

---

# 133. External Rotation

Rename-and-create：

```text
rename simulate.log simulate.log.1
create simulate.log
```

不会触发 process reopen。

进程可能继续写旧 inode。

因此：

> rename-and-create + live reopen 不属于 V1 支持协议。

---

# 134. `copytruncate`

部署侧可使用 copytruncate 类方法。

但 V1 不保证 rotation window 中：

```text
零日志丢失
零重复
事务一致性
```

`flush()` 也不提供 filesystem durability。

---

# 135. 多进程

多个 process 在同一 CWD 中默认都会打开：

```text
./simulate.log
```

`backend_mutex` 只同步当前 process。

V1 不保证跨 process：

```text
record atomicity
record ordering
flush ordering
rotation coordination
truncate coordination
```

多实例 deployment 应显式配置独立路径。

---

# 136. Public C++ ABI

以下属于正式 cross-DSO ABI：

```text
Level
SourceLocation
Policy

configure()
set_level()
level()
is_enabled()
flush()
to_string()
```

---

# 137. `Level` ABI

冻结：

```cpp
enum class Level : std::uint8_t
```

以及：

```text
Debug 10
Info  20
Warn  30
Error 40
Fatal 50
Off   255
```

---

# 138. `SourceLocation` ABI

冻结：

```cpp
struct SourceLocation
{
    const char* file;
    const char* function;
    std::int32_t line;
};
```

ABI-compatible release 不得重排、改类型或增删字段。

---

# 139. `Policy` ABI

`Policy` 包含：

```cpp
std::string
```

并通过：

```cpp
configure(const Policy&)
```

跨 DSO。

因此 consumer 与 library 必须使用兼容：

```text
architecture
compiler ABI
libstdc++ ABI
std::string ABI
calling convention
```

当前主要平台：

```text
Ubuntu 22.04
GCC 11
libstdc++
C++17
```

---

# 140. Macro-Support ABI

以下也必须保持 binary compatible：

```text
impl::commit_message()
```

虽然 namespace 为 `impl`，installed consumer 的 inline LogMessageStream 会引用这些 symbols。

---

# 141. C++17 Usage Requirement

Public header 使用 C++17 能力。

因此 CMake 必须：

```cmake
target_compile_features(
  mujoco_simulation
  PUBLIC
    cxx_std_17
)
```

不能只在 library 自身 PRIVATE 设置编译标准。

Installed consumer 自动继承 C++17 requirement。

---

# 142. Symbol Visibility

```cmake
set_target_properties(
  mujoco_simulation
  PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
)
```

导出：

```text
configure
set_level
level
is_enabled
flush
to_string
impl::commit_message
```

不导出：

```text
Logger
LoggingState
BackendAdapter
EasyloggingAdapter
ClockAdapter
BackendState
FakeBackend
```

---

# 143. Production CMake

```cmake
target_sources(
  mujoco_simulation
  PRIVATE
    src/log/logging.cpp
    src/log/impl/logger.cpp
    src/log/impl/easylogging_adapter.cpp
    third_party/easyloggingpp/easylogging++.cc
)

target_compile_features(
  mujoco_simulation
  PUBLIC
    cxx_std_17
)

target_include_directories(
  mujoco_simulation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>

  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/easyloggingpp
)

target_compile_definitions(
  simulate
  PRIVATE
    ELPP_THREAD_SAFE
    ELPP_DISABLE_DEFAULT_CRASH_HANDLING
    ELPP_NO_DEFAULT_LOG_FILE
)
```

---

# 144. 安装结果

```text
<prefix>/
├── include/
│   └── mujoco_simulation/
│       ├── export.hpp
│       └── log/
│           └── logging.hpp
│
└── lib/
    └── libmujoco_simulation.so
```

不得安装 Easylogging++ 或 internal logging core/backend headers。

---

# 145. 测试依赖注入

不向 public API 增加 test hook。

测试直接构造 internal：

```cpp
LoggingState state;

FakeBackendAdapter backend;
FakeClock clock;

Logger core{
    state,
    backend,
    clock};
```

Production singleton 则构造：

```text
LoggingState
EasyloggingAdapter
SteadyClockAdapter
Logger
```

这样 Fake Backend 和 Fake Clock 不需要替换 public singleton。

---

# 146. Internal State Observation

`LoggingState` 是 `src/log/impl/` 内部结构。

Internal test target 可以直接包含该内部 header 并在测试同步规则下观察状态。

不在 public header 或 public library API 中增加：

```text
get_backend_state_for_test()
set_fake_backend()
test_only_*
```

之类接口。

---

# 147. Fake Backend

Fake Backend 必须能够确定性注入：

```text
strict configure success
strict configure old-state-preserved failure
strict configure backend-faulted failure
default full success
default partial degradation
all-sink failure
reset success/failure
Console write failure
File write failure
global write failure
deactivate success/failure
Console flush failure
File flush failure
global flush failure
```

测试不得依赖真实 filesystem/第三方 backend 偶然出错来覆盖状态机。

---

# 148. Fake Clock

Fake Clock 支持直接：

```text
set time
advance time
```

Retry 测试不进行真实等待。

应验证：

```text
retry failure
→ deadline 更新

再次失败
→ deadline 再次前移

最终成功
→ retry=false
```

---

# 149. 必须覆盖的 Macro 测试

```cpp
#define ERROR 0
#define DEBUG 1
#define INFO  2

#include <mujoco_simulation/log/logging.hpp>
```

以下均必须正确：

```cpp
SIM_ERROR << "a";
SIM_LOG(ERROR) << "b";

SIM_DEBUG << "c";
SIM_LOG(DEBUG) << "d";

SIM_INFO << "e";
SIM_LOG(INFO) << "f";
```

还必须覆盖所有 public/internal reserved macro 的 include-before compile-fail collision test。

---

# 150. Compile-Time Gate 测试

五个 severity 均测试 compile-disabled RHS 不执行：

```text
DEBUG
INFO
WARN
ERROR
FATAL
```

并验证非法：

```text
-1
99
INVALID_TOKEN
```

compile fail。

---

# 151. Lazy/Stream 测试

需要覆盖：

```text
runtime-disabled RHS
custom operator<<
stream manipulators
user exception
operator<< exception
failbit
badbit
stream_.str() allocation failure
dangling-else macro usage
temporary object lifetime
empty message
```

---

# 152. Severity ABI 输入测试

需要覆盖：

```text
is_enabled(Level::Off) == false
is_enabled(invalid) == false

commit_message(Level::Off)
    no bootstrap
    no backend

commit_message(invalid)
    no bootstrap
    no backend
```

---

# 153. SourceLocation 测试

需要覆盖：

```text
normal caller file/function/line
basename
nullptr file
nullptr function
line <= 0
LF
CR
ESC
C0
超长 input
```

验证 fallback 和 normal record 都不会被 SourceLocation 注入控制字符。

---

# 154. File 测试

需要覆盖：

```text
default append
explicit configure append
default CWD
configure CWD
open 后 chdir
retry 前 chdir

file disabled + invalid path ignored
file enabled + empty path rejected
embedded NUL rejected
missing parent rejected
directory rejected
FIFO rejected
socket rejected
device rejected
regular file accepted
symlink→regular accepted
dangling symlink rejected

preflight 创建空文件
backend 后续失败时允许空文件保留
```

Automatic bootstrap 也必须运行同一组适用的 target-type validation 测试。

---

# 155. Policy 字段测试

必须验证：

```text
colored_console=false + TTY
    no ANSI

colored_console=true + TTY
    ANSI present

colored_console=true + redirected stderr
    no ANSI

File
    never ANSI

show_source_location=true
    Console/File 有 source

show_source_location=false
    Console/File 无 source
```

---

# 156. Output Record 测试

必须验证：

```text
Console target = stderr
uppercase severity
timestamp present
thread identifier present
exactly one physical newline
empty business message behavior
no message newline injection
File no ANSI
```

---

# 157. Bootstrap 状态机测试

覆盖：

```text
Uninitialized → Attempting → Ready
Uninitialized → Attempting → RetryPending
Uninitialized → Attempting → Faulted

RetryPending → Attempting → Ready
RetryPending → Attempting → RetryPending
RetryPending → Attempting → Faulted
```

还必须在：

```text
path resolution exception
allocation failure
backend mutex failure
adapter failure
```

后证明绝不会永久停留：

```text
Attempting
```

---

# 158. Attempting 惊群测试

大量线程同时在 retry due 时记录日志。

必须证明：

```text
只有一个 actual backend attempt
```

Attempting publication 后新进入线程的 RHS 不再执行。

在 publication 前已经通过 first admission 的线程允许成为 in-flight。

---

# 159. Configure 测试

覆盖：

```text
normal A → B
same path reconfigure
configure before first log
explicit no-sink
preparation failure
old-state-preserved failure
backend-faulted failure
Faulted reset failure
Faulted reset success + candidate success
Faulted reset success + candidate old-state-preserved failure
```

---

# 160. Runtime Sink Failure 测试

覆盖：

```text
Console+File
File fails
→ deactivate File
→ Console-only Ready

Console+File
Console fails
→ File-only Ready

last sink fails
→ RetryPending

global failure
→ Faulted

deactivate failed sink operation fails
→ Faulted
```

---

# 161. Generation 并发测试

Write/FATAL/flush 均必须在 backend lock 下取得 generation snapshot。

测试：

```text
Thread A detects failure generation N
Thread B configure generation N+1
Thread A failure handler
```

必须忽略 stale failure。

---

# 162. FATAL 测试

覆盖：

```text
normal FATAL write + flush + process survives

FATAL under Level::Off

FATAL partial write failure
successful sink still flushed

FATAL global write failure
no unsafe further backend operation

FATAL flush partial failure

failure handler only after backend lock released
```

---

# 163. `flush()` 测试

覆盖：

```text
no Active sink
    no-op

RetryPending
    no bootstrap

Faulted
    no backend call

normal active sink flush

per-sink flush failure
    sink degradation

last sink flush failure
    RetryPending

global flush failure
    Faulted

flush barrier
```

同时验证：

```text
flush != fsync durability
```

不会错误调用 durable-storage API。

---

# 164. `noexcept` 测试

必须通过 fault injection 覆盖：

```text
configure internal exception
set_level mutex/internal exception
flush adapter exception
commit backend exception
fallback internal failure
```

Exported noexcept 边界不得有异常逃逸或 `std::terminate()`。

Frontend `ostringstream`/用户 insertion exception 则按照 V1 public contract允许传播。

---

# 165. Fallback 测试

覆盖：

```text
Level::Off 下仍可工作
Faulted 下仍可工作
SourceLocation control chars
超长 SourceLocation
fixed-size truncation
rate limit
多线程 rate limit
fallback 内部 write failure
不递归
不在 logging lock 内调用复杂 fallback
```

---

# 166. 单进程 Record Atomicity 测试

多线程持续日志。

验证单个 sink 上：

```text
一条 record 的字节不会与另一条 record 交错
```

同时验证顺序只按 backend serialization 顺序，不宣称与线程开始时间一致。

---

# 167. 生命周期测试/限制

Public docs 必须明确：

```text
main 正常运行阶段
    supported

static/global constructor logging
    unsupported

static/global destructor logging
    unsupported

signal handler
    unsupported

fork child before exec
    unsupported

after shared-library unload
    unsupported
```

这些不应再作为模糊的实现行为。

---

# 168. Rotation 和多进程测试

Integration test 可验证：

```text
copytruncate 后 process 仍可继续写
```

但不测试零丢失保证。

两个 process 同 CWD：

```text
可以同时打开 simulate.log
library 不 crash
```

不要求跨进程 record atomicity。

---

# 169. Shared-Library Consumer 测试

独立 consumer：

```cmake
find_package(mujoco_simulation CONFIG REQUIRED)

target_link_libraries(
  consumer
  PRIVATE
    mujoco_simulation::mujoco_simulation
)
```

必须自动得到：

```text
C++17 usage requirement
mujoco_simulation include path
mujoco_simulation library
```

不得需要 Easylogging++。

---

# 170. ABI 测试

Release CI 应固定 baseline：

```text
Level underlying type/value

SourceLocation
    sizeof
    alignof
    field offset

Policy
    sizeof
    alignof
    field offset

exported public symbols

macro-support exported symbols
```

---

# 171. TSan

至少覆盖：

```text
ordinary writes
set_level
configure
automatic bootstrap
retry
Attempting publication
runtime sink failure
generation check
FATAL
flush
Faulted recovery
```

要求：

```text
no data race
no deadlock
no UAF
```

---

# 172. Public Header Self-Contained Test

```cpp
#include <mujoco_simulation/log/logging.hpp>

int main()
{
    SIM_INFO << "test";

    return 0;
}
```

必须在独立 consumer 中只依赖该 header 及 `mujoco_simulation::mujoco_simulation` 正常编译。

---

# 173. 最终普通日志路径

```text
SIM_INFO << expression
        │
        ▼
compile-time gate
        │
        ▼
is_enabled()
        │
        ├── Level disabled
        │      → RHS not evaluated
        │
        ├── Active sink
        │      → continue
        │
        ├── retry due
        │      → continue
        │
        └── Attempting / Faulted / backoff / explicit no-sink
               → RHS not evaluated
        │
        ▼
LogMessage
        │
        ▼
stream expression
        │
        ├── user/frontend exception
        │      → propagate
        │
        └── valid
               ▼
        destructor
               │
               ├── stack unwinding
               │      → drop
               ├── fail/bad
               │      → safe fallback
               └── valid
                      ▼
        commit_message()
               │
               ├── invalid severity/pointer
               │      → fallback/drop
               │
               ▼
        policy precheck
               │
               ├── retry due
               │      → try bootstrap ownership
               └── Active sink
                      ▼
        backend_mutex
               │
               ▼
        generation snapshot
               │
               ▼
        authoritative policy check
               │
               ├── disabled
               │      → drop
               └── enabled
                      ▼
        Source + Message sanitize
               │
               ▼
        BackendAdapter::write()
               │
               ▼
        release backend_mutex
               │
               ├── success
               │      → return
               └── failure
                      ▼
        safe fallback
               │
               ▼
        config_mutex
               │
               ▼
        backend_mutex
               │
               ▼
        generation check
               │
               ├── stale
               │      → ignore
               └── current
                      ▼
        physical sink deactivate /
        RetryPending / Faulted
               │
               ▼
        runtime policy publication
```

---

# 174. Final Configure 路径

```text
configure(candidate)
        │
        ▼
public input validation
        │
        ▼
deep-copy candidate
        │
        ▼
file path resolution + preflight
        │
        ▼
prepare all throwing resources
        │
        ├── failure
        │      → false
        │      → old internal/backend state unchanged
        │
        ▼
config_mutex
        │
        ▼
backend_mutex
        │
        ├── if Faulted
        │      reset_to_safe_baseline()
        │
        │      ├── failure
        │      │      → remain Faulted
        │      │      → false
        │      └── success
        │             → generation++
        │
        ▼
strict backend configure
        │
        ├── Success
        │      ▼
        │   no-throw Requested Policy publication
        │   Ready
        │   Active requested sinks
        │   retry=false
        │   generation++
        │   policy release store
        │   true
        │
        ├── FailureOldStatePreserved
        │      ▼
        │   candidate not published
        │   false
        │   Faulted recovery case remains Faulted baseline
        │
        └── FailureBackendFaulted
               ▼
            candidate not published
            Faulted
            no Active sinks
            retry=false
            policy release store
            false
```

---

# 175. 最终 Candidate 判定

本版设计已经在文档层面解决：

```text
backend_generation 锁归属冲突
runtime sink 仅改 policy、未改 backend
Attempting 永久卡死
FATAL/flush failure 锁序
noexcept exception mapping
shared_mutex writer starvation/record atomicity
状态图缺失
C++17 PUBLIC usage requirement
automatic bootstrap file validation
file type TOCTOU 语义
explicit append 语义
Console destination/颜色语义
输出 record 边界
process-local record atomicity
flush durability边界
frontend exception
invalid severity ABI 输入
fallback input safety
生命周期/fork/signal/unload
Faulted recovery 子状态
configure(false) 保守公共语义
retry deadline 数值与测试时钟
```

本文规定的以下验证已经完成：

```text
Fake Backend tests
Fake Clock tests
state-machine tests
failure-injection tests
shared-library consumer tests
ABI baseline
TSan
```

因此本文档状态为：

```text
V1 Frozen
```
