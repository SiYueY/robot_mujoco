# CommandBuffer 并发与实时性能设计

本文档将 `CommandBuffer` 抽象为通用的 **latest-command exchange** 问题：多个调用方提交控制目标，单个 scheduler 在物理步进前取得可应用的最新命令集合。

本文描述设计取舍，不代表所有提议均已实现。

## 1. 语义边界

`CommandBuffer` 不是历史命令 FIFO。它的核心语义是 latest-wins：

- 单组件写入覆盖该组件的最近命令。
- 批量写入仅覆盖对应强类型命令批次中有值的槽位。
- `RobotCommand` 先整体校验，再作为一个 snapshot 发布其 Joint 与 MobileBase 更新；
  scheduler 只会观察到旧快照或完整新快照。
- `clear()` 清空命令集合。
- scheduler 在一个物理周期内只应用每个组件的最终有效命令。

公开层保留以下强类型写入口：

```cpp
write_command(JointId, JointCommand)
write_command(MobileBaseId, MobileBaseCommand)
write_command(RobotCommand)
write_commands(JointCommands)
write_commands(MobileBaseCommands)
```

`RobotCommand` 的空 batch 与 batch 中空槽均保留已有命令；两个 batch 都为空时为
成功但不发布的无操作。单类型 batch 入口是优化机会，不是对调用方的强制要求，且
不提供跨类型原子提交保证。

## 2. 并发模型

| 模型 | producer | consumer | 适用情况 |
| --- | --- | --- | --- |
| SPSC | 单个控制线程 | scheduler | 单一控制回路，可使用最轻量交换机制 |
| MPSC | 多个调用线程 | scheduler | 公开 API 的兼容模型，需要定义写入顺序 |
| 批量替换 | 一个或多个调用线程 | scheduler | 控制器按周期生成完整命令集合 |

多 writer 对同一组件的写入顺序由成功进入 buffer 的顺序定义。实现必须避免“半个批量命令”被 scheduler 观察到。

## 3. 当前通用瓶颈

当前实现使用 mutex 保护通道注册与发布。通道写入以 copy-on-write 生成
不可变槽位快照；scheduler 只在 sequence 改变时取得新的类型擦除快照。
因此未更新周期不会复制完整命令数组。后续若需要有界实时路径，应以基准数据
决定是否引入 frame queue，而不与当前 latest-wins 协议混合。

- map 节点和 bucket 分配、复制与析构；
- 字符串哈希和名称查找；
- writer 与 scheduler 在同一临界区竞争；
- 命令集合变大时，scheduler 延迟随集合大小和 allocator 状态波动。

这些成本与 MuJoCo 物理计算无关，但会影响控制周期的 p99/max 延迟。

## 4. 动态模式与有界实时模式

### 动态模式

动态模式允许命令目标新增、删除或扩容。变更路径可以使用锁、哈希容器和分配，优先保证 API 易用性与拓扑弹性，不承诺确定性延迟。

### 有界实时模式

有界模式由调用方声明最大组件数、最大批量大小和队列容量。运行期不隐式扩容：超过上限、未知名称或队列满均返回失败并记录诊断计数。

有界并不要求系统配置永远固定；它要求每个实时运行窗口内的数据形状不超过已声明上界。需要扩展上界时可切回动态变更路径，再重新建立有界运行窗口。

## 5. 可选实现

| 实现 | 优点 | 局限 |
| --- | --- | --- |
| mutex + 容器复用 | 改动小，保留当前模型 | scheduler 仍与 writer 竞争；map 复制成本仍在 |
| pending delta + active set | scheduler 不复制完整命令集 | 需要处理批量替换与单组件更新的顺序 |
| 固定 slot + 有界 MPSC command-frame queue | 热路径无分配，时间上界清晰 | 需要容量、队列满和动态变更策略 |

推荐的实时目标是第三种：配置/变更路径将名称解析为 slot，运行路径传递预分配 command frame 与位图。scheduler 持有私有 active command set，drain 所有已发布 frame 后只应用最终 slot 值。

可额外提供提议接口，供调用方缓存解析结果：

```cpp
// 提议接口，尚未实现。
bool command_handle(const std::string& name, CommandHandle& out) const;
bool write_command(CommandHandle handle, const JointCommand& command);
```

名称接口必须继续存在；handle 仅用于避免严格实时调用方重复名称解析。

## 6. 队列与失败策略

有界 MPSC queue 必须使用 `try_enqueue`：

- 满时立即返回 `false`，不阻塞、不自旋等待、不扩容；
- scheduler 每周期按固定上限 drain；
- 批量替换、单组件增量和 `clear()` 都以 frame 形式进入同一顺序；
- 动态注册/注销必须与 frame 版本关联，旧 slot 或未知名称不能误写入新组件。

这将背压责任交给上层控制器：上层可重试、合并命令或报告控制回路过载。

## 7. 基准与验收

建议采集：

- 每周期和每次 enqueue 的堆分配次数；
- enqueue、scheduler drain 的 p50/p99/max 延迟；
- queue-full、unknown-target、dropped-frame 次数；
- 从成功提交到被 scheduler 应用的周期数；
- 动态注册、注销和容量变化的耗时。

实时模式验收标准是热路径零分配、无等待，以及延迟上界仅由已声明容量和每帧最大 slot 数决定。
