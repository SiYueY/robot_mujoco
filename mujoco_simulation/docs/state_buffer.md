# StateBuffer 并发与实时性能设计

本文档将 `StateBuffer` 抽象为通用的 **single-writer / multiple-reader latest immutable snapshot publication** 问题：仿真线程发布最新状态，多个调用方独立读取，而慢读者不应阻塞仿真推进。

本文描述设计取舍，不代表所有提议均已实现。

## 1. 语义边界

`StateBuffer` 是 latest-value 发布器，不是状态事件队列：

- writer 发布新快照会替换“当前最新”快照；
- reader 可以跳过中间状态，但不能读取撕裂或被并发修改的状态；
- reader 持有的快照必须在其使用期间有效；
- writer 不因慢 reader 而访问已发布对象的可变存储。

当前 `shared_ptr<const RobotState>` 原子发布模型满足以上安全语义，并提供零拷贝的只读快照获取。

## 2. 当前通用瓶颈

原子 `shared_ptr` 解决生命周期问题，但不自动保证确定性性能。完整 `RobotState` 的构建仍可能涉及：

- `unordered_map` 创建、复制或扩容；
- lidar ranges、camera 图像等 `vector` 的扩容；
- 状态字符串复制；
- shared pointer 控制块分配与多 reader 引用计数竞争。

因此应区分“读者无 mutex 等待”和“整个状态发布路径零分配”两个目标。

## 3. 动态模式与有界实时模式

### 动态模式

动态模式允许组件集合、状态字段和变长载荷增长。结构变化必须构造新版本快照，不能原地修改已经发布且可能被 reader 持有的对象。该模式允许分配，优先易用性和拓扑弹性。

### 有界实时模式

有界模式声明快照槽位数量、每类状态最大数量，以及 lidar/image 等载荷的最大尺寸。运行期超过上限时不隐式扩容。

有界模式不是“初始化后配置永久固定”的要求；它允许在非实时变更窗口重新协商上界、建立新的快照池，然后恢复实时发布。

## 4. 可选实现

| 实现 | 优点 | 局限 |
| --- | --- | --- |
| 原子 `shared_ptr<const T>` | 生命周期简单，API 易用，多 reader 安全 | 引用计数和对象分配可能造成延迟波动 |
| 固定快照池 + reader lease | writer/reader 热路径可无分配，槽位数有界 | 需要定义槽位耗尽、lease 生命周期和 ABA 防护 |
| seqlock | 标量 POD 复制快，无 writer mutex | 不适用于 map/vector 等变长或非平凡对象 |

对于包含 map、字符串和变长 sensor 数据的 `RobotState`，seqlock 不能单独保证 C++ 数据竞争安全。推荐实时目标是固定快照池加只读 `StateLease`：reader 获取 slot 后持有 lease，writer 只写入空闲槽位，再发布 slot index 与 generation。

```cpp
// 提议接口，尚未实现。
bool try_read_state(StateLease& out) const;
```

现有 shared pointer 与复制式读取接口可继续作为动态模式兼容入口。

## 5. 槽位耗尽与动态变更

有界模式下，若全部快照槽位仍被 reader 持有，writer 默认跳过本次发布并增加 `dropped_publication`，绝不等待 reader 或临时分配。这样仿真线程的执行时间保持有界。

动态模式可选择扩展池，但扩展必须在新快照版本上完成。旧快照及其变长存储须在所有 reader 释放后回收。

图像、depth 和 lidar scan 都属于可变长状态载荷。当前 `RobotState` 通过不可变共享组件快照复用未变化载荷；有界实时模式仍必须预分配上界，并将超过上界视为失败或丢弃。

## 6. 基准与验收

建议采集：

- publish、lease acquire/release 的 p50/p99/max 延迟；
- writer 与 reader 热路径的堆分配次数；
- dropped-publication 次数与最长 reader 持有时间；
- 读者数量变化时的引用计数或 lease 竞争；
- 组件/载荷动态变化建立新版本快照的耗时。

实时模式验收标准是 writer 不等待 reader、不隐式分配，reader 只能看到完整版本快照，槽位不足时行为可观测且可预测。
