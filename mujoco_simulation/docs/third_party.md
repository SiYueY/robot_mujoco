# 第三方依赖

本文档记录 `mujoco_simulation` 随源码一同维护的第三方文件。它们位于
`mujoco_simulation/third_party/`，不属于本项目自行维护的实现。

第三方源码不得由项目的 `clang-format` 流程格式化；必须保留源文件内的版权和
许可证声明。更新时不得静默修改上游文件，任何本地补丁都应在本表中记录。

## tinyxml2

| 项目 | 内容 |
| --- | --- |
| 上游 | [leethomason/tinyxml2](https://github.com/leethomason/tinyxml2) |
| 版本 | `v11.0.0` |
| 许可证 | Zlib（许可证声明保留在源码文件头） |
| 本地目录 | `third_party/tinyxml2/` |
| 保留文件 | `tinyxml2.h`、`tinyxml2.cpp` |
| 本地修改 | 无 |

tinyxml2 用于解析 `robot_mujoco.xml`。未保留其示例、测试、文档和上游构建文件。

## Easylogging++

| 项目 | 内容 |
| --- | --- |
| 上游 | [abumq/easyloggingpp](https://github.com/abumq/easyloggingpp) |
| 版本 | `v9.97.1` |
| 许可证 | MIT（许可证声明保留在源码文件头） |
| 本地目录 | `third_party/easyloggingpp/` |
| 保留文件 | `easylogging++.h`、`easylogging++.cc` |
| 本地修改 | 无 |

Easylogging++ 提供本模块使用的日志实现。仅保留构建所需的实现文件。

## 更新方式

更新任一依赖时，从表中记录的上游版本取得对应保留文件，保留其版权和许可证声明，
更新本页版本信息，并执行本模块可用的构建与测试检查。若必须存在本地修改，应记录
具体补丁及其原因。
