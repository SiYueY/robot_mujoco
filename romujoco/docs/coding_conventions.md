# 编码规范

本文档记录本仓库源码的编码约定。格式化由仓库根目录 `.clang-format`
（`BasedOnStyle: Google`，`ColumnLimit: 100`）统一负责；本文档另外定义
include 顺序，并说明如何对整个仓库执行格式化。

## 格式化范围

clang-format 覆盖仓库内所有自行维护的 C++ 源码，即 `romujoco/` 与
`ros2_mujoco/` 下的 `.hpp` / `.cpp` / `.h` / `.cc`，包括
`romujoco/src/viewer/simulate/` 与 `romujoco/src/viewer/lodepng/`。

不参与格式化：

- `romujoco/third_party/`：第三方源码，见
  [third_party.md](./third_party.md)，该目录下的 `DisableFormat: true`
  已禁止格式化；
- `build/` 等构建产物目录。

注意 `romujoco/third_party/mujoco/src/user/` 等子目录保留了上游自带的
`.clang-format`，其中部分选项高于本机 clang-format 版本，直接对整个仓库执行
clang-format 会在这些文件上报配置解析错误。因此统一按下面的命令排除
`third_party`。

```bash
# 检查整个仓库（不修改文件）
find . -path ./.git -prune -o -path '*/build' -prune \
  -o -path './romujoco/third_party' -prune \
  -o \( -name '*.hpp' -o -name '*.cpp' -o -name '*.h' -o -name '*.cc' \) -print0 |
  xargs -0 -r clang-format --dry-run -Werror

# 格式化整个仓库
find . -path ./.git -prune -o -path '*/build' -prune \
  -o -path './romujoco/third_party' -prune \
  -o \( -name '*.hpp' -o -name '*.cpp' -o -name '*.h' -o -name '*.cc' \) -print0 |
  xargs -0 -r clang-format -i
```

## include 顺序

适用于 `romujoco/src` 下所有非 vendored 的 `.hpp` / `.cpp` 文件。
`src/viewer/simulate/` 与 `src/viewer/lodepng/` 为上游 vendored 代码，不参与排序。

每个文件顶部的 include 块按以下分组排列，**组间空一行**，组内按 include 串
（不含 `#include` 前缀，不区分大小写）做字典序：

1. 自身头文件：仅 `.cpp` 中声明本文件实现的 `.hpp`；若它是公共头
   （如 `simulation.cpp` 的 `romujoco/simulation.hpp`），只出现在此处，
   不重复进入公共组。`.hpp` 文件没有这一组。
2. 标准库头：`<...>`。
3. 第三方 / 系统头：`<mujoco/*>`、GLFW、EGL、`easylogging++.h`、`tinyxml2.h`，
   以及 vendored 的 `glfw_adapter.h` / `simulate.h` / `lodepng.h`。
4. 项目公共头：`"romujoco/..."`。
5. 项目私有头：`"buffer/..."`、`"common/..."`、`"component/..."`、
   `"config/..."`、`"render/..."`、`"runtime/..."`、`"simulation/..."`、`"viewer/..."`。

示例（改造前 / 改造后）：

```cpp
// 改造前：私有头与公共头混排，第三方头先于标准库头
#include <mujoco/mujoco.h>

#include <memory>

#include "component/component.hpp"
#include "romujoco/component/camera.hpp"

// 改造后
#include <memory>

#include <mujoco/mujoco.h>

#include "romujoco/component/camera.hpp"

#include "component/component.hpp"
```

注意：仓库根目录 `.clang-format` 保持 `IncludeBlocks: Preserve` / `SortIncludes: false`，
clang-format 不会重排 include；本约定靠人工维护。
