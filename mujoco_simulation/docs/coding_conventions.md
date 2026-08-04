# 编码规范

本文档记录 `mujoco_simulation` 源码的本地编码约定。当前只定义 include 顺序；其他格式化由
仓库根目录 `.clang-format`（`BasedOnStyle: Google`）负责。

## include 顺序

适用于 `mujoco_simulation/src` 下所有非 vendored 的 `.hpp` / `.cpp` 文件。
`src/viewer/simulate/` 与 `src/viewer/lodepng/` 为上游 vendored 代码，不参与排序。

每个文件顶部的 include 块按以下分组排列，**组间空一行**，组内按 include 串
（不含 `#include` 前缀，不区分大小写）做字典序：

1. 自身头文件：仅 `.cpp` 中声明本文件实现的 `.hpp`；若它是公共头
   （如 `simulation.cpp` 的 `mujoco_simulation/simulation.hpp`），只出现在此处，
   不重复进入公共组。`.hpp` 文件没有这一组。
2. 标准库头：`<...>`。
3. 第三方 / 系统头：`<mujoco/*>`、GLFW、EGL、`easylogging++.h`、`tinyxml2.h`，
   以及 vendored 的 `glfw_adapter.h` / `simulate.h` / `lodepng.h`。
4. 项目公共头：`"mujoco_simulation/..."`。
5. 项目私有头：`"buffer/..."`、`"common/..."`、`"component/..."`、
   `"config/..."`、`"render/..."`、`"runtime/..."`、`"simulation/..."`、`"viewer/..."`。

示例（改造前 / 改造后）：

```cpp
// 改造前：私有头与公共头混排，第三方头先于标准库头
#include <mujoco/mujoco.h>

#include <memory>

#include "component/component.hpp"
#include "mujoco_simulation/component/camera.hpp"

// 改造后
#include <memory>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/component/camera.hpp"

#include "component/component.hpp"
```

注意：仓库根目录 `.clang-format` 保持 `IncludeBlocks: Preserve` / `SortIncludes: false`，
clang-format 不会重排 include；本约定靠人工维护。
