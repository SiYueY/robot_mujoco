# RoboCasa 集成指南

`robot_mujoco` 通过 `third_party/` vendored submodule 集成 RoboCasa 作为 kitchen 场景生成器，并通过统一 CLI 完成“场景生成 + viewer 仿真展示”。本文档说明如何拉取依赖、下载资产并运行这条完整链路。

## 1. 拉取 Third-Party Submodule

RoboCasa 及其依赖 `robosuite` 以 git submodule 形式 vendored 在 `third_party/` 下：

```bash
cd robot_mujoco

# 注册 submodule 并拉取代码
git submodule add https://github.com/ARISE-Initiative/robosuite.git third_party/robosuite
git submodule add https://github.com/robocasa/robocasa.git   third_party/robocasa
git submodule update --init --recursive
```

> 如果 `git submodule update --init --recursive` 输出为空且 `third_party/` 未出现，说明 `.gitmodules` 存在但从未注册到 `.git/config`。使用上方 `git submodule add` 命令重新注册。

拉取后目录结构：

```
third_party/
├── COLCON_IGNORE       # 防止 colcon 扫描该目录
├── robosuite/           # ARISE-Initiative/robosuite (master)
└── robocasa/            # robocasa/robocasa (main)
```

> `COLCON_IGNORE` 空文件确保 `colcon build` 跳过 `third_party/`，避免其中可能存在的 `package.xml` 被误识别为 ROS 2 包。

## 2. Python 虚拟环境

项目使用 Python 3.10（匹配 ROS 2 Humble 的 Python 版本），通过 `uv` 管理依赖：

```bash
# 创建虚拟环境
uv venv --python 3.10.12 .venv
source .venv/bin/activate

# 若 Python 3.10 未安装
uv python install 3.10.12
```

## 3. 安装 Robosuite 与 RoboCasa

以 editable 模式安装 vendored 的 submodule：

```bash
uv pip install --upgrade pip setuptools wheel
uv pip install -e "robosuite@third_party/robosuite"
uv pip install -e "robocasa@third_party/robocasa"
```

`pip install -e` 会从本地 `third_party/` 目录安装，修改 submodule 源码后无需重新安装。

## 4. 初始化系统宏

Robosuite 和 RoboCasa 期望存在各自的 `macros_private.py` 配置文件。使用内置脚本生成默认版本：

```bash
uv run third_party/robosuite/robosuite/scripts/setup_macros.py
uv run third_party/robocasa/robocasa/scripts/setup_macros.py
```

生成的文件：

| 文件 | 用途 |
|------|------|
| `third_party/robosuite/robosuite/macros_private.py` | robosuite 本地配置（数据集路径、SpaceMouse ID） |
| `third_party/robocasa/robocasa/macros_private.py` | robocasa 本地配置 |

## 5. 下载 Kitchen 资产

RoboCasa 的厨房场景依赖约 **10 GB** 的纹理、模型和 fixture 资产。使用内置脚本下载：

```bash
# 下载全部资产（约 10 GB）
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type all
```

资产分组（可按需选择）：

| 组名 | 内容 | 说明 |
|------|------|------|
| `tex` | 环境纹理 | 墙壁、地板材质 |
| `tex_generative` | 生成式纹理 | AI 生成的纹理变体 |
| `fixtures_lw` | 轻量 fixture | 橱柜、台面等固定装置 |
| `objs_objaverse` | Objaverse 物体 | 3D 模型库中的物体 |
| `objs_aigen` | AI 生成物体 | 程序化生成的物体 |
| `objs_lw` | 轻量物体 | 小物件、厨房用具 |

按需下载示例：

```bash
# 仅下载纹理和 fixture
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type tex fixtures_lw
```

资产安装到 `third_party/robocasa/robocasa/models/assets/` 目录。

## 6. 验证安装

### 6.1 Python 导入

```bash
uv run python - <<'PY'
import robosuite
import robocasa

print("robosuite:", robosuite.__file__)
print("robocasa:",  robocasa.__file__)
PY
```

### 6.2 环境创建

```bash
uv run python - <<'PY'
import gymnasium as gym
import robocasa

env = gym.make(
    "robocasa/PickPlaceCounterToCabinet",
    split="pretrain",
    seed=0,
)
print("Environment created:", type(env))
env.close()
PY
```

首次运行时可能产生非致命的警告，只要 import 和环境创建不实际报错即可。

## 7. 故障排除

| 问题 | 原因 | 解决 |
|------|------|------|
| `git submodule update` 无输出 | `.gitmodules` 从未注册到 `.git/config` | 使用 `git submodule add` 重新注册 |
| `pip install` 报 `package directory not found` | submodule 目录为空 | 先执行 `git submodule update --init` |
| 下载资产超时 | Box 服务器不可达 | 重试；或按组分别下载 |
| `gladLoadGL error` (MuJoCo 3.x) | 系统 GL 驱动不兼容 | 当前项目只支持 viewer 运行；需要先修复本机 OpenGL / 显示环境 |
| `ImportError: robocasa` | Python 环境未安装 | 确保已执行 `pip install -e` |
| 资产下载空间不足 | /tmp 空间不够 | 资产直接下载到 robocasa 包目录，确保有 12 GB+ 可用空间 |

## 8. 统一生成与展示入口

当前推荐入口不是直接调用底层 `scene_cli.py`，而是统一使用 `robot_mujoco` 提供的正式场景 CLI：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/franka_fr3/robocasa_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml
```

这条命令会完成四件事：

1. 读取 RoboCasa YAML 配置
2. 若未显式传 `task/layout/style`，进入交互式菜单选择场景
3. 生成厨房场景并删除占位机器人 `robot0_base`
4. 插入默认 FR3 MJCF，并写出组合后的 MuJoCo XML
5. 通过 `robot_mujoco/launch/robot_mujoco.launch.py` 启动 viewer 仿真

交互规则参考 RoboCasa 官方脚本：

- `task` 默认使用配置文件中的 `task_name`
- `layout` 和 `style` 默认使用“随机”选项
- 菜单接受 1-based 编号；输入 `0` 或直接回车时走默认值

如果只想生成 XML，不立刻启动仿真：

```bash
uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/franka_fr3/robocasa_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml \
  --no-launch
```

可选覆盖场景参数：

```bash
uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/franka_fr3/robocasa_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml \
  --task OpenDrawer \
  --layout 3 \
  --style 2
```

## 9. 配置与模型组合规则

统一 CLI 仍然复用 `robot_mujoco.robocasa.SceneGenerator` 和 MJCF 适配器，组合规则固定为：

- RoboCasa 先生成原始厨房 MJCF
- 删除占位机器人 body：`placeholder_robot_body_name`
- 插入项目机器人 MJCF：`robot.xml_path`
- 若 RoboCasa 提供推荐初始位姿，则写入 `robot.root_body_name`

默认示例配置文件为 [robot_mujoco/config/franka_fr3/robocasa_scene.yaml](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/robot_mujoco/config/franka_fr3/robocasa_scene.yaml)。

其中：

- `robot.xml_path: __DEFAULT_FR3_XML__` 表示交由统一 CLI 自动解析到仓库内默认 FR3 MJCF
- `robot.root_body_name: base` 对应当前 FR3 模型的根 body 名

## 10. 底层 Python API

如果你只想在 Python 中做离线生成，而不直接启动 viewer，仍可继续使用底层 API：

RoboCasa 通过 `robot_mujoco` 包的 `robocasa` 模块暴露：

```python
from robot_mujoco import SceneConfig, load_config
from robot_mujoco.robocasa import SceneGenerator

config = load_config("scene.yaml")
scene = SceneGenerator().generate(config)
```

配置示例参考：

- [robot_mujoco/config/robocasa_scene.example.yaml](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/robot_mujoco/config/robocasa_scene.example.yaml)
- [robot_mujoco/config/franka_fr3/robocasa_scene.yaml](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/robot_mujoco/config/franka_fr3/robocasa_scene.yaml)
