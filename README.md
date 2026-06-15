# robot_mujoco

Generic MuJoCo + ROS 2 simulation workspace.

The repository name and Python package name remain `robot_mujoco` for now.
The runtime is model-agnostic, and the default example robot model is Franka / Panda.

## Setup

### 1. Initialize submodules

This repository vendors `robosuite` and `robocasa` under `third_party` as git submodules.

```bash
git submodule update --init --recursive
```

Verify:

```bash
git submodule status
```

### 2. Create a Python virtual environment with `uv`

To avoid polluting the system Python environment, create and use a local virtual environment.
Use Python `3.10.12` so the environment matches the Python version used by ROS 2:

```bash
uv venv --python 3.10.12 .venv
source .venv/bin/activate
```

If `uv` is not installed yet:

```bash
python3 -m pip install --user uv
```

If Python `3.10.12` is not available locally yet:

```bash
uv python install 3.10.12
```

### 3. Install the local `robot_mujoco` Python package

Install the workspace Python package first so `robot_mujoco.robocasa` is importable:

```bash
uv pip install -e .
```

### 4. Install `robosuite` and `robocasa`

Install the vendored submodules in editable mode:

```bash
uv pip install --upgrade pip setuptools wheel
uv pip install -e "robosuite@third_party/robosuite"
uv pip install -e "robocasa@third_party/robocasa"
```

### 5. Initialize robosuite and RoboCasa private macros

Both `robosuite` and `robocasa` expect private macro files for machine-local settings.
Create them once after installation:

```bash
uv run third_party/robosuite/robosuite/scripts/setup_macros.py
uv run third_party/robocasa/robocasa/scripts/setup_macros.py
```

This creates:

- `third_party/robosuite/robosuite/macros_private.py`
- `third_party/robocasa/robocasa/macros_private.py`

You can later edit these files for machine-local settings such as dataset paths or SpaceMouse IDs.

### 6. Download RoboCasa kitchen assets

RoboCasa environments are not complete until the kitchen assets are downloaded.
The full asset package is about 10 GB.

Download all assets:

```bash
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type all
```

If you only want specific subsets, the available asset groups are:

- `tex`
- `tex_generative`
- `fixtures_lw`
- `objs_objaverse`
- `objs_aigen`
- `objs_lw`

Example:

```bash
uv run third_party/robocasa/robocasa/scripts/download_kitchen_assets.py --type tex objs_objaverse
```

### 7. Validate Python imports

```bash
uv run python - <<'PY'
import robot_mujoco
import robot_mujoco.robocasa
import robosuite
import robocasa

print("robot_mujoco:", robot_mujoco.__file__)
print("robot_mujoco.robocasa:", robot_mujoco.robocasa.__file__)
print("robosuite:", robosuite.__file__)
print("robocasa:", robocasa.__file__)
PY
```

### 8. Validate RoboCasa environment creation

Run a minimal environment creation check:

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

Ignore non-fatal warnings during the first setup unless the import or environment creation step actually fails.

## Build split

`colcon build` now installs the ROS 2 workspace packages:

- `robot_mujoco`
- `mujoco_simulation`
- `mujoco_hardware`

Runtime rendering currently remains viewer-only:

- the unified simulation path always starts the MuJoCo viewer
- this repository does not provide a headless runtime path today
- camera registration state is now managed internally through a single `HardwareManager` path

RoboCasa scene generation is now provided by the top-level Python package:

- import path: `robot_mujoco.robocasa`
- installation command: `uv pip install -e .`
- unified scene command: `robot-mujoco-robocasa-scene`

## RoboCasa Scene

After building and sourcing the workspace, use the unified `robot_mujoco` entry point to interactively select one RoboCasa kitchen scene, compose it with the default FR3 model, and launch the viewer-backed simulation:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_mujoco mujoco_simulation mujoco_hardware
source install/setup.bash
uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/robocasa_fr3_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml
```

When `--task`, `--layout`, or `--style` are omitted, the command enters an interactive terminal menu. It writes a composed MJCF file, then launches `ros2_control_node`, `robot_state_publisher`, and the default FR3 controllers through `robot_mujoco/launch/sim.launch.py`.

Use `--no-launch` when you only want to generate the MJCF:

```bash
uv run robot-mujoco-robocasa-scene \
  --config robot_mujoco/config/robocasa_fr3_scene.yaml \
  --output /tmp/robot_mujoco_robocasa_scene.xml \
  --no-launch
```

## robot_mujoco FR3 example

This repository still includes the standalone resource/example package at [example/franka_fr3_mujoco](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/example/franka_fr3_mujoco).

It vendors the MuJoCo Menagerie FR3 MJCF under `example/franka_fr3_mujoco/model` and remains the source of the default FR3 model used by the unified RoboCasa scene flow.
