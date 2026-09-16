# RoMuJoCo

RoMuJoCo is a general-purpose robotics simulation runtime built on MuJoCo. It
provides a reusable C++ interface for simulation lifecycle, robot components,
rendering, and viewer integration.

The public API is in `include/romujoco/`; its main entry point is
`romujoco::Simulation`. XML configuration remains compatible with the existing
`robot_mujoco` schema.

## Build

MuJoCo 3.12.0 is pinned as the `third_party/mujoco` Git submodule. Initialize
it before preparing its private staging prefix at `mujoco/`; this directory is
outside the submodule and is not a prefix users need to add to their environment.

```bash
git submodule update --init --recursive
./scripts/mujoco.sh build

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

`scripts/mujoco.sh` is independent of the RoMuJoCo build tree and supports
`build`, `configure`, `install`, `version`, `status`, `clean`, `purge`, and
`rebuild`. If a prior build created `third_party/mujoco/install`, run
`./scripts/mujoco.sh rebuild` to remove that legacy staging directory. Use
`./scripts/mujoco.sh --help` for its build-type and job options.

## Install and consume

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/romujoco"
cmake --build build
cmake --install build
```

An installed consumer only needs RoMuJoCo:

```cmake
find_package(romujoco CONFIG REQUIRED)
target_link_libraries(app PRIVATE romujoco::romujoco)
```

The install merges the bundled MuJoCo SDK into the same prefix and uses a
same-directory runtime path for `libromujoco.so` and `libmujoco.so`.
