# MuJoCo `simulate` provenance

- Upstream repository: <https://github.com/google-deepmind/mujoco>
- Upstream version: `3.12.0`
- Upstream commit: `13827e9ee56f097f57acf69ae52b078f9839682d`
- Import date: 2026-09-16
- Imported files: the local `simulate/` viewer implementation and its GLFW
  platform adapters used by `romujoco_viewer`.
- Local modifications: the files are retained as a RoMuJoCo-owned fork and
  are integrated with `simulation_viewer`; no viewer behavior was changed by
  the project-identity refactor.

## Upgrade notes

The bundled MuJoCo baseline and this fork's upstream baseline must stay in
lockstep. Before upgrading MuJoCo, compare the corresponding upstream
`simulate/` sources, reapply intentional local changes, and record the new
version, commit, import date, and differences above.
