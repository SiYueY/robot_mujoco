#pragma once

// https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MUJOCO_SIMULATION_EXPORT __attribute__((dllexport))
#define MUJOCO_SIMULATION_IMPORT __attribute__((dllimport))
#else
#define MUJOCO_SIMULATION_EXPORT __declspec(dllexport)
#define MUJOCO_SIMULATION_IMPORT __declspec(dllimport)
#endif
#ifdef MUJOCO_SIMULATION_BUILDING_LIBRARY
#define MUJOCO_SIMULATION_PUBLIC MUJOCO_SIMULATION_EXPORT
#else
#define MUJOCO_SIMULATION_PUBLIC MUJOCO_SIMULATION_IMPORT
#endif
#define MUJOCO_SIMULATION_PUBLIC_TYPE MUJOCO_SIMULATION_PUBLIC
#define MUJOCO_SIMULATION_LOCAL
#else
#define MUJOCO_SIMULATION_EXPORT __attribute__((visibility("default")))
#define MUJOCO_SIMULATION_IMPORT
#if __GNUC__ >= 4
#define MUJOCO_SIMULATION_PUBLIC __attribute__((visibility("default")))
#define MUJOCO_SIMULATION_LOCAL __attribute__((visibility("hidden")))
#else
#define MUJOCO_SIMULATION_PUBLIC
#define MUJOCO_SIMULATION_LOCAL
#endif
#define MUJOCO_SIMULATION_PUBLIC_TYPE
#endif
