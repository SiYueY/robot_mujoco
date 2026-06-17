#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mujoco_hardware/mujoco_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_hardware {
namespace {

double mean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double p95(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index =
      std::min(values.size() - 1, static_cast<std::size_t>(std::ceil(values.size() * 0.95) - 1));
  return values[index];
}

struct TempModelFile {
  explicit TempModelFile(std::string xml_contents) {
    path = std::filesystem::temp_directory_path() / "mujoco_hardware_perf_loop.xml";
    std::ofstream output(path);
    output << xml_contents;
  }

  ~TempModelFile() {
    if (!path.empty()) {
      std::error_code error;
      std::filesystem::remove(path, error);
    }
  }

  std::filesystem::path path;
};

hardware_interface::HardwareInfo parse_sensor_hardware_info(const std::string& model_path) {
  const std::string urdf = R"(
<robot name="perf_robot">
  <ros2_control name="mujoco_system" type="system">
    <hardware>
      <plugin>mujoco_hardware/MuJoCoHardwareInterface</plugin>
      <param name="mujoco_model_path">)" +
                           model_path + R"(</param>
      <param name="render_mode">headless</param>
      <param name="sim_speed_factor">1.0</param>
    </hardware>
    <joint name="hinge">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <param name="actuator_name">hinge_vel</param>
    </joint>
    <sensor name="imu">
      <state_interface name="orientation.x"/>
      <state_interface name="orientation.y"/>
      <state_interface name="orientation.z"/>
      <state_interface name="orientation.w"/>
      <state_interface name="angular_velocity.x"/>
      <state_interface name="angular_velocity.y"/>
      <state_interface name="angular_velocity.z"/>
      <state_interface name="linear_acceleration.x"/>
      <state_interface name="linear_acceleration.y"/>
      <state_interface name="linear_acceleration.z"/>
      <param name="mujoco_orientation_sensor">imu_quat</param>
      <param name="mujoco_gyro_sensor">imu_gyro</param>
      <param name="mujoco_accel_sensor">imu_acc</param>
      <param name="frame_id">imu_link</param>
      <param name="topic">/perf/imu</param>
      <param name="update_rate">1000.0</param>
    </sensor>
    <sensor name="camera">
      <param name="mujoco_type">camera</param>
      <param name="mujoco_camera_name">cam</param>
      <param name="frame_id">camera_link</param>
      <param name="optical_frame_id">camera_optical_frame</param>
      <param name="image_topic">/perf/camera/image_raw</param>
      <param name="depth_topic">/perf/camera/depth</param>
      <param name="camera_info_topic">/perf/camera/info</param>
      <param name="width">64</param>
      <param name="height">48</param>
      <param name="enable_rgb">true</param>
      <param name="enable_depth">true</param>
      <param name="update_rate">100.0</param>
    </sensor>
    <sensor name="front_lidar">
      <param name="mujoco_type">lidar</param>
      <param name="sensor_prefix">front_lidar</param>
      <param name="frame_id">front_lidar_link</param>
      <param name="scan_topic">/perf/lidar/scan</param>
      <param name="angle_min">-0.2</param>
      <param name="angle_max">0.2</param>
      <param name="angle_increment">0.2</param>
      <param name="range_min">0.1</param>
      <param name="range_max">5.0</param>
      <param name="update_rate">100.0</param>
    </sensor>
  </ros2_control>
</robot>)";

  std::vector<hardware_interface::HardwareInfo> infos =
      hardware_interface::parse_control_resources_from_urdf(urdf);
  return infos.front();
}

template <typename InterfaceT>
InterfaceT* find_interface(std::vector<InterfaceT>& interfaces, const std::string& prefix,
                           const std::string& name) {
  for (auto& interface : interfaces) {
    if (interface.get_prefix_name() == prefix && interface.get_interface_name() == name) {
      return &interface;
    }
  }
  return nullptr;
}

}  // namespace

int run_main(int argc, char** argv) {
  std::size_t iterations = 200;
  double soak_seconds = 0.0;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--iterations" && index + 1 < argc) {
      iterations = static_cast<std::size_t>(std::stoull(argv[++index]));
    } else if (argument == "--soak-seconds" && index + 1 < argc) {
      soak_seconds = std::stod(argv[++index]);
    } else {
      std::cerr << "Unknown argument: " << argument << '\n';
      return 2;
    }
  }
  if (soak_seconds < 0.0) {
    std::cerr << "--soak-seconds must be non-negative\n";
    return 2;
  }

  if (!rclcpp::ok()) {
    int init_argc = 1;
    const char* init_argv[] = {"perf_read_write_loop", nullptr};
    rclcpp::init(init_argc, init_argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  }

  const TempModelFile model(R"(
<mujoco model="hardware_perf_loop">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body name="sensor_body">
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <site name="imu_site" pos="0 0 0"/>
      <site name="front_lidar_site_0" pos="0 0 0" zaxis="1 0 0"/>
      <site name="front_lidar_site_1" pos="0 0 0" zaxis="1 0 0"/>
      <site name="front_lidar_site_2" pos="0 0 0" zaxis="1 0 0"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
    <body name="far_target" pos="3 0 0">
      <geom type="box" size="0.05 0.05 0.05" contype="0" conaffinity="0"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
    <rangefinder name="front_lidar-0" site="front_lidar_site_0"/>
    <rangefinder name="front_lidar-1" site="front_lidar_site_1"/>
    <rangefinder name="front_lidar-2" site="front_lidar_site_2"/>
  </sensor>
</mujoco>)");

  mujoco_hardware::MuJoCoHardwareInterface hardware;
  const auto hardware_info = parse_sensor_hardware_info(model.path.string());
  if (hardware.on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS) {
    std::cerr << "on_init failed\n";
    return 1;
  }

  auto state_interfaces = hardware.export_state_interfaces();
  auto command_interfaces = hardware.export_command_interfaces();
  auto* joint_command =
      find_interface(command_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  if (joint_command == nullptr) {
    std::cerr << "joint velocity command interface not found\n";
    return 1;
  }

  if (hardware.prepare_command_mode_switch({"hinge/velocity"}, {}) !=
      hardware_interface::return_type::OK) {
    std::cerr << "prepare_command_mode_switch failed\n";
    return 1;
  }
  if (hardware.perform_command_mode_switch({}, {}) != hardware_interface::return_type::OK) {
    std::cerr << "perform_command_mode_switch failed\n";
    return 1;
  }

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  if (hardware.on_activate(inactive_state) != hardware_interface::CallbackReturn::SUCCESS) {
    std::cerr << "on_activate failed\n";
    return 1;
  }

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
  const bool soak_mode = soak_seconds > 0.0;
  std::vector<double> write_ms;
  std::vector<double> read_ms;
  std::vector<double> loop_ms;
  if (!soak_mode) {
    write_ms.reserve(iterations);
    read_ms.reserve(iterations);
    loop_ms.reserve(iterations);
  }

  const auto soak_start = std::chrono::steady_clock::now();
  const auto soak_deadline =
      soak_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(soak_seconds));
  std::size_t completed_iterations = 0;
  while (true) {
    if (!soak_mode && completed_iterations >= iterations) {
      break;
    }
    if (soak_mode && std::chrono::steady_clock::now() >= soak_deadline) {
      break;
    }

    joint_command->set_value(completed_iterations % 2 == 0 ? 0.5 : -0.5);
    const auto loop_start = std::chrono::steady_clock::now();
    const auto write_start = loop_start;
    if (hardware.write(time, period) != hardware_interface::return_type::OK) {
      std::cerr << "write failed\n";
      return 1;
    }
    const auto after_write = std::chrono::steady_clock::now();
    if (hardware.read(time, period) != hardware_interface::return_type::OK) {
      std::cerr << "read failed\n";
      return 1;
    }
    const auto after_read = std::chrono::steady_clock::now();

    if (!soak_mode) {
      write_ms.push_back(
          std::chrono::duration<double, std::milli>(after_write - write_start).count());
      read_ms.push_back(
          std::chrono::duration<double, std::milli>(after_read - after_write).count());
      loop_ms.push_back(std::chrono::duration<double, std::milli>(after_read - loop_start).count());
    }
    ++completed_iterations;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto soak_end = std::chrono::steady_clock::now();

  hardware.on_deactivate(inactive_state);
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  std::cout << "scenario=hardware_read_write_loop\n";
  std::cout << "iterations=" << completed_iterations << '\n';
  std::cout << "wall_seconds=" << std::chrono::duration<double>(soak_end - soak_start).count()
            << '\n';
  if (!soak_mode) {
    std::cout << "write_ms_mean=" << mean(write_ms) << '\n';
    std::cout << "write_ms_p95=" << p95(write_ms) << '\n';
    std::cout << "read_ms_mean=" << mean(read_ms) << '\n';
    std::cout << "read_ms_p95=" << p95(read_ms) << '\n';
    std::cout << "loop_ms_mean=" << mean(loop_ms) << '\n';
    std::cout << "loop_ms_p95=" << p95(loop_ms) << '\n';
  }
  return 0;
}

}  // namespace mujoco_hardware

int main(int argc, char** argv) { return mujoco_hardware::run_main(argc, argv); }
