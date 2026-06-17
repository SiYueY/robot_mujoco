#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "mujoco_hardware/mujoco_hardware_interface.hpp"
#include "mujoco_ros2_bridge_msgs/srv/reset_world.hpp"
#include "mujoco_ros2_bridge_msgs/srv/set_realtime_factor.hpp"
#include "mujoco_ros2_bridge_msgs/srv/step_simulation.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mujoco_hardware {
namespace {

using namespace std::chrono_literals;

class MuJoCoHardwareInterfaceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros_home_path_ = std::filesystem::temp_directory_path() /
                     ("mujoco_hardware_interface_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(ros_home_path_ / "log");
    ::setenv("ROS_HOME", ros_home_path_.c_str(), 1);
    ::setenv("ROS_LOG_DIR", (ros_home_path_ / "log").c_str(), 1);
    if (!rclcpp::ok()) {
      int argc = 1;
      const char* argv[] = {"test_mujoco_hardware_interface", nullptr};
      rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
  }

  void TearDown() override {
    if (executor_ != nullptr && subscriber_node_ != nullptr) {
      executor_->remove_node(subscriber_node_);
    }
    executor_.reset();
    subscriber_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
    if (!ros_home_path_.empty()) {
      std::error_code error;
      std::filesystem::remove_all(ros_home_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ = temp_dir / "mujoco_hardware_interface_test.xml";
    std::ofstream output(model_path_);
    EXPECT_TRUE(output.is_open());
    output << xml_contents;
    output.close();
    return model_path_.string();
  }

  hardware_interface::HardwareInfo parse_hardware_info(const std::string& model_path) const {
    const std::string urdf = R"(
<robot name="test_robot">
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
  </ros2_control>
</robot>)";

    std::vector<hardware_interface::HardwareInfo> infos =
        hardware_interface::parse_control_resources_from_urdf(urdf);
    EXPECT_EQ(infos.size(), 1u);
    return infos.front();
  }

  hardware_interface::HardwareInfo parse_sensor_hardware_info(const std::string& model_path) const {
    const std::string urdf = R"(
<robot name="test_robot">
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
      <param name="topic">/test/imu</param>
      <param name="update_rate">1000.0</param>
    </sensor>
    <sensor name="camera">
      <param name="mujoco_type">camera</param>
      <param name="mujoco_camera_name">cam</param>
      <param name="frame_id">camera_link</param>
      <param name="optical_frame_id">camera_optical_frame</param>
      <param name="image_topic">/test/camera/image_raw</param>
      <param name="depth_topic">/test/camera/depth</param>
      <param name="camera_info_topic">/test/camera/info</param>
      <param name="width">64</param>
      <param name="height">48</param>
      <param name="enable_rgb">true</param>
      <param name="enable_depth">true</param>
      <param name="update_rate">1000.0</param>
    </sensor>
    <sensor name="front_lidar">
      <param name="mujoco_type">lidar</param>
      <param name="sensor_prefix">front_lidar</param>
      <param name="frame_id">front_lidar_link</param>
      <param name="scan_topic">/test/lidar/scan</param>
      <param name="angle_min">-0.2</param>
      <param name="angle_max">0.2</param>
      <param name="angle_increment">0.2</param>
      <param name="range_min">0.1</param>
      <param name="range_max">5.0</param>
      <param name="update_rate">1000.0</param>
    </sensor>
  </ros2_control>
</robot>)";

    std::vector<hardware_interface::HardwareInfo> infos =
        hardware_interface::parse_control_resources_from_urdf(urdf);
    EXPECT_EQ(infos.size(), 1u);
    return infos.front();
  }

  void create_subscriber_node(const std::string& node_name) {
    subscriber_node_ = std::make_shared<rclcpp::Node>(node_name);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(subscriber_node_);
  }

  bool spin_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (executor_ != nullptr) {
        executor_->spin_some();
      }
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> call_trigger_service(
      const std::string& service_name) {
    auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>(service_name);
    if (!spin_until([&client]() { return client->wait_for_service(0s); })) {
      return nullptr;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    if (!spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; })) {
      return nullptr;
    }

    return future.get();
  }

  std::shared_ptr<mujoco_ros2_bridge_msgs::srv::StepSimulation::Response> call_step_service(
      uint32_t steps) {
    auto client =
        subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::StepSimulation>("/step");
    if (!spin_until([&client]() { return client->wait_for_service(0s); })) {
      return nullptr;
    }

    auto request = std::make_shared<mujoco_ros2_bridge_msgs::srv::StepSimulation::Request>();
    request->steps = steps;
    auto future = client->async_send_request(request);
    if (!spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; })) {
      return nullptr;
    }

    return future.get();
  }

  std::shared_ptr<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor::Response>
  call_set_realtime_factor_service(double realtime_factor) {
    auto client = subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor>(
        "/set_realtime_factor");
    if (!spin_until([&client]() { return client->wait_for_service(0s); })) {
      return nullptr;
    }

    auto request = std::make_shared<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor::Request>();
    request->realtime_factor = realtime_factor;
    auto future = client->async_send_request(request);
    if (!spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; })) {
      return nullptr;
    }

    return future.get();
  }

  std::shared_ptr<mujoco_ros2_bridge_msgs::srv::ResetWorld::Response> call_load_keyframe_service(
      const std::string& keyframe) {
    auto client =
        subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::ResetWorld>("/load_keyframe");
    if (!spin_until([&client]() { return client->wait_for_service(0s); })) {
      return nullptr;
    }

    auto request = std::make_shared<mujoco_ros2_bridge_msgs::srv::ResetWorld::Request>();
    request->keyframe = keyframe;
    auto future = client->async_send_request(request);
    if (!spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; })) {
      return nullptr;
    }

    return future.get();
  }

  static hardware_interface::StateInterface* find_state_interface(
      std::vector<hardware_interface::StateInterface>& interfaces, const std::string& prefix,
      const std::string& name) {
    for (auto& interface : interfaces) {
      if (interface.get_prefix_name() == prefix && interface.get_interface_name() == name) {
        return &interface;
      }
    }
    return nullptr;
  }

  static hardware_interface::CommandInterface* find_command_interface(
      std::vector<hardware_interface::CommandInterface>& interfaces, const std::string& prefix,
      const std::string& name) {
    for (auto& interface : interfaces) {
      if (interface.get_prefix_name() == prefix && interface.get_interface_name() == name) {
        return &interface;
      }
    }
    return nullptr;
  }

  std::filesystem::path model_path_;
  std::filesystem::path ros_home_path_;
  rclcpp::Node::SharedPtr subscriber_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(MuJoCoHardwareInterfaceTest, ActivatesAndPropagatesJointCommandsThroughSimulation) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_hardware_info(model_path);

  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::StateInterface> state_interfaces =
      hardware.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      hardware.export_command_interfaces();
  ASSERT_EQ(state_interfaces.size(), 2u);
  ASSERT_EQ(command_interfaces.size(), 1u);

  ASSERT_EQ(hardware.prepare_command_mode_switch({"hinge/velocity"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  ASSERT_EQ(hardware.on_activate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  EXPECT_NEAR(state_interfaces[0].get_value(), 0.0, 1e-6);

  command_interfaces[0].set_value(1.0);

  bool moved = false;
  for (int i = 0; i < 40; ++i) {
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (state_interfaces[0].get_value() > 1e-4 && state_interfaces[1].get_value() > 1e-4) {
      moved = true;
      break;
    }
  }

  EXPECT_TRUE(moved);

  ASSERT_EQ(hardware.on_deactivate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(MuJoCoHardwareInterfaceTest, ReadPublishesClockAndSensorMessages) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_sensor_test">
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

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_sensor_hardware_info(model_path);

  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::StateInterface> state_interfaces =
      hardware.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      hardware.export_command_interfaces();
  auto* joint_position =
      find_state_interface(state_interfaces, "hinge", hardware_interface::HW_IF_POSITION);
  auto* joint_velocity =
      find_state_interface(state_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  auto* joint_command =
      find_command_interface(command_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  ASSERT_NE(joint_position, nullptr);
  ASSERT_NE(joint_velocity, nullptr);
  ASSERT_NE(joint_command, nullptr);

  create_subscriber_node("mujoco_hardware_sensor_listener");

  std::mutex mutex;
  bool clock_received = false;
  bool imu_received = false;
  bool image_received = false;
  bool depth_received = false;
  bool info_received = false;
  bool lidar_received = false;
  std::unordered_set<std::uint64_t> clock_stamps;

  const auto encode_stamp = [](const builtin_interfaces::msg::Time& stamp) -> std::uint64_t {
    return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
  };

  auto clock_sub = subscriber_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [&mutex, &clock_received, &clock_stamps,
       &encode_stamp](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        clock_received = true;
        clock_stamps.insert(encode_stamp(message->clock));
      });
  auto imu_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/test/imu", rclcpp::SensorDataQoS(),
      [&mutex, &imu_received](const sensor_msgs::msg::Imu::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        imu_received = true;
      });
  auto image_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/image_raw", rclcpp::SensorDataQoS(),
      [&mutex, &image_received](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        image_received = true;
      });
  auto depth_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/depth", rclcpp::SensorDataQoS(),
      [&mutex, &depth_received](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        depth_received = true;
      });
  auto info_sub = subscriber_node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/test/camera/info", rclcpp::SensorDataQoS(),
      [&mutex, &info_received](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        info_received = true;
      });
  auto lidar_sub = subscriber_node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/test/lidar/scan", rclcpp::SensorDataQoS(),
      [&mutex, &lidar_received](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        lidar_received = true;
      });
  (void)clock_sub;
  (void)imu_sub;
  (void)image_sub;
  (void)depth_sub;
  (void)info_sub;
  (void)lidar_sub;

  ASSERT_EQ(hardware.prepare_command_mode_switch({"hinge/velocity"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  ASSERT_EQ(hardware.on_activate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);

  bool observed_nonzero_timestamp = false;
  for (int i = 0; i < 40 && !observed_nonzero_timestamp; ++i) {
    joint_command->set_value(0.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    ASSERT_TRUE(spin_until([&mutex, &clock_received, &imu_received, &image_received,
                            &depth_received, &info_received, &lidar_received, &clock_stamps]() {
      std::lock_guard<std::mutex> lock(mutex);
      return clock_received && imu_received && image_received && depth_received && info_received &&
             lidar_received && !clock_stamps.empty();
    }));

    std::lock_guard<std::mutex> lock(mutex);
    for (const std::uint64_t stamp : clock_stamps) {
      if (stamp > 0U) {
        observed_nonzero_timestamp = true;
        break;
      }
    }
  }

  EXPECT_TRUE(observed_nonzero_timestamp);
  EXPECT_TRUE(std::isfinite(joint_position->get_value()));
  EXPECT_TRUE(std::isfinite(joint_velocity->get_value()));

  ASSERT_EQ(hardware.on_deactivate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(MuJoCoHardwareInterfaceTest, ResetServiceRequestsSimulationResetAndRestoresJointState) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_reset_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_hardware_info(model_path);
  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::StateInterface> state_interfaces =
      hardware.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      hardware.export_command_interfaces();
  auto* joint_position =
      find_state_interface(state_interfaces, "hinge", hardware_interface::HW_IF_POSITION);
  auto* joint_command =
      find_command_interface(command_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  ASSERT_NE(joint_position, nullptr);
  ASSERT_NE(joint_command, nullptr);

  ASSERT_EQ(hardware.prepare_command_mode_switch({"hinge/velocity"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);

  create_subscriber_node("mujoco_hardware_reset_client");

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  ASSERT_EQ(hardware.on_activate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);

  bool moved = false;
  for (int i = 0; i < 40; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value()) > 1e-3) {
      moved = true;
      break;
    }
  }
  ASSERT_TRUE(moved);

  auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/reset");
  ASSERT_TRUE(spin_until([&client]() { return client->wait_for_service(0s); }));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }));

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Reset requested.");

  bool reset_observed = false;
  for (int i = 0; i < 80; ++i) {
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value()) < 1e-5) {
      reset_observed = true;
      break;
    }
  }
  EXPECT_TRUE(reset_observed);

  ASSERT_EQ(hardware.on_deactivate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(MuJoCoHardwareInterfaceTest, ControlServicesPauseResumeStopAndStartSimulation) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_control_services_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_hardware_info(model_path);
  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::StateInterface> state_interfaces =
      hardware.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      hardware.export_command_interfaces();
  auto* joint_position =
      find_state_interface(state_interfaces, "hinge", hardware_interface::HW_IF_POSITION);
  auto* joint_command =
      find_command_interface(command_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  ASSERT_NE(joint_position, nullptr);
  ASSERT_NE(joint_command, nullptr);

  ASSERT_EQ(hardware.prepare_command_mode_switch({"hinge/velocity"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);

  create_subscriber_node("mujoco_hardware_control_services_client");

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  ASSERT_EQ(hardware.on_activate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);

  bool moved = false;
  for (int i = 0; i < 40; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value()) > 1e-3) {
      moved = true;
      break;
    }
  }
  ASSERT_TRUE(moved);

  const auto pause_response = call_trigger_service("/pause");
  ASSERT_NE(pause_response, nullptr);
  EXPECT_TRUE(pause_response->success);
  EXPECT_EQ(pause_response->message, "Simulation paused.");

  std::this_thread::sleep_for(10ms);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  const double paused_position = joint_position->get_value();
  bool advanced_while_paused = false;
  for (int i = 0; i < 20; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value() - paused_position) > 1e-5) {
      advanced_while_paused = true;
      break;
    }
  }
  EXPECT_FALSE(advanced_while_paused);

  const auto resume_response = call_trigger_service("/resume");
  ASSERT_NE(resume_response, nullptr);
  EXPECT_TRUE(resume_response->success);
  EXPECT_EQ(resume_response->message, "Simulation resumed.");

  bool resumed_motion = false;
  for (int i = 0; i < 40; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (joint_position->get_value() > paused_position + 1e-4) {
      resumed_motion = true;
      break;
    }
  }
  EXPECT_TRUE(resumed_motion);

  const auto stop_response = call_trigger_service("/stop");
  ASSERT_NE(stop_response, nullptr);
  EXPECT_TRUE(stop_response->success);
  EXPECT_EQ(stop_response->message, "Simulation stopped.");

  std::this_thread::sleep_for(10ms);
  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  const double stopped_position = joint_position->get_value();
  bool advanced_while_stopped = false;
  for (int i = 0; i < 20; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value() - stopped_position) > 1e-5) {
      advanced_while_stopped = true;
      break;
    }
  }
  EXPECT_FALSE(advanced_while_stopped);

  const auto step_response = call_step_service(1);
  ASSERT_NE(step_response, nullptr);
  EXPECT_TRUE(step_response->success);
  EXPECT_EQ(step_response->message, "Simulation stepped.");

  ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
  EXPECT_GT(joint_position->get_value(), stopped_position + 1e-5);
  const double stepped_position = joint_position->get_value();

  const auto start_response = call_trigger_service("/start");
  ASSERT_NE(start_response, nullptr);
  EXPECT_TRUE(start_response->success);
  EXPECT_EQ(start_response->message, "Simulation started.");

  bool restarted_motion = false;
  for (int i = 0; i < 40; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (joint_position->get_value() > stepped_position + 1e-4) {
      restarted_motion = true;
      break;
    }
  }
  EXPECT_TRUE(restarted_motion);

  ASSERT_EQ(hardware.on_deactivate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(MuJoCoHardwareInterfaceTest, LoadKeyframeServiceRequestsResetToNamedKeyframe) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_load_keyframe_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
  <keyframe>
    <key name="offset" qpos="0.75"/>
  </keyframe>
</mujoco>)");

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_hardware_info(model_path);
  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  std::vector<hardware_interface::StateInterface> state_interfaces =
      hardware.export_state_interfaces();
  std::vector<hardware_interface::CommandInterface> command_interfaces =
      hardware.export_command_interfaces();
  auto* joint_position =
      find_state_interface(state_interfaces, "hinge", hardware_interface::HW_IF_POSITION);
  auto* joint_command =
      find_command_interface(command_interfaces, "hinge", hardware_interface::HW_IF_VELOCITY);
  ASSERT_NE(joint_position, nullptr);
  ASSERT_NE(joint_command, nullptr);

  ASSERT_EQ(hardware.prepare_command_mode_switch({"hinge/velocity"}, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware.perform_command_mode_switch({}, {}), hardware_interface::return_type::OK);

  create_subscriber_node("mujoco_hardware_load_keyframe_client");

  const rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                               "inactive");
  ASSERT_EQ(hardware.on_activate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period = rclcpp::Duration::from_seconds(0.001);

  bool moved = false;
  for (int i = 0; i < 40; ++i) {
    joint_command->set_value(1.0);
    ASSERT_EQ(hardware.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(5ms);
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value()) > 1e-3) {
      moved = true;
      break;
    }
  }
  ASSERT_TRUE(moved);

  const auto stop_response = call_trigger_service("/stop");
  ASSERT_NE(stop_response, nullptr);
  EXPECT_TRUE(stop_response->success);
  EXPECT_EQ(stop_response->message, "Simulation stopped.");

  const auto response = call_load_keyframe_service("offset");
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Keyframe reset completed.");

  bool keyframe_observed = false;
  for (int i = 0; i < 10; ++i) {
    ASSERT_EQ(hardware.read(time, period), hardware_interface::return_type::OK);
    if (std::abs(joint_position->get_value() - 0.75) < 1e-4) {
      keyframe_observed = true;
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  EXPECT_TRUE(keyframe_observed) << "observed position=" << joint_position->get_value();

  ASSERT_EQ(hardware.on_deactivate(inactive_state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(MuJoCoHardwareInterfaceTest, SetRealtimeFactorServiceUpdatesRuntimeAndRejectsInvalidValues) {
  const std::string model_path = write_model(R"(
<mujoco model="hardware_interface_realtime_factor_test">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  MuJoCoHardwareInterface hardware;
  const hardware_interface::HardwareInfo hardware_info = parse_hardware_info(model_path);
  ASSERT_EQ(hardware.on_init(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  create_subscriber_node("mujoco_hardware_realtime_factor_client");

  const auto valid_response = call_set_realtime_factor_service(2.0);
  ASSERT_NE(valid_response, nullptr);
  EXPECT_TRUE(valid_response->success);
  EXPECT_EQ(valid_response->message, "Realtime factor updated.");

  const auto invalid_response = call_set_realtime_factor_service(0.0);
  ASSERT_NE(invalid_response, nullptr);
  EXPECT_FALSE(invalid_response->success);
  EXPECT_EQ(invalid_response->message,
            "SimulationScheduler realtime factor must be greater than zero.");
}

}  // namespace
}  // namespace mujoco_hardware
