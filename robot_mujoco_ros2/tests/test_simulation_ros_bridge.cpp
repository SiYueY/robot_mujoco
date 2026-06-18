#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "robot_mujoco_msgs/srv/reset_world.hpp"
#include "robot_mujoco_msgs/srv/set_realtime_factor.hpp"
#include "robot_mujoco_msgs/srv/step_simulation.hpp"
#include "robot_mujoco_ros2/config_builder.hpp"
#include "robot_mujoco_ros2/message_mapper.hpp"
#include "robot_mujoco_ros2/publish_channel.hpp"
#include "robot_mujoco_ros2/qos_profiles.hpp"
#include "robot_mujoco_ros2/simulation_ros_bridge.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_mujoco_ros2 {
namespace {

using namespace std::chrono_literals;

rclcpp::NodeOptions make_node_options() {
  rclcpp::NodeOptions options;
  options.use_global_arguments(false)
      .enable_rosout(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false)
      .use_clock_thread(false);
  return options;
}

hardware_interface::HardwareInfo parse_hardware_info(const std::string& urdf) {
  auto infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  EXPECT_EQ(infos.size(), 1u);
  return infos.front();
}

mujoco_simulation::CameraSample make_camera_sample(std::uint64_t timestamp_ns,
                                                   std::uint64_t sequence) {
  mujoco_simulation::CameraSample sample;
  sample.sequence = sequence;
  sample.timestamp_ns = timestamp_ns;
  sample.color = mujoco_simulation::ImageBuffer{.width = 2,
                                                .height = 1,
                                                .step = 6,
                                                .format = mujoco_simulation::PixelFormat::Rgb8,
                                                .data = {1, 2, 3, 4, 5, 6}};
  sample.depth =
      mujoco_simulation::DepthBuffer{.width = 2,
                                     .height = 1,
                                     .format = mujoco_simulation::DepthFormat::Float32Meters,
                                     .data = {0.5F, 1.5F}};
  sample.intrinsics.fx = 100.0;
  sample.intrinsics.fy = 110.0;
  sample.intrinsics.cx = 10.0;
  sample.intrinsics.cy = 11.0;
  sample.intrinsics.k = {100.0, 0.0, 10.0, 0.0, 110.0, 11.0, 0.0, 0.0, 1.0};
  sample.intrinsics.p = {100.0, 0.0, 10.0, 0.0, 0.0, 110.0, 11.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return sample;
}

class SimulationRosBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros_home_path_ = std::filesystem::temp_directory_path() /
                     ("simulation_ros_bridge_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(ros_home_path_ / "log");
    ::setenv("ROS_HOME", ros_home_path_.c_str(), 1);
    ::setenv("ROS_LOG_DIR", (ros_home_path_ / "log").c_str(), 1);
    if (!rclcpp::ok()) {
      int argc = 1;
      const char* argv[] = {"test_simulation_ros_bridge", nullptr};
      rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
  }

  void TearDown() override {
    bridge_.reset();
    if (executor_ != nullptr && subscriber_node_ != nullptr) {
      executor_->remove_node(subscriber_node_);
    }
    executor_.reset();
    subscriber_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (!ros_home_path_.empty()) {
      std::error_code error;
      std::filesystem::remove_all(ros_home_path_, error);
    }
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

  void create_subscriber_node(const std::string& node_name) {
    subscriber_node_ = std::make_shared<rclcpp::Node>(node_name, make_node_options());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(subscriber_node_);
  }

  std::unique_ptr<SimulationRosBridge> bridge_;
  rclcpp::Node::SharedPtr subscriber_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::filesystem::path ros_home_path_;
};

TEST_F(SimulationRosBridgeTest, PublishesClockFromSimulationTime) {
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_clock_test"},
      rclcpp::contexts::get_global_default_context(), []() { return true; });
  ASSERT_TRUE(bridge_->start().ok());
  create_subscriber_node("simulation_ros_bridge_clock_listener");

  std::mutex mutex;
  bool received = false;
  builtin_interfaces::msg::Time last_clock;
  auto subscription = subscriber_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos_profiles::clock(),
      [&mutex, &received, &last_clock](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        received = true;
        last_clock = message->clock;
      });

  (void)subscription;
  bridge_->update_sim_time(rclcpp::Time(123456789LL, RCL_ROS_TIME));
  ASSERT_TRUE(spin_until([&mutex, &received, &last_clock]() {
    std::lock_guard<std::mutex> lock(mutex);
    return received && last_clock.nanosec == 123456789U;
  }));
}

TEST_F(SimulationRosBridgeTest, ControlServicesInvokeCallbacksAndReturnSuccess) {
  std::atomic<int> start_calls{0};
  std::atomic<int> stop_calls{0};
  std::atomic<int> pause_calls{0};
  std::atomic<int> resume_calls{0};
  std::atomic<int> step_calls{0};
  std::atomic<double> last_realtime_factor{0.0};
  std::string last_keyframe;

  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_control_service_test"},
      rclcpp::contexts::get_global_default_context(), []() { return true; },
      SimulationRosBridge::StatusCallback{},
      [&start_calls]() {
        ++start_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&stop_calls]() {
        ++stop_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&pause_calls]() {
        ++pause_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&resume_calls]() {
        ++resume_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&step_calls](uint32_t) {
        ++step_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&last_realtime_factor](double realtime_factor) {
        last_realtime_factor.store(realtime_factor);
        return mujoco_simulation::Status::Ok();
      },
      [&last_keyframe](const std::string& keyframe) {
        last_keyframe = keyframe;
        return mujoco_simulation::Status::Ok();
      });
  ASSERT_TRUE(bridge_->start().ok());
  create_subscriber_node("simulation_ros_bridge_control_service_client");

  auto start_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/start");
  auto stop_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/stop");
  auto pause_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/pause");
  auto resume_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/resume");
  auto step_client =
      subscriber_node_->create_client<robot_mujoco_msgs::srv::StepSimulation>("/step");
  auto set_realtime_factor_client =
      subscriber_node_->create_client<robot_mujoco_msgs::srv::SetRealtimeFactor>(
          "/set_realtime_factor");
  auto load_keyframe_client =
      subscriber_node_->create_client<robot_mujoco_msgs::srv::ResetWorld>("/load_keyframe");

  ASSERT_TRUE(spin_until([&]() {
    return start_client->wait_for_service(0s) && stop_client->wait_for_service(0s) &&
           pause_client->wait_for_service(0s) && resume_client->wait_for_service(0s) &&
           step_client->wait_for_service(0s) && set_realtime_factor_client->wait_for_service(0s) &&
           load_keyframe_client->wait_for_service(0s);
  }));

  auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto step_request = std::make_shared<robot_mujoco_msgs::srv::StepSimulation::Request>();
  step_request->steps = 3;
  auto realtime_request = std::make_shared<robot_mujoco_msgs::srv::SetRealtimeFactor::Request>();
  realtime_request->realtime_factor = 2.5;
  auto keyframe_request = std::make_shared<robot_mujoco_msgs::srv::ResetWorld::Request>();
  keyframe_request->keyframe = "home";

  auto start_future = start_client->async_send_request(trigger_request);
  auto stop_future = stop_client->async_send_request(trigger_request);
  auto pause_future = pause_client->async_send_request(trigger_request);
  auto resume_future = resume_client->async_send_request(trigger_request);
  auto step_future = step_client->async_send_request(step_request);
  auto realtime_future = set_realtime_factor_client->async_send_request(realtime_request);
  auto keyframe_future = load_keyframe_client->async_send_request(keyframe_request);

  ASSERT_TRUE(spin_until([&]() {
    return start_future.wait_for(0s) == std::future_status::ready &&
           stop_future.wait_for(0s) == std::future_status::ready &&
           pause_future.wait_for(0s) == std::future_status::ready &&
           resume_future.wait_for(0s) == std::future_status::ready &&
           step_future.wait_for(0s) == std::future_status::ready &&
           realtime_future.wait_for(0s) == std::future_status::ready &&
           keyframe_future.wait_for(0s) == std::future_status::ready;
  }));

  EXPECT_TRUE(start_future.get()->success);
  EXPECT_TRUE(stop_future.get()->success);
  EXPECT_TRUE(pause_future.get()->success);
  EXPECT_TRUE(resume_future.get()->success);
  EXPECT_TRUE(step_future.get()->success);
  EXPECT_TRUE(realtime_future.get()->success);
  EXPECT_TRUE(keyframe_future.get()->success);
  EXPECT_EQ(start_calls.load(), 1);
  EXPECT_EQ(stop_calls.load(), 1);
  EXPECT_EQ(pause_calls.load(), 1);
  EXPECT_EQ(resume_calls.load(), 1);
  EXPECT_EQ(step_calls.load(), 1);
  EXPECT_DOUBLE_EQ(last_realtime_factor.load(), 2.5);
  EXPECT_EQ(last_keyframe, "home");
}

TEST_F(SimulationRosBridgeTest, ServiceGateRejectsRequestsWhenInactive) {
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_gate_test"},
      rclcpp::contexts::get_global_default_context(), []() { return false; },
      []() { return mujoco_simulation::Status::Ok(); });
  ASSERT_TRUE(bridge_->start().ok());
  create_subscriber_node("simulation_ros_bridge_gate_client");

  auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/reset");
  ASSERT_TRUE(spin_until([&]() { return client->wait_for_service(0s); }));

  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_TRUE(spin_until([&]() { return future.wait_for(0s) == std::future_status::ready; }));
  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Simulation ROS bridge is not active.");
}

TEST_F(SimulationRosBridgeTest, PublishesBundleWithExpectedTimestamps) {
  SimulationRosBridgeConfig config;
  config.node_name = "simulation_ros_bridge_publish_test";
  config.imus.push_back({.name = "imu", .frame_id = "imu_link", .topic = "/test/imu"});
  config.cameras.push_back({.name = "camera",
                            .frame_id = "camera_optical_frame",
                            .rgb_topic = "/test/camera/rgb",
                            .depth_topic = "/test/camera/depth",
                            .camera_info_topic = "/test/camera/info",
                            .width = 2,
                            .height = 1,
                            .enable_rgb = true,
                            .enable_depth = true});
  config.lidars.push_back(
      {.name = "lidar", .frame_id = "lidar_link", .topic = "/test/lidar", .sample_count = 2});
  bridge_ = std::make_unique<SimulationRosBridge>(
      std::move(config), rclcpp::contexts::get_global_default_context(), []() { return true; });
  ASSERT_TRUE(bridge_->start().ok());
  create_subscriber_node("simulation_ros_bridge_publish_listener");

  std::mutex mutex;
  bool clock_received = false;
  bool imu_received = false;
  bool rgb_received = false;
  bool depth_received = false;
  bool info_received = false;
  bool lidar_received = false;
  builtin_interfaces::msg::Time clock_message;
  sensor_msgs::msg::Imu imu_message;
  sensor_msgs::msg::Image rgb_message;
  sensor_msgs::msg::Image depth_message;
  sensor_msgs::msg::CameraInfo info_message;
  sensor_msgs::msg::LaserScan lidar_message;

  auto clock_sub = subscriber_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", qos_profiles::clock(), [&](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        clock_received = true;
        clock_message = message->clock;
      });
  auto imu_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/test/imu", qos_profiles::sensor_data(),
      [&](const sensor_msgs::msg::Imu::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        imu_received = true;
        imu_message = *message;
      });
  auto rgb_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/rgb", qos_profiles::sensor_data(),
      [&](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        rgb_received = true;
        rgb_message = *message;
      });
  auto depth_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/depth", qos_profiles::sensor_data(),
      [&](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        depth_received = true;
        depth_message = *message;
      });
  auto info_sub = subscriber_node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/test/camera/info", qos_profiles::sensor_data(),
      [&](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        info_received = true;
        info_message = *message;
      });
  auto lidar_sub = subscriber_node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/test/lidar", qos_profiles::sensor_data(),
      [&](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        lidar_received = true;
        lidar_message = *message;
      });

  (void)clock_sub;
  (void)imu_sub;
  (void)rgb_sub;
  (void)depth_sub;
  (void)info_sub;
  (void)lidar_sub;

  const rclcpp::Time sim_time(2'000'000'000LL, RCL_ROS_TIME);
  auto camera_sample = make_camera_sample(1'500'000'000ULL, 7U);
  PublishBundle bundle;
  bundle.sim_time = sim_time;
  bundle.imus.push_back({.publisher_index = 0,
                         .sample = {.sequence = 1,
                                    .timestamp_ns = 0,
                                    .orientation = {0.0, 0.0, 0.0, 1.0},
                                    .angular_velocity = {1.0, 2.0, 3.0},
                                    .linear_acceleration = {4.0, 5.0, 6.0}}});
  bundle.lidars.push_back({.publisher_index = 0,
                           .sample = {.sequence = 2,
                                      .timestamp_ns = 0,
                                      .angle_min = -0.1,
                                      .angle_max = 0.1,
                                      .angle_increment = 0.2,
                                      .range_min = 0.2,
                                      .range_max = 5.0,
                                      .ranges = {1.0, 2.0},
                                      .intensities = {3.0, 4.0}}});
  bundle.cameras.push_back({.publisher_index = 0, .sample = &camera_sample});

  ASSERT_TRUE(bridge_->enqueue_publish_bundle(bundle).ok());
  ASSERT_TRUE(spin_until([&]() {
    std::lock_guard<std::mutex> lock(mutex);
    return clock_received && imu_received && rgb_received && depth_received && info_received &&
           lidar_received;
  }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(clock_message.sec, 2);
  EXPECT_EQ(imu_message.header.stamp.sec, 2);
  EXPECT_EQ(lidar_message.header.stamp.sec, 2);
  EXPECT_EQ(rgb_message.header.stamp.sec, 1);
  EXPECT_EQ(depth_message.header.stamp.sec, 1);
  EXPECT_EQ(info_message.header.stamp.sec, 1);
  EXPECT_EQ(rgb_message.data.size(), 6u);
  EXPECT_EQ(depth_message.data.size(), sizeof(float) * 2u);
}

TEST(PublishChannelTest, SmallSnapshotLatestWins) {
  PublishChannel channel({.imu_count = 1, .lidar_count = 0, .camera_queue_capacity = 1});

  PublishBundle first;
  first.sim_time = rclcpp::Time(1'000'000'000LL, RCL_ROS_TIME);
  first.imus.push_back({.publisher_index = 0, .sample = {.sequence = 1, .timestamp_ns = 0}});
  PublishBundle second = first;
  second.sim_time = rclcpp::Time(2'000'000'000LL, RCL_ROS_TIME);
  second.imus[0].sample.sequence = 2;

  ASSERT_TRUE(channel.publish_bundle(first).ok());
  ASSERT_TRUE(channel.publish_bundle(second).ok());

  SmallPublishSnapshot snapshot;
  ASSERT_TRUE(channel.consume_latest_small_snapshot(&snapshot));
  EXPECT_EQ(snapshot.sim_time.nanoseconds(), second.sim_time.nanoseconds());
  EXPECT_EQ(snapshot.imus[0].sample.sequence, 2u);
  EXPECT_FALSE(channel.consume_latest_small_snapshot(&snapshot));
}

TEST(PublishChannelTest, CameraQueueDropsOldFramesWithoutGrowing) {
  PublishChannel channel(
      {.imu_count = 0,
       .lidar_count = 0,
       .camera_frames = {{.width = 2, .height = 1, .enable_rgb = true, .enable_depth = true}},
       .camera_queue_capacity = 1});
  EXPECT_EQ(channel.camera_slot_count(), 1u);

  auto first_frame = make_camera_sample(1'000'000'000ULL, 1U);
  auto second_frame = make_camera_sample(2'000'000'000ULL, 2U);
  PublishBundle first;
  first.cameras.push_back({.publisher_index = 0, .sample = &first_frame});
  PublishBundle second;
  second.cameras.push_back({.publisher_index = 0, .sample = &second_frame});

  ASSERT_TRUE(channel.publish_bundle(first).ok());
  ASSERT_TRUE(channel.publish_bundle(second).ok());

  CameraFrame frame;
  ASSERT_TRUE(channel.pop_camera_frame(&frame));
  EXPECT_EQ(frame.sequence, 2u);
  EXPECT_FALSE(channel.pop_camera_frame(&frame));
}

TEST(QosProfilesTest, UsesKeepLastOneBestEffortVolatile) {
  const auto clock_qos = qos_profiles::clock().get_rmw_qos_profile();
  EXPECT_EQ(clock_qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(clock_qos.depth, 1u);
  EXPECT_EQ(clock_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(clock_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

  const auto sensor_qos = qos_profiles::sensor_data().get_rmw_qos_profile();
  EXPECT_EQ(sensor_qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  EXPECT_EQ(sensor_qos.depth, 1u);
  EXPECT_EQ(sensor_qos.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(sensor_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST(ConfigBuilderTest, BuildsRuntimeRosAndMappingOutputs) {
  const std::string urdf = R"(
<robot name="test_robot">
  <ros2_control name="mujoco_system" type="system">
    <hardware>
      <plugin>robot_mujoco_ros2/MuJoCoHardwareInterface</plugin>
      <param name="mujoco_model_path">/tmp/test_model.xml</param>
      <param name="render_mode">headless</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <param name="actuator_name">joint1_vel</param>
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
    </sensor>
  </ros2_control>
</robot>)";

  std::string error_message;
  AdapterConfigBundle config;
  ASSERT_TRUE(build_adapter_config(parse_hardware_info(urdf), &config, error_message))
      << error_message;
  EXPECT_EQ(config.runtime_config.joints.size(), 1u);
  EXPECT_EQ(config.ros_interface_config.imus.size(), 1u);
  EXPECT_EQ(config.hardware_mapping_config.imu_names.size(), 1u);
}

TEST(MessageMapperTest, MapsImuAndCameraMessages) {
  const ImuPublisherConfig imu_config{.name = "imu", .frame_id = "imu_link", .topic = "/imu"};
  mujoco_simulation::ImuSample imu_sample;
  imu_sample.timestamp_ns = 0;
  imu_sample.orientation = {0.0, 0.0, 0.0, 1.0};
  imu_sample.angular_velocity = {1.0, 2.0, 3.0};
  imu_sample.linear_acceleration = {4.0, 5.0, 6.0};
  const auto imu_message = message_mapper::make_imu_message(
      imu_config, imu_sample, rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME));
  EXPECT_EQ(imu_message.header.frame_id, "imu_link");
  EXPECT_EQ(imu_message.header.stamp.sec, 10);

  const CameraPublisherConfig camera_config{.name = "camera",
                                            .frame_id = "camera_optical_frame",
                                            .rgb_topic = "/rgb",
                                            .depth_topic = "/depth",
                                            .camera_info_topic = "/info",
                                            .width = 2,
                                            .height = 1,
                                            .enable_rgb = true,
                                            .enable_depth = true};
  CameraFrame frame;
  frame.acquisition_stamp = rclcpp::Time(3'000'000'000LL, RCL_ROS_TIME);
  frame.width = 2;
  frame.height = 1;
  frame.rgb_step = 6;
  frame.depth_step = static_cast<std::uint32_t>(sizeof(float) * 2U);
  frame.has_rgb = true;
  frame.has_depth = true;
  frame.rgb_data = {1, 2, 3, 4, 5, 6};
  frame.depth_data.resize(sizeof(float) * 2U);
  const float depth_values[2] = {0.5F, 1.5F};
  std::memcpy(frame.depth_data.data(), depth_values, sizeof(depth_values));

  sensor_msgs::msg::Image rgb_message;
  rgb_message.data.resize(6);
  message_mapper::fill_rgb_image_message(camera_config, frame, &rgb_message);
  EXPECT_EQ(rgb_message.header.stamp.sec, 3);
  EXPECT_EQ(rgb_message.data[0], 1u);
}

}  // namespace
}  // namespace robot_mujoco_ros2
