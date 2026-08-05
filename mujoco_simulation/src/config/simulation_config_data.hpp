#pragma once

namespace mujoco_simulation::config_names {

inline constexpr char kRobotMujoco[] = "robot_mujoco";
inline constexpr char kMujoco[] = "mujoco";
inline constexpr char kMjcf[] = "mjcf";
inline constexpr char kRobot[] = "robot";
inline constexpr char kSimulation[] = "simulation";
inline constexpr char kPhysics[] = "physics";
inline constexpr char kViewer[] = "viewer";
inline constexpr char kJoint[] = "joint";
inline constexpr char kImu[] = "imu";
inline constexpr char kCamera[] = "camera";
inline constexpr char kLidar[] = "lidar";
inline constexpr char kMobileBase[] = "mobile_base";
inline constexpr char kWheel[] = "wheel";
inline constexpr char kControl[] = "control";
inline constexpr char kLimit[] = "limit";
inline constexpr char kPosition[] = "position";
inline constexpr char kVelocity[] = "velocity";
inline constexpr char kEffort[] = "effort";

inline constexpr char kId[] = "id";
inline constexpr char kName[] = "name";
inline constexpr char kActuator[] = "actuator";
inline constexpr char kPeriod[] = "period";
inline constexpr char kUpdateRate[] = "update_rate";
inline constexpr char kMin[] = "min";
inline constexpr char kMax[] = "max";
inline constexpr char kStiffness[] = "stiffness";
inline constexpr char kDamping[] = "damping";
inline constexpr char kFrameId[] = "frame_id";
inline constexpr char kFramequatSensor[] = "framequat_sensor";
inline constexpr char kGyroSensor[] = "gyro_sensor";
inline constexpr char kAccelerometerSensor[] = "accelerometer_sensor";
inline constexpr char kOrientationCovariance[] = "orientation_covariance";
inline constexpr char kAngularVelocityCovariance[] = "angular_velocity_covariance";
inline constexpr char kLinearAccelerationCovariance[] = "linear_acceleration_covariance";
inline constexpr char kCameraName[] = "camera_name";
inline constexpr char kOpticalFrameId[] = "optical_frame_id";
inline constexpr char kWidth[] = "width";
inline constexpr char kHeight[] = "height";
inline constexpr char kEnableRgb[] = "enable_rgb";
inline constexpr char kEnableDepth[] = "enable_depth";
inline constexpr char kSensorPrefix[] = "sensor_prefix";
inline constexpr char kAngleMin[] = "angle_min";
inline constexpr char kAngleMax[] = "angle_max";
inline constexpr char kAngleIncrement[] = "angle_increment";
inline constexpr char kRangeMin[] = "range_min";
inline constexpr char kRangeMax[] = "range_max";
inline constexpr char kBaseBody[] = "base_body";
inline constexpr char kBaseFrameId[] = "base_frame_id";
inline constexpr char kOdomFrameId[] = "odom_frame_id";
inline constexpr char kWheelRadius[] = "wheel_radius";
inline constexpr char kWheelBase[] = "wheel_base";
inline constexpr char kTrackWidth[] = "track_width";
inline constexpr char kEnabled[] = "enabled";

inline constexpr char kModelPath[] = "model_path";
inline constexpr char kViewerStartupTimeout[] = "viewer_startup_timeout";
inline constexpr char kCameraRenderer[] = "camera_renderer";
inline constexpr char kMaxSceneGeometries[] = "max_scene_geometries";
inline constexpr char kCompletedTicketHistory[] = "completed_ticket_history";
inline constexpr char kPhysicsPeriod[] = "physics_period";
inline constexpr char kViewerPeriod[] = "viewer_period";
inline constexpr char kCovariance[] = "covariance";

inline constexpr char kJointKind[] = "joint";
inline constexpr char kImuKind[] = "IMU";
inline constexpr char kCameraKind[] = "camera";
inline constexpr char kLidarKind[] = "lidar";
inline constexpr char kMobileBaseKind[] = "mobile base";

}  // namespace mujoco_simulation::config_names
