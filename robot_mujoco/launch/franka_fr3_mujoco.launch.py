from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from robot_mujoco.launch_utils import package_file


def generate_launch_description() -> LaunchDescription:
    common_launch = package_file("launch", "robot_mujoco.launch.py")
    default_model_path = str(package_file("model", "franka_fr3", "franka_fr3_scene.xml"))
    default_template_path = str(package_file("config", "franka_fr3", "franka_fr3.urdf.template"))
    default_controllers_path = str(package_file("config", "franka_fr3", "controllers.yaml"))
    default_rviz_config = str(package_file("config", "franka_fr3", "franka_fr3_mujoco.rviz"))
    default_robocasa_config = str(package_file("config", "franka_fr3", "robocasa_scene.yaml"))

    return LaunchDescription(
        [
            DeclareLaunchArgument("mujoco_model_path", default_value=default_model_path),
            DeclareLaunchArgument("robot_description_template", default_value=default_template_path),
            DeclareLaunchArgument("controllers_yaml", default_value=default_controllers_path),
            DeclareLaunchArgument("primary_controller", default_value="arm_position_controller"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("initial_keyframe", default_value="home"),
            DeclareLaunchArgument("render_mode", default_value="viewer"),
            DeclareLaunchArgument("use_robocasa_scene", default_value="false"),
            DeclareLaunchArgument("robocasa_config", default_value=default_robocasa_config),
            DeclareLaunchArgument("robocasa_output_xml", default_value=""),
            DeclareLaunchArgument("robocasa_task", default_value=""),
            DeclareLaunchArgument("robocasa_layout", default_value=""),
            DeclareLaunchArgument("robocasa_style", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(common_launch)),
                launch_arguments={
                    "robot_name": "franka_fr3",
                    "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
                    "robot_description_template": LaunchConfiguration("robot_description_template"),
                    "controllers_yaml": LaunchConfiguration("controllers_yaml"),
                    "primary_controller": LaunchConfiguration("primary_controller"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "initial_keyframe": LaunchConfiguration("initial_keyframe"),
                    "render_mode": LaunchConfiguration("render_mode"),
                    "use_robocasa_scene": LaunchConfiguration("use_robocasa_scene"),
                    "robocasa_config": LaunchConfiguration("robocasa_config"),
                    "robocasa_output_xml": LaunchConfiguration("robocasa_output_xml"),
                    "robocasa_task": LaunchConfiguration("robocasa_task"),
                    "robocasa_layout": LaunchConfiguration("robocasa_layout"),
                    "robocasa_style": LaunchConfiguration("robocasa_style"),
                }.items(),
            ),
        ]
    )
