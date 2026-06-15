from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLER_MANAGER_TIMEOUT_SECONDS = "10.0"
CONTROLLER_START_DELAY_SECONDS = 2.0


def load_robot_description(render_mode: str, initial_keyframe: str) -> str:
    example_share_dir = Path(get_package_share_directory("franka_fr3_mujoco"))
    urdf_template_path = example_share_dir / "config" / "fr3.urdf.template"
    mujoco_model_path = example_share_dir / "model" / "scene.xml"

    robot_description = urdf_template_path.read_text(encoding="utf-8")
    robot_description = robot_description.replace("__MUJOCO_MODEL_PATH__", str(mujoco_model_path))
    robot_description = robot_description.replace("__RENDER_MODE__", render_mode)
    robot_description = robot_description.replace("__INITIAL_KEYFRAME__", initial_keyframe)
    return robot_description


def launch_setup(context, *args, **kwargs):
    render_mode = LaunchConfiguration("render_mode").perform(context)
    initial_keyframe = LaunchConfiguration("initial_keyframe").perform(context)

    example_share_dir = Path(get_package_share_directory("franka_fr3_mujoco"))
    controllers_config_path = example_share_dir / "config" / "controllers.yaml"

    robot_description_parameters = {
        "robot_description": load_robot_description(render_mode, initial_keyframe)
    }

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CONTROLLER_MANAGER_TIMEOUT_SECONDS,
            "--service-call-timeout",
            CONTROLLER_MANAGER_TIMEOUT_SECONDS,
            "--param-file",
            str(controllers_config_path),
        ],
        output="screen",
    )

    arm_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            CONTROLLER_MANAGER_TIMEOUT_SECONDS,
            "--service-call-timeout",
            CONTROLLER_MANAGER_TIMEOUT_SECONDS,
            "--param-file",
            str(controllers_config_path),
        ],
        output="screen",
    )

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description_parameters,
                {"update_rate": 250, "use_sim_time": True},
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description_parameters, {"use_sim_time": True}],
            output="screen",
        ),
        TimerAction(
            period=CONTROLLER_START_DELAY_SECONDS,
            actions=[joint_state_broadcaster_spawner],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_position_controller_spawner],
            )
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    render_mode_arg = DeclareLaunchArgument(
        "render_mode",
        default_value="viewer",
        description="MuJoCo render mode for the robot_mujoco FR3 workspace example. Use viewer by default; switch to headless when no OpenGL viewer is available.",
    )
    initial_keyframe_arg = DeclareLaunchArgument(
        "initial_keyframe",
        default_value="home",
        description="Initial MuJoCo keyframe to load for the robot_mujoco FR3 workspace example.",
    )

    return LaunchDescription(
        [
            render_mode_arg,
            initial_keyframe_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
