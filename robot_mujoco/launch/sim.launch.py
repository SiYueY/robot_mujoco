from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLER_MANAGER_TIMEOUT_SECONDS = "10.0"
CONTROLLER_START_DELAY_SECONDS = 2.0


def load_robot_description(
    mujoco_model_path: str,
    robot_description_template: str,
    initial_keyframe: str,
) -> str:
    template_path = Path(robot_description_template).expanduser().resolve()
    robot_description = template_path.read_text(encoding="utf-8")
    robot_description = robot_description.replace(
        "__MUJOCO_MODEL_PATH__", str(Path(mujoco_model_path).expanduser().resolve())
    )
    robot_description = robot_description.replace("__RENDER_MODE__", "viewer")
    robot_description = robot_description.replace("__INITIAL_KEYFRAME__", initial_keyframe)
    return robot_description


def launch_setup(context, *args, **kwargs):
    mujoco_model_path = LaunchConfiguration("mujoco_model_path").perform(context)
    robot_description_template = LaunchConfiguration("robot_description_template").perform(context)
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    initial_keyframe = LaunchConfiguration("initial_keyframe").perform(context)

    robot_description_parameters = {
        "robot_description": load_robot_description(
            mujoco_model_path, robot_description_template, initial_keyframe
        )
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
            controllers_yaml,
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
            controllers_yaml,
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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mujoco_model_path",
                description="MuJoCo XML path to load into robot_mujoco.",
            ),
            DeclareLaunchArgument(
                "robot_description_template",
                description="URDF template path used for robot_state_publisher and ros2_control.",
            ),
            DeclareLaunchArgument(
                "controllers_yaml",
                description="Controller parameter file passed to controller spawners.",
            ),
            DeclareLaunchArgument(
                "initial_keyframe",
                default_value="",
                description="Initial MuJoCo keyframe to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
