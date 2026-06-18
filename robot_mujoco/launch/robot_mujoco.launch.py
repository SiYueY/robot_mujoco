import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_mujoco.launch_utils import (
    generate_robocasa_scene,
    load_robot_description,
    package_file,
    parse_bool,
)

CONTROLLER_MANAGER_TIMEOUT_SECONDS = "10.0"
CONTROLLER_START_DELAY_SECONDS = 2.0
SUPPORTED_ROBOTS = {"franka_fr3", "turtlebot3"}


def _has_graphical_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    robot_name = LaunchConfiguration("robot_name").perform(context)
    mujoco_model_path = LaunchConfiguration("mujoco_model_path").perform(context)
    robot_description_template = LaunchConfiguration("robot_description_template").perform(context)
    controllers_yaml = LaunchConfiguration("controllers_yaml").perform(context)
    primary_controller = LaunchConfiguration("primary_controller").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_rviz = parse_bool(LaunchConfiguration("use_rviz").perform(context))
    initial_keyframe = LaunchConfiguration("initial_keyframe").perform(context)
    render_mode = LaunchConfiguration("render_mode").perform(context)
    use_robocasa_scene = parse_bool(LaunchConfiguration("use_robocasa_scene").perform(context))
    robocasa_config = LaunchConfiguration("robocasa_config").perform(context)
    robocasa_output_xml = LaunchConfiguration("robocasa_output_xml").perform(context)
    robocasa_task = LaunchConfiguration("robocasa_task").perform(context)
    robocasa_layout = LaunchConfiguration("robocasa_layout").perform(context)
    robocasa_style = LaunchConfiguration("robocasa_style").perform(context)

    if robot_name not in SUPPORTED_ROBOTS:
        raise RuntimeError(
            f"Unsupported robot_name '{robot_name}'. Supported values: {sorted(SUPPORTED_ROBOTS)}"
        )

    if render_mode == "viewer" and not _has_graphical_display():
        print(
            "robot_mujoco.launch.py: render_mode=viewer requested without DISPLAY/WAYLAND_DISPLAY; "
            "falling back to render_mode=headless."
        )
        render_mode = "headless"
    if use_rviz and not _has_graphical_display():
        print(
            "robot_mujoco.launch.py: use_rviz=true requested without DISPLAY/WAYLAND_DISPLAY; "
            "skipping rviz2."
        )
        use_rviz = False

    if use_robocasa_scene:
        if not robocasa_config.strip():
            robocasa_config = str(package_file("config", robot_name, "robocasa_scene.yaml"))
        # The static FR3 scene uses a "home" keyframe, but RoboCasa-generated MJCF
        # and TurtleBot3 static scene uses a "home" keyframe. RoboCasa-generated
        # MJCF does not define those keyframes, so keep RoboCasa launch on the
        # model default unless the user provides a different existing keyframe
        # explicitly.
        if initial_keyframe == "home":
            initial_keyframe = ""
        mujoco_model_path = str(
            generate_robocasa_scene(
                config_path=robocasa_config,
                output_path=robocasa_output_xml,
                task_name=robocasa_task,
                layout_id=robocasa_layout,
                style_id=robocasa_style,
            )
        )

    robot_description_parameters = {
        "robot_description": load_robot_description(
            mujoco_model_path=mujoco_model_path,
            robot_description_template=robot_description_template,
            initial_keyframe=initial_keyframe,
            render_mode=render_mode,
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

    primary_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            primary_controller,
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

    actions = [
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
                on_exit=[primary_controller_spawner],
            )
        ),
    ]

    if use_rviz:
        rviz_arguments = ["-d", rviz_config] if rviz_config else []
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=rviz_arguments,
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", description="Robot identifier for this launch."),
            DeclareLaunchArgument(
                "mujoco_model_path",
                description="MuJoCo XML path to load when RoboCasa is disabled.",
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
                "primary_controller",
                description="Primary controller name to spawn after joint_state_broadcaster.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="RViz config path when use_rviz is enabled.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Whether to start RViz for this launch.",
            ),
            DeclareLaunchArgument(
                "initial_keyframe",
                default_value="",
                description="Initial MuJoCo keyframe to load.",
            ),
            DeclareLaunchArgument(
                "render_mode",
                default_value="viewer",
                description="MuJoCo render mode.",
            ),
            DeclareLaunchArgument(
                "use_robocasa_scene",
                default_value="false",
                description="Whether to generate and use a RoboCasa scene instead of a static XML.",
            ),
            DeclareLaunchArgument(
                "robocasa_config",
                default_value="",
                description="RoboCasa YAML config path used when RoboCasa scenes are enabled.",
            ),
            DeclareLaunchArgument(
                "robocasa_output_xml",
                default_value="",
                description="Output XML path used when RoboCasa scenes are enabled.",
            ),
            DeclareLaunchArgument(
                "robocasa_task",
                default_value="",
                description="Optional RoboCasa task override.",
            ),
            DeclareLaunchArgument(
                "robocasa_layout",
                default_value="",
                description="Optional RoboCasa layout override.",
            ),
            DeclareLaunchArgument(
                "robocasa_style",
                default_value="",
                description="Optional RoboCasa style override.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
