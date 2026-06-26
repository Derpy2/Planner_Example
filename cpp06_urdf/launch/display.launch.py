from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    cpp06_urdf_dir = get_package_share_directory("cpp06_urdf")
    default_model_path = os.path.join(cpp06_urdf_dir, "urdf/urdf", "demo01_helloworld.urdf")
    default_rviz_path = os.path.join(cpp06_urdf_dir, "rviz", "display.rviz")
    print("model_path: ", default_model_path)
    print("rviz_path: ", default_rviz_path)
    model = DeclareLaunchArgument(name="model", default_value=default_model_path)

    # robot_state_publisher: load URDF
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # joint_state_publisher: publish joint states (no movable joints here, but kept for compatibility)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    # Static TF: map -> odom (so the car shows up at the map origin)
    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # Global path planner
    global_planner = Node(
        package="cpp06_urdf",
        executable="global_planner_node",
        name="global_planner_node",
        output="screen",
    )

    # Local path planner
    local_planner = Node(
        package="cpp06_urdf",
        executable="local_planner_node",
        name="local_planner_node",
        output="screen",
    )

    # rviz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", default_rviz_path],
    )

    return LaunchDescription([
        model,
        robot_state_publisher,
        joint_state_publisher,
        map_to_odom,
        global_planner,
        local_planner,
        rviz2,
    ])
