#!/usr/bin/env python
import os
import pathlib
import launch
import yaml
import xacro
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from nav2_common.launch import RewrittenYaml

PACKAGE_NAME = 'webots_ros2_suv'
USE_SIM_TIME = True

def get_ros2_nodes(*args):
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)

    node_sensors_webots = Node(
        package=PACKAGE_NAME,
        executable='node_sensors_webots',
        name='node_sensors_webots',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    node_ego_controller = Node(
        package=PACKAGE_NAME,
        executable='node_ego_controller',
        name='node_ego_controller',
        output='screen' ,
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    package_dir = get_package_share_directory(PACKAGE_NAME)
    urdf = os.path.join(
        package_dir,
        os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf'))))
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': USE_SIM_TIME, 'robot_description': robot_desc}],
            arguments=[urdf])

    static_transforms = [
        ["base_link", "imu_link"],
        ["base_link", "lidar"],
        ["map", "odom"],
        ["base_link", "range_finder"]
    ]
    static_transform_nodes = []
    for s in static_transforms:
        static_transform_nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0"] + s,
            parameters=[{'use_sim_time': USE_SIM_TIME}]
        ))

    return [
        state_publisher_node,
        node_sensors_webots,
        node_ego_controller,
    ] + static_transform_nodes


def generate_launch_description():
    package_dir = get_package_share_directory(PACKAGE_NAME)
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(world=PathJoinSubstitution([package_dir, 'worlds', world]), ros2_supervisor=True, stream=True)
    robot_description_path = os.path.join(package_dir, pathlib.Path(os.path.join(package_dir, 'resource', 'suv.urdf')))
    vehicle_driver = WebotsController(
        robot_name='vehicle',
        parameters=[
            {'robot_description': robot_description_path}
        ],
        respawn=True
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='robocross.wbt',
            description='Robocross simulation world'
        ),
        webots,
        webots._supervisor,
        vehicle_driver,
        launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
             target_action=webots,
             on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ] + get_ros2_nodes())
