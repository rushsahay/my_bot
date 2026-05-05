from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory("my_bot"),
        "config"
    )
    return LaunchDescription([
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename","my_robot.lua"
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": True}, {"resolution": 0.05}],
        ),
    ])