from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ackermann_chassis_driver")
    params = os.path.join(pkg, "config", "chassis.yaml")

    return LaunchDescription([
        Node(
            package="ackermann_chassis_driver",
            executable="chassis_driver_node",
            name="ackermann_chassis_driver",
            output="screen",
            parameters=[params],
        ),

        # 你需要补一条静态TF：base_link -> laser（按你的安装位置填写xyz/yaw）
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=["0.10", "0.0", "0.08", "0.0", "0.0", "0.0", "base_link", "laser"],
            output="screen"
        ),
    ])
