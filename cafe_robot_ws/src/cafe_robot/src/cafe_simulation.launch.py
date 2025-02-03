# Launches Gazebo, Nav2, and nodes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteLaunch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo
        ExecuteLaunch(
            launch_description=[
                (get_package_share_directory('turtlebot3_gazebo'), '/launch/robot.launch.py')
            ],
            launch_arguments={'world': 'cafe.world'}.items()
        ),
        
        # Launch Nav2
        Node(
            package='nav2_bringup',
            executable='navigation_launch.py'
        ),
        
        # Custom nodes
        Node(
            package='cafe_robot',
            executable='order_manager'
        ),
        Node(
            package='cafe_robot',
            executable='navigation_controller'
        )
    ])