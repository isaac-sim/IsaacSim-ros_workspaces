from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("publish_period_ms", default_value="20", description="publishing dt in milliseconds"),
        DeclareLaunchArgument("track_width", default_value="0.24", description="wheel separation distance (m)"),
        DeclareLaunchArgument("acceleration", default_value="0.0", description="acceleration, 0 means change speed as quickly as possible (ms^-2)"),
        DeclareLaunchArgument("steering_velocity", default_value="0.0", description="delta steering angle, 0 means change angle as quickly as possible (radians/s)"),
        
        Node(
            package='cmdvel_to_ackermann',
            executable='cmdvel_to_ackermann.py',
            name='cmdvel_to_ackermann',
            output="screen",
            parameters=[{
                'publish_period_ms': LaunchConfiguration('publish_period_ms'),
                'track_width': LaunchConfiguration('track_width'),
                'acceleration': LaunchConfiguration('acceleration'),
                'steering_velocity': LaunchConfiguration('steering_velocity')
            }]
        ),
    ])
