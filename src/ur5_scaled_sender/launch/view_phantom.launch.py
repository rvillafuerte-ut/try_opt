from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import os


def generate_launch_description():
    # Get URDF path
    urdf_file = '/home/utec/try_opt/src/haptic/omni_description/urdf/omni.urdf'
    
    # Read and process URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # Robot state publisher for Phantom Omni
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='phantom_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 100.0
            }],
            remappings=[
                ('/joint_states', '/phantom/joint_states'),
            ]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
