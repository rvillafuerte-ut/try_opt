from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF files
    omni_urdf = os.path.join(
        get_package_share_directory('omni_description'),
        'urdf',
        'omni.urdf'
    )
    
    with open(omni_urdf, 'r') as file:
        omni_description = file.read()
    
    return LaunchDescription([
        # Omni state publisher (publishes /phantom/pose and joint states)
        Node(
            package="omni_common",
            executable="omni_state",
            output="screen",
            parameters=[
                {"omni_name": "phantom"},
                {"publish_rate": 1000},
                {"reference_frame": "world"},
                {"units": "mm"}
            ]
        ),
        
        # Robot state publisher for Omni (publishes TF tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='omni_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': omni_description,
                'frame_prefix': 'phantom_'
            }],
            remappings=[
                ('/joint_states', '/phantom/joint_states')
            ]
        ),
        
        # Static transform: world -> phantom base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_phantom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'phantom_base']
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('omni_description'), 'rviz', 'omni.rviz')]
            if os.path.exists(os.path.join(get_package_share_directory('omni_description'), 'rviz', 'omni.rviz'))
            else []
        ),
    ])
