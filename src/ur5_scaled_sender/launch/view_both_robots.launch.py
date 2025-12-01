from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')
    use_fake_hardware_arg = DeclareLaunchArgument('use_fake_hardware', default_value='true')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='127.0.0.1')
    launch_ur5_arg = DeclareLaunchArgument('launch_ur5', default_value='true', description='Whether to launch the UR5 driver')

    # Get Phantom URDF
    omni_urdf_file = os.path.join(
        get_package_share_directory('omni_description'),
        'urdf',
        'omni.urdf'
    )
    
    with open(omni_urdf_file, 'r') as file:
        omni_description = file.read()

    # 1. Phantom Omni Driver & State Publisher
    omni_state_node = Node(
        package="omni_common",
        executable="omni_state",
        name="omni_state",
        output="screen",
        parameters=[
            {"omni_name": "phantom"},
            {"publish_rate": 1000},
            {"reference_frame": "world"},
            {"units": "mm"}
        ]
    )

    phantom_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='phantom_state_publisher',
        namespace='phantom',
        output='screen',
        parameters=[{
            'robot_description': omni_description,
            'frame_prefix': 'phantom_' 
        }],
        remappings=[
            ('/joint_states', '/phantom/joint_states')
        ]
    )

    # 2. UR5 Launch (using ur_robot_driver)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ),
        condition=IfCondition(LaunchConfiguration('launch_ur5')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'launch_rviz': 'false', # We will launch our own RViz
            'initial_joint_controller': 'scaled_joint_trajectory_controller'
        }.items()
    )

    # 3. Static Transforms
    # Connect phantom_base to world
    tf_world_phantom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_phantom',
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'world', 'phantom_base']
    )

    # Connect ur5 base_link to world
    tf_world_ur5 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_ur5',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 4. RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('ur5_scaled_sender'), 'rviz', 'dual_robot.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        ur_type_arg,
        use_fake_hardware_arg,
        robot_ip_arg,
        launch_ur5_arg,
        omni_state_node,
        phantom_rsp_node,
        ur_control_launch,
        tf_world_phantom,
        tf_world_ur5,
        rviz_node
    ])
