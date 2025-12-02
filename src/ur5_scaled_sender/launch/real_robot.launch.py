from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')
    robot_ip_arg = DeclareLaunchArgument('robot_ip', default_value='192.168.10.103')
    kinematics_params_arg = DeclareLaunchArgument(
        'kinematics_params_file', 
        default_value=os.path.expanduser('~/my_robot_calibration.yaml')
    )
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='false')  # False por defecto en robot real
    launch_teleop_arg = DeclareLaunchArgument('launch_teleop', default_value='false',
        description='Launch teleoperation node automatically (default: false, start manually after robot is ready)')

    # Get Phantom URDF
    try:
        omni_urdf_file = os.path.join(
            get_package_share_directory('omni_description'),
            'urdf',
            'omni.urdf'
        )
        
        with open(omni_urdf_file, 'r') as file:
            omni_description = file.read()
    except Exception as e:
        print(f"Warning: Could not load Phantom URDF: {e}")
        omni_description = ""

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

    # 2. UR5 Real Robot Launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': 'false',  # Robot real
            'kinematics_params_file': LaunchConfiguration('kinematics_params_file'),
            'launch_rviz': 'false',  # Lanzamos nuestro propio RViz
            'initial_joint_controller': 'scaled_joint_trajectory_controller'
        }.items()
    )

    # 3. Static Transforms
    tf_world_phantom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_phantom',
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'world', 'phantom_base']
    )

    tf_world_ur5 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_ur5',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 4. RViz (opcional)
    rviz_config_file = os.path.join(
        get_package_share_directory('ur5_scaled_sender'), 'rviz', 'dual_robot.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    
    # Delay RViz para asegurar que todo esté cargado
    delayed_rviz = TimerAction(
        period=5.0,  # Aumentado a 5s para robot real
        actions=[rviz_node]
    )
    
    # 5. Teleoperation Node (opcional, recomendado iniciar manualmente)
    teleop_node = Node(
        package='ur5_scaled_sender',
        executable='opt_teleop_haptic',
        name='opt_teleop_haptic',
        output='screen',
        parameters=[{
            'urdf_path': '/home/utec/try_opt/urdf/ur5.urdf',
            'ctrl_hz': 125.0,
            'nmspace': '',
            'filter_gain': 0.6,
            'max_joint_vel': 2.5,
            'haptic_scale_pos': 2.5,
            'haptic_scale_rot': 1.0,
            'sign_x': -1.0,
            'sign_y': -1.0,
            'sign_z': 1.0,
            'sign_roll': 1.0,
            'sign_pitch': 1.0,
            'sign_yaw': 1.0,
            'map_x': 2,
            'map_y': 0,
            'map_z': 1,
            'map_roll': 2,
            'map_pitch': 0,
            'map_yaw': 1,
        }],
        condition=IfCondition(LaunchConfiguration('launch_teleop'))
    )
    
    # Delay teleop para asegurar que robot y phantom estén listos
    delayed_teleop = TimerAction(
        period=8.0,  # 8s después del launch
        actions=[teleop_node]
    )

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        kinematics_params_arg,
        launch_rviz_arg,
        launch_teleop_arg,
        omni_state_node,
        phantom_rsp_node,
        ur_control_launch,
        tf_world_phantom,
        tf_world_ur5,
        delayed_rviz,
        delayed_teleop
    ])
