from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', 
            default_value='/home/utec/try_opt/urdf/ur5.urdf',
            description='Path to URDF file'),
        
        DeclareLaunchArgument('ctrl_hz', 
            default_value='250.0',
            description='Control frequency in Hz'),
        
        DeclareLaunchArgument('traj_duration', 
            default_value='30.0',
            description='Trajectory duration in seconds'),
        
        DeclareLaunchArgument('controller_name', 
            default_value='scaled_joint_trajectory_controller',
            description='Name of the trajectory controller'),
        
        DeclareLaunchArgument('haptic_scale_pos', 
            default_value='1.0',
            description='Scaling factor for haptic position (1.0 = 1:1 mapping)'),
        
        DeclareLaunchArgument('haptic_scale_rot', 
            default_value='0.5',
            description='Scaling factor for haptic rotation (1.0 = 1:1 mapping)'),
        
        # Axis inversion arguments
        DeclareLaunchArgument('sign_x', default_value='-1.0', description='X axis sign (1.0 or -1.0)'),
        DeclareLaunchArgument('sign_y', default_value='-1.0', description='Y axis sign (1.0 or -1.0)'),
        DeclareLaunchArgument('sign_z', default_value='1.0', description='Z axis sign (1.0 or -1.0)'),
        DeclareLaunchArgument('sign_roll', default_value='1.0', description='Roll sign (1.0 or -1.0)'),
        DeclareLaunchArgument('sign_pitch', default_value='-1.0', description='Pitch sign (1.0 or -1.0)'),
        DeclareLaunchArgument('sign_yaw', default_value='-1.0', description='Yaw sign (1.0 or -1.0)'),
        
        # Axis remapping arguments (0=X/Roll, 1=Y/Pitch, 2=Z/Yaw)
        DeclareLaunchArgument('map_x', default_value='2', description='Which phantom axis for robot X (0=X, 1=Y, 2=Z)'),
        DeclareLaunchArgument('map_y', default_value='0', description='Which phantom axis for robot Y (0=X, 1=Y, 2=Z)'),
        DeclareLaunchArgument('map_z', default_value='1', description='Which phantom axis for robot Z (0=X, 1=Y, 2=Z)'),
        DeclareLaunchArgument('map_roll', default_value='0', description='Which phantom axis for robot Roll (0=R, 1=P, 2=Y)'),
        DeclareLaunchArgument('map_pitch', default_value='1', description='Which phantom axis for robot Pitch (0=R, 1=P, 2=Y)'),
        DeclareLaunchArgument('map_yaw', default_value='2', description='Which phantom axis for robot Yaw (0=R, 1=P, 2=Y)'),
        
        Node(
            package='ur5_scaled_sender',
            executable='opt_teleop_haptic',
            name='opt_teleop_haptic',
            output='screen',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf_path'),
                'ctrl_hz': LaunchConfiguration('ctrl_hz'),
                'traj_duration': LaunchConfiguration('traj_duration'),
                'controller_name': LaunchConfiguration('controller_name'),
                'haptic_scale_pos': LaunchConfiguration('haptic_scale_pos'),
                'haptic_scale_rot': LaunchConfiguration('haptic_scale_rot'),
                'sign_x': LaunchConfiguration('sign_x'),
                'sign_y': LaunchConfiguration('sign_y'),
                'sign_z': LaunchConfiguration('sign_z'),
                'sign_roll': LaunchConfiguration('sign_roll'),
                'sign_pitch': LaunchConfiguration('sign_pitch'),
                'sign_yaw': LaunchConfiguration('sign_yaw'),
                'map_x': LaunchConfiguration('map_x'),
                'map_y': LaunchConfiguration('map_y'),
                'map_z': LaunchConfiguration('map_z'),
                'map_roll': LaunchConfiguration('map_roll'),
                'map_pitch': LaunchConfiguration('map_pitch'),
                'map_yaw': LaunchConfiguration('map_yaw'),
            }]
        ),
    ])
