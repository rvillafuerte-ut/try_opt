from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', default_value='/home/utec/try_opt/urdf/ur5.urdf'),
        DeclareLaunchArgument('x_end', default_value='0.0'),
        DeclareLaunchArgument('y_end', default_value='0.0'),
        DeclareLaunchArgument('z_end', default_value='0.0'),
        DeclareLaunchArgument('roll_des', default_value='0.0'),
        DeclareLaunchArgument('pitch_des', default_value='0.0'),
        DeclareLaunchArgument('yaw_des', default_value='0.0'),
        DeclareLaunchArgument('traj_duration', default_value='5.0'),
        DeclareLaunchArgument('ctrl_hz', default_value='100.0'),
        DeclareLaunchArgument('controller_name', default_value='scaled_joint_trajectory_controller'),
        
        Node(
            package='ur5_scaled_sender',
            executable='opt_teleop_pinocchio',
            name='opt_teleop',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf_path'),
                'x_end': LaunchConfiguration('x_end'),
                'y_end': LaunchConfiguration('y_end'),
                'z_end': LaunchConfiguration('z_end'),
                'roll_des': LaunchConfiguration('roll_des'),
                'pitch_des': LaunchConfiguration('pitch_des'),
                'yaw_des': LaunchConfiguration('yaw_des'),
                'traj_duration': LaunchConfiguration('traj_duration'),
                'ctrl_hz': LaunchConfiguration('ctrl_hz'),
                'controller_name': LaunchConfiguration('controller_name'),
            }],
            output='screen'
        )
    ])
