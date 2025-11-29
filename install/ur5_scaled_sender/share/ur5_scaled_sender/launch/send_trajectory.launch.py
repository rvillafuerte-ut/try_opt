from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    controller_name = LaunchConfiguration('controller_name').perform(context)
    joint_names_csv = LaunchConfiguration('joint_names').perform(context)
    positions_csv = LaunchConfiguration('positions').perform(context)
    time_from_start = float(LaunchConfiguration('time_from_start_sec').perform(context))
    repeat = LaunchConfiguration('repeat').perform(context).lower() in ['1', 'true', 'yes']
    publish_rate_hz = float(LaunchConfiguration('publish_rate_hz').perform(context))

    try:
        positions = [float(x) for x in positions_csv.split(',') if x.strip() != '']
    except Exception:
        positions = [0.0, -1.9, 1.7, -1.7, -1.57, 0.0]

    # Default UR5 joint names
    default_joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    joint_names = [s.strip() for s in joint_names_csv.split(',') if s.strip()] if joint_names_csv else default_joint_names

    node = Node(
        package='ur5_scaled_sender',
        executable='scaled_traj_publisher',
        name='scaled_traj_publisher',
        parameters=[{
            'controller_name': controller_name,
            'joint_names': joint_names,
            'positions': positions,
            'time_from_start_sec': time_from_start,
            'repeat': repeat,
            'publish_rate_hz': publish_rate_hz,
        }]
    )

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller_name', default_value='scaled_joint_trajectory_controller',
            description='Name of the joint trajectory controller'),
        DeclareLaunchArgument(
            'joint_names',
            default_value='shoulder_pan_joint,shoulder_lift_joint,elbow_joint,wrist_1_joint,wrist_2_joint,wrist_3_joint',
            description='Comma-separated joint names in controller order'),
        DeclareLaunchArgument(
            'positions', default_value='0.0,-1.57.,1.57,0.0,0.0,0.0',
            description='Comma-separated target joint positions (rad) for 6 UR5 joints'),
        DeclareLaunchArgument(
            'time_from_start_sec', default_value='2.0',
            description='Execution time for the single point trajectory'),
        DeclareLaunchArgument(
            'repeat', default_value='false',
            description='If true, periodically re-publish the command'),
        DeclareLaunchArgument(
            'publish_rate_hz', default_value='0.5',
            description='Rate used when repeat=true'),
        OpaqueFunction(function=_launch_setup)
    ])
