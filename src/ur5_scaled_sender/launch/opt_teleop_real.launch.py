from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file optimizado para robot real con calibración de orientación.
    
    IMPORTANTE sobre 'sign':
    - Los valores DEBEN ser 1.0 o -1.0 (no usar -0.5, etc.)
    - Solo invierten la dirección, NO escalan
    - Para escalar, usar 'haptic_scale_pos' y 'haptic_scale_rot'
    
    Ejemplos de uso:
    
    # Configuración por defecto (robot orientado hacia adelante)
    ros2 launch ur5_scaled_sender opt_teleop_real.launch.py
    
    # Robot rotado 90° a la derecha (intercambiar X ↔ Y)
    ros2 launch ur5_scaled_sender opt_teleop_real.launch.py map_x:=1 map_y:=0
    
    # Robot rotado 180° (invertir X e Y)
    ros2 launch ur5_scaled_sender opt_teleop_real.launch.py sign_x:=-1.0 sign_y:=-1.0
    
    # Escalar movimiento al 50% (más seguro para pruebas)
    ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_pos:=0.5
    """
    
    return LaunchDescription([
        DeclareLaunchArgument('urdf_path', 
            default_value='/home/utec/try_opt/urdf/ur5.urdf',
            description='Path to URDF file'),
        
        DeclareLaunchArgument('ctrl_hz', 
            default_value='100.0',
            description='Control frequency in Hz (125 Hz recomendado para robot real)'),
        
        DeclareLaunchArgument('nmspace',
            default_value='',
            description='Namespace for joints (empty for single robot)'),
        
        DeclareLaunchArgument('filter_gain',
            default_value='0.6',
            description='Low-pass filter gain (0.0-1.0, mayor = más reactivo)'),
        
        DeclareLaunchArgument('max_joint_vel',
            default_value='2.5',
            description='Maximum joint velocity in rad/s'),
        
        # Escalas (ESTOS son los que multiplican)
        DeclareLaunchArgument('haptic_scale_pos', 
            default_value='1.3',
            description='Position scaling: robot_movement = phantom_movement * scale'),
        
        DeclareLaunchArgument('haptic_scale_rot', 
            default_value='0.5',
            description='Rotation scaling: robot_rotation = phantom_rotation * scale'),
        
        # Inversión de ejes (SOLO -1.0 o 1.0)
        DeclareLaunchArgument('sign_x', 
            default_value='-1.0', 
            description='X axis direction (1.0 or -1.0 ONLY)'),
        DeclareLaunchArgument('sign_y', 
            default_value='-1.0', 
            description='Y axis direction (1.0 or -1.0 ONLY)'),
        DeclareLaunchArgument('sign_z', 
            default_value='1.0', 
            description='Z axis direction (1.0 or -1.0 ONLY)'),
        DeclareLaunchArgument('sign_roll', 
            default_value='1.0', 
            description='Roll direction (1.0 or -1.0 ONLY)'),
        DeclareLaunchArgument('sign_pitch', 
            default_value='-1.0', 
            description='Pitch direction (1.0 or -1.0 ONLY)'),
        DeclareLaunchArgument('sign_yaw', 
            default_value='-1.0', 
            description='Yaw direction (1.0 or -1.0 ONLY)'),
        
        # Remapeo de ejes (0=X/Roll, 1=Y/Pitch, 2=Z/Yaw)
        DeclareLaunchArgument('map_x', 
            default_value='2', 
            description='Phantom axis for robot X (0=phantom_X, 1=phantom_Y, 2=phantom_Z)'),
        DeclareLaunchArgument('map_y', 
            default_value='0', 
            description='Phantom axis for robot Y (0=phantom_X, 1=phantom_Y, 2=phantom_Z)'),
        DeclareLaunchArgument('map_z', 
            default_value='1', 
            description='Phantom axis for robot Z (0=phantom_X, 1=phantom_Y, 2=phantom_Z)'),
        DeclareLaunchArgument('map_roll', 
            default_value='0', 
            description='Phantom axis for robot Roll'),
        DeclareLaunchArgument('map_pitch', 
            default_value='1', 
            description='Phantom axis for robot Pitch'),
        DeclareLaunchArgument('map_yaw', 
            default_value='2', 
            description='Phantom axis for robot Yaw'),
        
        Node(
            package='ur5_scaled_sender',
            executable='opt_teleop_haptic',
            name='opt_teleop_haptic',
            output='screen',
            parameters=[{
                'urdf_path': LaunchConfiguration('urdf_path'),
                'ctrl_hz': LaunchConfiguration('ctrl_hz'),
                'nmspace': LaunchConfiguration('nmspace'),
                'filter_gain': LaunchConfiguration('filter_gain'),
                'max_joint_vel': LaunchConfiguration('max_joint_vel'),
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
