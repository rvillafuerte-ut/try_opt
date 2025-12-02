from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo controlador del gripper
        Node(
            package='griper_control',
            executable='gripper_controller_node',
            name='gripper_controller',
            output='screen',
            parameters=[
                {'device': '/dev/ttyUSB0'},
                {'baudrate': 115200},
                {'slave_id': 9}
            ]
        ),
        
        # Nodo de conversión de botones a comandos del gripper
        Node(
            package='griper_control',
            executable='button_to_gripper_node',
            name='button_to_gripper',
            output='screen',
            parameters=[
                {'closed_position': 52},      # 0 = completamente cerrado
                {'open_position': 100},      # 250 = casi completamente abierto
                {'close_force': 255},        # Fuerza máxima para cerrar
                {'open_force': 150},         # Fuerza media para abrir
                {'button_topic': '/phantom/button'}
            ]
        )
    ])