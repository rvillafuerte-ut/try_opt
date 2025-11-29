# ur5_scaled_sender

Publicador simple (C++) para enviar `trajectory_msgs/JointTrajectory` al controlador `scaled_joint_trajectory_controller` de UR5 en ROS 2 Humble. Incluye uso de Eigen3 para operaciones básicas con vectores.

## Requisitos
- ROS 2 Humble (con `colcon` y `ament_cmake`)
- `universal_robot` driver activo con el controlador `scaled_joint_trajectory_controller` cargado
- Eigen3 instalado (en Ubuntu: `libeigen3-dev`). Si usas rosdep, este paquete declara `eigen3_cmake_module` + `Eigen3`.

## Compilación
```bash
# Terminal 1: preparar workspace y compilar
source /opt/ros/humble/setup.bash
cd /home/utec/try_opt
colcon build --symlink-install --packages-select ur5_scaled_sender
source install/setup.bash
```

## Ejecución
Asegúrate de tener el driver del UR5 levantado y el controlador activo (p. ej. `/scaled_joint_trajectory_controller`).

```bash
# Enviar un punto único con 2s de duración
ros2 launch ur5_scaled_sender send_trajectory.launch.py \
  controller_name:=scaled_joint_trajectory_controller \
  positions:="0.0,-1.57,1.57,0.0,0.0,0.0" \
  time_from_start_sec:=2.0
```

Parámetros opcionales:
- `repeat` (bool): si `true`, republica periódicamente.
- `publish_rate_hz` (double): frecuencia al usar `repeat=true`.

## Tópico objetivo
- `/scaled_joint_trajectory_controller/joint_trajectory` (tipo: `trajectory_msgs/msg/JointTrajectory`)

## Nombres de juntas (UR5)
Por defecto:
```
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```
Puedes sobreescribirlos pasando un archivo YAML o cambiando parámetros, aunque el launch ya cubre la mayoría de casos.
