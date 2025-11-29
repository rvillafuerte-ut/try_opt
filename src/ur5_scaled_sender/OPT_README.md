# opt_teleop

Controlador óptimo (QP-LDLT) para UR5e calibrado que sigue trayectorias cartesianas lineales leyendo `/joint_states` y publicando al `scaled_joint_trajectory_controller`.

## Algoritmo
- **FK + Jacobiano**: Cinemática directa con calibración del YAML + Jacobiano geométrico 6×6
- **Error 6D**: Posición (x,y,z) + Orientación (error de rotación ángulo-eje)
- **QP**: `min 0.5*dq'*H*dq + g'*dq` con pesos [10,10,10,1,1,1] + regularización 0.01
- **Solver**: `H.ldlt().solve(-g)` (más rápido que CasADi qpoases según `comparation.cpp`)
- **Iteraciones**: 5 por ciclo, convergencia < 1e-4

## Compilación
```bash
source /opt/ros/humble/setup.bash
cd /home/utec/try_opt
colcon build --symlink-install --packages-select ur5_scaled_sender
source install/setup.bash
```

## Ejecución
Driver UR5 + controlador activo requeridos.

```bash
ros2 launch ur5_scaled_sender opt_teleop.launch.py \
  x_start:=0.3 z_start:=0.4 \
  x_end:=0.5 z_end:=0.6 \
  traj_duration:=5.0 \
  ctrl_hz:=100.0
```

**Parámetros**:
- `calib_yaml`: Ruta al YAML de calibración (default: `/home/utec/try_opt/config/my_robot_calibration_ur5e.yaml`)
- `x_start`, `y_start`, `z_start`: Posición inicial [m]
- `x_end`, `y_end`, `z_end`: Posición final [m]
- `roll_des`, `pitch_des`, `yaw_des`: Orientación deseada [rad]
- `traj_duration`: Duración [s]
- `ctrl_hz`: Frecuencia de control [Hz]
- `controller_name`: Nombre del controlador

## Tópicos
- **Sub**: `/joint_states` (sensor_msgs/JointState)
- **Pub**: `/scaled_joint_trajectory_controller/joint_trajectory` (trajectory_msgs/JointTrajectory)

## Dependencias
- ROS 2 Humble
- Eigen3
- yaml-cpp (`sudo apt install libyaml-cpp-dev`)
- UR driver con `scaled_joint_trajectory_controller`
