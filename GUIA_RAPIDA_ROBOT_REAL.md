# 🤖 Guía Rápida - Robot Real UR5e con Haptic

## 🚀 Comandos de Inicio

### 1. Lanzar el Robot Real + Phantom
```bash
# Terminal 1: Driver del robot real
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.10.103 \
  kinematics_params_file:="${HOME}/my_robot_calibration.yaml" \
  initial_joint_controller:=scaled_joint_trajectory_controller

# Terminal 2: Driver del Phantom Omni
ros2 run omni_common omni_state \
  --ros-args \
  -p omni_name:=phantom \
  -p publish_rate:=1000 \
  -p reference_frame:=world \
  -p units:=mm

# Terminal 3: Teleoperation (después de que el robot esté conectado)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py
```

### 2. Verificar Conexión del Robot
```bash
# Ver tópicos disponibles
ros2 topic list | grep joint

# Debe aparecer:
# /joint_states
# /scaled_joint_trajectory_controller/joint_trajectory
# /scaled_joint_trajectory_controller/state

# Ver estado actual del robot
ros2 topic echo /joint_states --once
```

### 3. Verificar Phantom
```bash
# Ver pose del phantom
ros2 topic echo /phantom/pose
```

---

## ⚙️ Configuración de Parámetros

### Escala de Movimiento
```bash
# Movimiento MÁS LENTO (más seguro para pruebas)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_pos:=1.0

# Movimiento NORMAL (por defecto)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_pos:=2.5

# Movimiento MÁS RÁPIDO
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_pos:=4.0
```

### Filtro de Suavizado
```bash
# Más suave (menos reactivo, más estable)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py filter_gain:=0.3

# Normal (por defecto)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py filter_gain:=0.6

# Más reactivo (puede ser nervioso)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py filter_gain:=0.9
```

### Límite de Velocidad Articular
```bash
# MÁS LENTO (más seguro)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py max_joint_vel:=1.5

# Normal (por defecto)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py max_joint_vel:=2.5

# Más rápido
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py max_joint_vel:=3.5
```

---

## 🔧 Corrección de Orientación del Robot

Si el robot está orientado diferente (rotado respecto al phantom):

### Robot Girado 180° (invertir X e Y)
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  sign_x:=-1.0 \
  sign_y:=-1.0
```

### Robot Girado 90° a la Derecha (intercambiar X ↔ Y)
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  map_x:=1 \
  map_y:=0
```

### Robot Girado 90° a la Izquierda
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  map_x:=1 \
  map_y:=0 \
  sign_x:=-1.0 \
  sign_y:=-1.0
```

### Invertir Solo el Eje Z (arriba/abajo)
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py sign_z:=-1.0
```

---

## 🛠️ Solución de Problemas

### ❌ Robot No Se Mueve

1. **Verificar que el robot esté conectado y en modo Remote Control**
   ```bash
   ros2 topic hz /joint_states
   # Debe mostrar ~125 Hz
   ```

2. **Verificar que el controlador esté activo**
   ```bash
   ros2 control list_controllers
   # scaled_joint_trajectory_controller debe estar [active]
   ```

3. **Reiniciar el nodo de teleoperation**
   ```bash
   # Ctrl+C en Terminal 3, luego:
   ros2 launch ur5_scaled_sender opt_teleop_real.launch.py
   ```

### ⚠️ Robot Se Mueve Muy Lento

```bash
# Aumentar escala de posición
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_pos:=4.0

# Aumentar límite de velocidad
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  haptic_scale_pos:=3.5 \
  max_joint_vel:=3.5
```

### ⚠️ Robot Se Mueve Muy Rápido / Errático

```bash
# Reducir escala y aumentar suavizado
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  haptic_scale_pos:=1.5 \
  filter_gain:=0.4 \
  max_joint_vel:=2.0
```

### ⚠️ Error: "Velocity saturation" o "22 rad/s"

**SOLUCIÓN APLICADA:** El código ahora tiene control adaptativo de `time_from_start` que ajusta automáticamente la velocidad según el error. Si persiste:

```bash
# Reducir límite de velocidad articular
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py max_joint_vel:=1.5

# Y/o reducir frecuencia de control
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py ctrl_hz:=80.0
```

### ❌ Direcciones Invertidas (derecha es izquierda, etc.)

**Tabla de Corrección Rápida:**

| Problema | Solución |
|----------|----------|
| Derecha → Izquierda | `sign_x:=-1.0` |
| Adelante → Atrás | `sign_y:=-1.0` |
| Arriba → Abajo | `sign_z:=-1.0` |
| Todo invertido | `sign_x:=-1.0 sign_y:=-1.0 sign_z:=-1.0` |

### 🔄 Robot Rota Extraño

Si la rotación del robot no coincide con la del phantom:

```bash
# Invertir roll
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py sign_roll:=-1.0

# Invertir pitch
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py sign_pitch:=-1.0

# Invertir yaw
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py sign_yaw:=-1.0

# Reducir escala de rotación (menos sensible)
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py haptic_scale_rot:=0.5
```

---

## 📊 Monitoreo en Tiempo Real

### Ver Comandos Enviados al Robot
```bash
ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory
```

### Ver Estado del Robot
```bash
ros2 topic echo /joint_states
```

### Ver Datos del CSV (después de correr)
```bash
# Archivo generado en:
cat /home/utec/try_opt/teleop_data.csv

# Graficar en Python:
python3 /home/utec/try_opt/plot_teleop.py
```

---

## 🎯 Configuración Recomendada para Inicio

**Primera prueba (MUY SEGURO):**
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  haptic_scale_pos:=1.0 \
  filter_gain:=0.4 \
  max_joint_vel:=1.5
```

**Después de verificar que funciona (NORMAL):**
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  haptic_scale_pos:=2.5 \
  filter_gain:=0.6 \
  max_joint_vel:=2.5
```

**Para movimientos rápidos (AVANZADO):**
```bash
ros2 launch ur5_scaled_sender opt_teleop_real.launch.py \
  haptic_scale_pos:=3.5 \
  filter_gain:=0.7 \
  max_joint_vel:=3.5
```

---

## 🔍 Cambios Importantes Aplicados

### ✅ Control Adaptativo de Velocidad
- El `time_from_start` ahora se ajusta dinámicamente (0.02s - 0.15s)
- Si el error es grande, el robot se mueve más lento automáticamente
- **SOLUCIONA:** Errores de "22 rad/s" y velocidad saturada

### ✅ Corrección de Orientación
- Cambio de `R_start * R_delta` a `R_delta * R_start`
- **SOLUCIONA:** Rotaciones extrañas del efector final

### ✅ Mejora en Límites de Velocidad
- Escalado por componente en lugar de norm vectorial
- Preserva la dirección del movimiento
- **SOLUCIONA:** Movimientos lentos o trabados

---

## 📞 Checklist Rápido Pre-Prueba

- [ ] Robot conectado a 192.168.10.103
- [ ] Robot en modo **Remote Control** (teach pendant)
- [ ] Phantom conectado y publicando `/phantom/pose`
- [ ] Driver del robot ejecutándose sin errores
- [ ] `scaled_joint_trajectory_controller` activo
- [ ] Workspace libre de obstáculos
- [ ] Botón de parada de emergencia accesible

---

## 🚨 En Caso de Emergencia

1. **Presionar STOP en el teach pendant**
2. `Ctrl+C` en todos los terminales
3. Si el robot no responde, usar parada de emergencia física

**Comando de parada suave:**
```bash
ros2 topic pub --once /scaled_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{}"
```
