# Teleoperación Háptica - Guía de Configuración

## Funcionamiento

El sistema transforma automáticamente los movimientos del dispositivo háptico (phantom) al marco de referencia del robot UR5, independientemente de sus orientaciones iniciales.

### Transformación de Coordenadas

**Posición:**
```
offset_phantom = phantom_actual - phantom_inicial  (en marco phantom)
offset_robot = R_start * offset_phantom * scale_pos  (transformado a marco robot)
p_deseada = p_robot_inicial + offset_robot
```

**Orientación:**
```
quat_offset = quat_phantom_actual * quat_phantom_inicial⁻¹
quat_offset_escalado = axis_angle(quat_offset * scale_rot)
quat_deseada = quat_robot_inicial * quat_offset_escalado
```

## Parámetros de Calibración

### `haptic_scale_pos` (Escala de Posición)
- **Default**: `1.0` (mapeo 1:1)
- **Uso**: Ajusta la sensibilidad del movimiento
- **Ejemplos**:
  - `0.5` → Robot se mueve la mitad que el phantom
  - `2.0` → Robot se mueve el doble que el phantom
  - `-1.0` → Invierte todos los ejes (útil si el phantom está al revés)

### `haptic_scale_rot` (Escala de Rotación)
- **Default**: `1.0` (mapeo 1:1)
- **Uso**: Ajusta la sensibilidad de la rotación
- **Ejemplos**:
  - `0.5` → Robot rota la mitad que el phantom
  - `0.0` → Desactiva seguimiento de orientación (solo posición)

## Ejemplos de Uso

### Configuración Estándar (1:1)
```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py
```

### Movimiento Más Lento (50% de velocidad)
```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_pos:=0.5 \
  haptic_scale_rot:=0.5
```

### Movimiento Más Rápido (200% de velocidad)
```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_pos:=2.0 \
  haptic_scale_rot:=2.0
```

### Solo Seguimiento de Posición (sin orientación)
```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_rot:=0.0
```

## Solución de Problemas

### Problema: Ejes X/Y invertidos
**Causa**: Los marcos de referencia del phantom y robot no están alineados.

**Solución**: La transformación `R_start * offset_phantom` automáticamente maneja esto. El sistema usa la orientación inicial del robot para transformar correctamente los offsets.

**Verificación**:
- Observa los logs: `Phantom:[x,y,z] -> Robot:[x',y',z']`
- El offset en marco phantom se transforma al marco robot automáticamente

### Problema: Orientación incorrecta
**Causa**: El quaternion inicial del phantom define la orientación de referencia.

**Solución**: 
1. El sistema captura automáticamente la pose inicial cuando inicia
2. El offset rotacional se aplica respecto a esa referencia
3. Ajusta `haptic_scale_rot` si necesitas invertir o escalar la rotación

### Problema: Robot se mueve demasiado rápido/lento
**Solución**: Ajusta `haptic_scale_pos` entre 0.1 y 2.0

## Monitoreo

### Ver transformaciones en tiempo real
```bash
ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory
```

### Ver logs del sistema
Los logs muestran:
- Offset en marco phantom: `[x_p, y_p, z_p]`
- Offset transformado a marco robot: `[x_r, y_r, z_r]`
- Pose objetivo: `[x_des, y_des, z_des]`
- Errores: posición (m) y orientación (rad)

### Datos guardados
El sistema guarda datos en `teleop_haptic_data.csv` con:
- Offsets del phantom en su marco de referencia
- Posiciones comandadas y actuales del robot
- Errores de seguimiento

## Notas Técnicas

1. **Independencia de poses iniciales**: El sistema automáticamente transforma entre marcos, por lo que puedes iniciar el robot y phantom en cualquier pose.

2. **Singularidades**: Si el robot se acerca a una singularidad, los movimientos pueden volverse erráticos. Mantén el phantom dentro de rangos razonables.

3. **Límites de joints**: El optimizador respeta los límites [-π, π] de cada joint. Movimientos que excedan estos límites serán limitados.

4. **Frecuencia de control**: 100Hz con lookahead de 20ms para seguimiento suave.
