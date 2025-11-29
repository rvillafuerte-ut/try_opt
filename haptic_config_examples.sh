#!/bin/bash
# Configuración de Mapeo Háptico - Ejemplos

# ============================================
# CONFIGURACIÓN POR DEFECTO (recomendada)
# ============================================
# Z y Roll invertidos, el resto normal
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py

# ============================================
# INVERTIR EJES INDIVIDUALES
# ============================================

# Invertir solo X
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_x:=-1.0

# Invertir solo Y
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_y:=-1.0

# Mantener Z normal (no invertido)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_z:=1.0

# Mantener Roll normal (no invertido)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_roll:=1.0

# Invertir Pitch
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_pitch:=-1.0

# Invertir Yaw
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_yaw:=-1.0

# ============================================
# COMBINACIONES COMUNES
# ============================================

# Todo normal (sin inversiones)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  sign_x:=1.0 sign_y:=1.0 sign_z:=1.0 \
  sign_roll:=1.0 sign_pitch:=1.0 sign_yaw:=1.0

# Todo invertido
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  sign_x:=-1.0 sign_y:=-1.0 sign_z:=-1.0 \
  sign_roll:=-1.0 sign_pitch:=-1.0 sign_yaw:=-1.0

# Solo posición invertida (mantener orientación normal)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  sign_x:=-1.0 sign_y:=-1.0 sign_z:=-1.0 \
  sign_roll:=1.0 sign_pitch:=1.0 sign_yaw:=1.0

# ============================================
# CON ESCALADO ADICIONAL
# ============================================

# Movimiento más lento (50%) + Z y Roll invertidos
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_pos:=0.5 haptic_scale_rot:=0.5

# Movimiento más rápido (150%) con configuración específica
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_pos:=1.5 \
  sign_x:=1.0 sign_y:=-1.0 sign_z:=-1.0

# Solo seguir posición (desactivar orientación)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  haptic_scale_rot:=0.0

# ============================================
# MAPEO PERSONALIZADO
# ============================================
# Ejemplo: Si descubres que necesitas invertir Y y Pitch
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
  sign_y:=-1.0 sign_pitch:=-1.0
