#!/bin/bash

# Haptic Teleoperation - Axis Remapping Examples
# ===============================================
# This script shows how to use axis remapping and sign control together.
# 
# AXIS SIGN CONTROL (multiply by -1):
#   sign_x, sign_y, sign_z: Position axis signs (default: 1.0, 1.0, -1.0)
#   sign_roll, sign_pitch, sign_yaw: Orientation axis signs (default: -1.0, 1.0, 1.0)
#
# AXIS REMAPPING (swap axes):
#   map_x, map_y, map_z: Which phantom axis (0=X, 1=Y, 2=Z) maps to robot X/Y/Z
#   map_roll, map_pitch, map_yaw: Which phantom axis (0=Roll, 1=Pitch, 2=Yaw) maps to robot Roll/Pitch/Yaw
#
# Example: Robot X = -Phantom Y would use: map_x:=1 sign_x:=-1.0

# ===== POSITION AXIS REMAPPING EXAMPLES =====

# Swap X and Y axes (Robot X←Phantom Y, Robot Y←Phantom X)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=1 map_y:=0

# Swap X and Y with inversion (Robot X = -Phantom Y, Robot Y = Phantom X)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=1 map_y:=0 sign_x:=-1.0

# Robot X uses Phantom Z (inverted)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=2 sign_x:=-1.0

# Robot Y uses Phantom Z (normal)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_y:=2

# Cyclic rotation: Robot X←Y, Y←Z, Z←X
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=1 map_y:=2 map_z:=0

# Robot: X=-Y, Y=-X, Z=-Z
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 sign_x:=-1.0 \
    map_y:=0 sign_y:=-1.0 \
    sign_z:=-1.0

# ===== ORIENTATION AXIS REMAPPING EXAMPLES =====

# Swap Roll and Pitch (Robot Roll←Phantom Pitch, Robot Pitch←Phantom Roll)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_roll:=1 map_pitch:=0

# Robot Roll uses Phantom Yaw (inverted)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_roll:=2 sign_roll:=-1.0

# Robot Pitch uses Phantom Yaw
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_pitch:=2

# Cyclic: Robot Roll←Pitch, Pitch←Yaw, Yaw←Roll
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_roll:=1 map_pitch:=2 map_yaw:=0

# Robot: Roll=Pitch, Pitch=-Roll, Yaw=Yaw
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_roll:=1 \
    map_pitch:=0 sign_pitch:=-1.0

# ===== COMBINED POSITION + ORIENTATION REMAPPING =====

# Complete swap: X↔Y and Roll↔Pitch
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 map_y:=0 \
    map_roll:=1 map_pitch:=0

# Complex example: Robot X=-Y, Y=-X, Z=Z, Roll=Pitch, Pitch=-Roll, Yaw=Yaw
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 sign_x:=-1.0 \
    map_y:=0 sign_y:=-1.0 \
    map_roll:=1 \
    map_pitch:=0 sign_pitch:=-1.0

# All axes cycled by one: X←Y, Y←Z, Z←X, Roll←Pitch, Pitch←Yaw, Yaw←Roll
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 map_y:=2 map_z:=0 \
    map_roll:=1 map_pitch:=2 map_yaw:=0

# ===== MIRROR/REFLECTION CONFIGURATIONS =====

# Mirror X axis (flip sign)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_x:=-1.0

# Mirror Y-Z plane (flip X, keep Y and Z)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_x:=-1.0 sign_z:=1.0

# 180° rotation around Z (flip X and Y)
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py sign_x:=-1.0 sign_y:=-1.0 sign_z:=1.0

# ===== WITH SCALING =====

# Swap X and Y with half-speed
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 map_y:=0 \
    haptic_scale_pos:=0.5

# Full custom: remapping + signs + scaling
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 sign_x:=-1.0 \
    map_roll:=1 \
    haptic_scale_pos:=0.75 \
    haptic_scale_rot:=1.5
