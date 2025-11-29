# Axis Remapping System - Haptic Teleoperation

## Overview

The haptic teleoperation system now supports **full axis remapping** in addition to sign inversion. This allows you to:

1. **Invert axes** (multiply by -1) using `sign_*` parameters
2. **Swap/permute axes** using `map_*` parameters

This is useful when the phantom device coordinate frame doesn't match the robot's frame orientation.

## Parameter Reference

### Sign Control Parameters (Axis Inversion)

Apply a sign flip (×-1) to individual axes:

```bash
sign_x         # X position sign (default: 1.0)
sign_y         # Y position sign (default: 1.0)
sign_z         # Z position sign (default: -1.0)
sign_roll      # Roll orientation sign (default: -1.0)
sign_pitch     # Pitch orientation sign (default: 1.0)
sign_yaw       # Yaw orientation sign (default: 1.0)
```

### Axis Remapping Parameters

Select which phantom axis to use for each robot axis:

```bash
# Position axes (0=X, 1=Y, 2=Z)
map_x          # Which phantom axis maps to robot X (default: 0 = phantom X)
map_y          # Which phantom axis maps to robot Y (default: 1 = phantom Y)
map_z          # Which phantom axis maps to robot Z (default: 2 = phantom Z)

# Orientation axes (0=Roll, 1=Pitch, 2=Yaw)
map_roll       # Which phantom axis maps to robot Roll (default: 0 = phantom Roll)
map_pitch      # Which phantom axis maps to robot Pitch (default: 1 = phantom Pitch)
map_yaw        # Which phantom axis maps to robot Yaw (default: 2 = phantom Yaw)
```

## How It Works

The transformation pipeline:

```
1. Read phantom offset (position and orientation)
2. Apply remapping: select which phantom axis to use for each robot axis
3. Apply signs: multiply by ±1 for each axis
4. Apply scaling: multiply by haptic_scale_pos or haptic_scale_rot
5. Transform to robot frame: R_start * offset
6. Run inverse kinematics
```

**Formula for each axis:**
```
robot_axis_i = sign_i * phantom_axis_map[i] * scale
```

Example: If `map_x=1` and `sign_x=-1.0`, then:
```
robot_X = -phantom_Y * scale
```

## Common Use Cases

### 1. Swap X and Y Axes

Phantom moving in Y → Robot moves in X  
Phantom moving in X → Robot moves in Y

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=1 map_y:=0
```

### 2. Swap X and Y with Inversion

Robot X = -Phantom Y  
Robot Y = Phantom X

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 sign_x:=-1.0 \
    map_y:=0
```

### 3. Use Z for X Movement (Inverted)

Robot X = -Phantom Z

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=2 sign_x:=-1.0
```

### 4. Cyclic Rotation of Axes

X←Y, Y←Z, Z←X

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_x:=1 map_y:=2 map_z:=0
```

### 5. Swap Roll and Pitch

Robot Roll = Phantom Pitch  
Robot Pitch = Phantom Roll

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py map_roll:=1 map_pitch:=0
```

### 6. Complex Remapping

Robot X = -Phantom Y  
Robot Y = -Phantom X  
Robot Roll = Phantom Pitch  
Robot Pitch = -Phantom Roll

```bash
ros2 launch ur5_scaled_sender opt_teleop_haptic.launch.py \
    map_x:=1 sign_x:=-1.0 \
    map_y:=0 sign_y:=-1.0 \
    map_roll:=1 \
    map_pitch:=0 sign_pitch:=-1.0
```

## Quick Reference Table

### Position Axis Values

| Value | Maps to Phantom Axis |
|-------|---------------------|
| 0     | X                   |
| 1     | Y                   |
| 2     | Z                   |

### Orientation Axis Values

| Value | Maps to Phantom Axis |
|-------|---------------------|
| 0     | Roll                |
| 1     | Pitch               |
| 2     | Yaw                 |

## Examples Script

See `haptic_remap_examples.sh` for many example configurations covering:

- Position axis swapping (X↔Y, X←Z, etc.)
- Orientation axis swapping (Roll↔Pitch, etc.)
- Combined position + orientation remapping
- Mirror/reflection configurations
- Remapping with scaling

## Debugging

The system logs the active mapping on startup:

```
[INFO] Position mapping: Robot[X,Y,Z] <- Phantom[Y, X, Z]
[INFO] Position signs: x=-1.0, y=1.0, z=-1.0 (scale=1.00)
[INFO] Orientation mapping: Robot[R,P,Y] <- Phantom[Pitch, Roll, Yaw]
[INFO] Orientation signs: roll=1.0, pitch=-1.0, yaw=1.0 (scale=1.00)
```

This clearly shows which phantom axis is used for each robot axis and what sign is applied.

## Mathematical Details

### Position Remapping

Given phantom offset `p = [px, py, pz]` and mapping parameters:

```cpp
robot_offset_remapped[0] = p[map_x] * sign_x * scale_pos  // Robot X
robot_offset_remapped[1] = p[map_y] * sign_y * scale_pos  // Robot Y
robot_offset_remapped[2] = p[map_z] * sign_z * scale_pos  // Robot Z
```

Then transformed to robot frame:
```cpp
robot_offset_final = R_start * robot_offset_remapped
```

### Orientation Remapping

Given phantom RPY offset `φ = [roll, pitch, yaw]`:

```cpp
robot_rpy[0] = φ[map_roll] * sign_roll * scale_rot    // Robot Roll
robot_rpy[1] = φ[map_pitch] * sign_pitch * scale_rot  // Robot Pitch
robot_rpy[2] = φ[map_yaw] * sign_yaw * scale_rot      // Robot Yaw
```

Then converted back to rotation matrix and applied.

## Tips

1. **Start with defaults**: Test without remapping first, use only sign inversions if possible
2. **One axis at a time**: Change one mapping parameter, test, then add more
3. **Check logs**: The startup logs show exactly what mapping is active
4. **Use examples**: Copy configurations from `haptic_remap_examples.sh`
5. **Combine carefully**: Remapping + signs + scaling can create complex behaviors

## Related Files

- `opt_teleop_haptic.cpp`: Implementation
- `opt_teleop_haptic.launch.py`: Launch file with all parameters
- `haptic_remap_examples.sh`: Example configurations
- `HAPTIC_TELEOP_README.md`: General haptic teleop documentation
