# Configuración del Control del Gripper Robotiq

## Dependencias Instaladas ✓
- `libmodbus5` y `libmodbus-dev` (ya instalados)
- Paquete `griper_control` compilado correctamente

## Estructura del Paquete

```
griper_control/
├── msg/
│   └── GripperCommand.msg         # Mensaje personalizado (position, speed, force)
├── src/
│   ├── gripper_controller_node.cpp  # Controlador principal (Modbus RTU)
│   └── button_to_gripper_node.cpp   # Mapeo de botones Phantom → Gripper
└── launch/
    └── gripper_modular.launch.py    # Launch file principal
```

## Configuración del Puerto Serie

Antes de usar el gripper, configura los permisos del puerto USB:

```bash
# Identificar el puerto (generalmente /dev/ttyUSB0 o /dev/ttyUSB1)
ls -l /dev/ttyUSB*

# Dar permisos (el nodo lo hace automáticamente, pero puedes hacerlo manual)
sudo chmod 666 /dev/ttyUSB0

# O agregar tu usuario al grupo dialout (permanente, requiere logout)
sudo usermod -a -G dialout $USER
```

## Uso Básico

### 1. Lanzar el controlador del gripper

```bash
# Sourcer el workspace
source /home/utec/try_opt/install/setup.bash

# Lanzar el controlador completo (gripper + mapeo de botones Phantom)
ros2 launch griper_control gripper_modular.launch.py

# O lanzar solo el controlador del gripper
ros2 run griper_control gripper_controller_node
```

### 2. Enviar comandos manualmente

```bash
# Mensaje GripperCommand tiene 3 campos: position (0-255), speed (0-255), force (0-255)

# Cerrar gripper (posición 0, velocidad media, fuerza alta)
ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 0, speed: 150, force: 255}"

# Abrir gripper (posición 250, velocidad media, fuerza baja)
ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 250, speed: 150, force: 100}"

# Posición intermedia (agarre suave)
ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 120, speed: 100, force: 150}"
```

### 3. Control con botones del Phantom Omni

El nodo `button_to_gripper_node` mapea automáticamente:
- **Botón 1**: Cierra el gripper
- **Botón 2**: Abre el gripper

Se suscribe al topic `/phantom1/phantom/button` (configurable en el launch file).

## Parámetros del Launch File

```python
# gripper_modular.launch.py
parameters=[
    {'device': '/dev/ttyUSB0'},       # Puerto serie del gripper
    {'baudrate': 115200},             # Velocidad de comunicación
    {'slave_id': 9},                  # ID Modbus del gripper
    {'closed_position': 0},           # Posición cerrada (0-255)
    {'open_position': 250},           # Posición abierta (0-255)
    {'close_force': 255},             # Fuerza al cerrar (0-255)
    {'open_force': 150},              # Fuerza al abrir (0-255)
    {'button_topic': '/phantom1/phantom/button'}  # Topic de botones
]
```

## Troubleshooting

### Error: "No such file or directory /dev/ttyUSB0"
```bash
# Verificar dispositivos USB conectados
ls -l /dev/ttyUSB*
dmesg | grep tty

# Si está en otro puerto, modificar el parámetro 'device' en el launch file
```

### Error: "Permission denied"
```bash
# Dar permisos temporales
sudo chmod 666 /dev/ttyUSB0

# O agregar usuario a grupo dialout (permanente)
sudo usermod -a -G dialout $USER
# Luego hacer logout/login
```

### Error: "Modbus connection failed"
```bash
# Verificar que el gripper esté encendido y conectado
# Verificar baudrate correcto (115200 para Robotiq)
# Verificar slave_id correcto (generalmente 9)

# Probar comunicación manual
sudo apt install libmodbus-dev
```

### El gripper no responde
```bash
# Ver logs del nodo
ros2 launch griper_control gripper_modular.launch.py

# Verificar topic de comandos
ros2 topic echo /gripper/command

# Activar el gripper manualmente (el nodo lo hace al inicio)
# El comando de activación está en el código
```

## Integración con Teleoperación

Para usar el gripper junto con la teleoperación del UR5:

```bash
# Terminal 1: Lanzar robot real + haptic (sin gripper)
ros2 launch ur5_scaled_sender real_robot.launch.py

# Terminal 2: Lanzar teleoperación
ros2 run ur5_scaled_sender opt_teleop_haptic

# Terminal 3: Lanzar control del gripper
ros2 launch griper_control gripper_modular.launch.py
```

Ahora puedes teleoperar el robot con el Phantom Omni y controlar el gripper con los botones.

## Comandos Rápidos

```bash
# Compilar solo gripper
cd /home/utec/try_opt
source /opt/ros/humble/setup.bash
colcon build --packages-select griper_control --symlink-install

# Sourcer
source install/setup.bash

# Lanzar
ros2 launch griper_control gripper_modular.launch.py

# Cerrar
ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 0, speed: 150, force: 255}"

# Abrir  
ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 250, speed: 150, force: 100}"
```
