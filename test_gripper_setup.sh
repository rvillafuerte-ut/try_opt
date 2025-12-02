#!/bin/bash
# Script de prueba para verificar que el gripper funciona correctamente

echo "=========================================="
echo "  Test del Control del Gripper Robotiq"
echo "=========================================="
echo ""

# Colores para output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Sourcer workspace
echo -e "${YELLOW}[1/5] Sourcing workspace...${NC}"
source /home/utec/try_opt/install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

# Verificar puerto USB
echo -e "${YELLOW}[2/5] Verificando puerto USB...${NC}"
if [ -e "/dev/ttyUSB0" ]; then
    echo -e "${GREEN}✓ Puerto /dev/ttyUSB0 encontrado${NC}"
    ls -l /dev/ttyUSB0
    
    # Intentar dar permisos
    echo "Configurando permisos..."
    sudo chmod 666 /dev/ttyUSB0
    echo -e "${GREEN}✓ Permisos configurados${NC}"
else
    echo -e "${RED}✗ Puerto /dev/ttyUSB0 no encontrado${NC}"
    echo "Puertos USB disponibles:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "Ningún puerto ttyUSB encontrado"
    echo ""
    echo "Opciones:"
    echo "1. Conecta el gripper al puerto USB"
    echo "2. Verifica con: dmesg | grep tty"
    echo "3. Si está en otro puerto, edita el launch file"
fi
echo ""

# Verificar mensaje personalizado
echo -e "${YELLOW}[3/5] Verificando mensaje GripperCommand...${NC}"
if ros2 interface show griper_control/msg/GripperCommand &> /dev/null; then
    echo -e "${GREEN}✓ Mensaje GripperCommand disponible${NC}"
    ros2 interface show griper_control/msg/GripperCommand
else
    echo -e "${RED}✗ Mensaje GripperCommand no encontrado${NC}"
    echo "Ejecuta: colcon build --packages-select griper_control"
    exit 1
fi
echo ""

# Verificar ejecutables
echo -e "${YELLOW}[4/5] Verificando ejecutables...${NC}"
if [ -f "/home/utec/try_opt/install/griper_control/lib/griper_control/gripper_controller_node" ]; then
    echo -e "${GREEN}✓ gripper_controller_node instalado${NC}"
else
    echo -e "${RED}✗ gripper_controller_node no encontrado${NC}"
fi

if [ -f "/home/utec/try_opt/install/griper_control/lib/griper_control/button_to_gripper_node" ]; then
    echo -e "${GREEN}✓ button_to_gripper_node instalado${NC}"
else
    echo -e "${RED}✗ button_to_gripper_node no encontrado${NC}"
fi
echo ""

# Resumen
echo -e "${YELLOW}[5/5] Resumen${NC}"
echo "=========================================="
echo ""
echo "Para lanzar el gripper con botones Phantom:"
echo -e "  ${GREEN}ros2 launch griper_control gripper_modular.launch.py${NC}"
echo ""
echo "Para probar comandos manualmente:"
echo -e "  ${GREEN}# Cerrar gripper${NC}"
echo '  ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 0, force: 255}"'
echo ""
echo -e "  ${GREEN}# Abrir gripper${NC}"
echo '  ros2 topic pub --once /gripper/command griper_control/msg/GripperCommand "{position: 250, force: 100}"'
echo ""
echo "Para integrar con teleoperación:"
echo "  Terminal 1: ros2 launch ur5_scaled_sender real_robot.launch.py"
echo "  Terminal 2: ros2 run ur5_scaled_sender opt_teleop_haptic"
echo "  Terminal 3: ros2 launch griper_control gripper_modular.launch.py"
echo ""
echo "=========================================="
