cmake_minimum_required(VERSION 3.8)
project(griper_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl REQUIRED)
find_package(omni_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(/usr/include/modbus)

# Generar mensajes personalizados
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperCommand.msg"
  DEPENDENCIES std_msgs
)

set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Nuevo nodo controlador del gripper
add_executable(gripper_controller_node ${SRC_DIR}/gripper_controller_node.cpp)
ament_target_dependencies(gripper_controller_node 
        rclcpp 
        sensor_msgs
    )
target_link_libraries(gripper_controller_node modbus)

# Nuevo nodo de conversión de botones
add_executable(button_to_gripper_node ${SRC_DIR}/button_to_gripper_node.cpp)
ament_target_dependencies(button_to_gripper_node 
        rclcpp 
        omni_msgs
    )

# Vincular las interfaces generadas con los ejecutables que las usan
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(gripper_controller_node "${cpp_typesupport_target}")
target_link_libraries(button_to_gripper_node "${cpp_typesupport_target}")

install(DIRECTORY 
    src 
    launch
    DESTINATION share/${PROJECT_NAME})
    
# Instalar todos los ejecutables
install(TARGETS
    gripper_controller_node
    button_to_gripper_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
