cmake_minimum_required(VERSION 3.8)
project(ur5_scaled_sender)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(scaled_traj_publisher src/joint_trajectory_publisher.cpp)
ament_target_dependencies(scaled_traj_publisher rclcpp trajectory_msgs)
target_include_directories(scaled_traj_publisher PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(scaled_traj_publisher Eigen3::Eigen)


add_executable(opt_teleop_haptic src/opt_teleop_haptic.cpp)
ament_target_dependencies(opt_teleop_haptic rclcpp trajectory_msgs sensor_msgs geometry_msgs)
target_include_directories(opt_teleop_haptic PRIVATE ${EIGEN3_INCLUDE_DIRS} ${pinocchio_INCLUDE_DIRS})
target_link_libraries(opt_teleop_haptic Eigen3::Eigen ${pinocchio_LIBRARIES})

install(TARGETS scaled_traj_publisher opt_teleop_haptic
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
