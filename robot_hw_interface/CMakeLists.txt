cmake_minimum_required(VERSION 3.5)
project(robot_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_info REQUIRED)
find_package(process_commu REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_fun REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_hw_interface.cpp
)
target_include_directories(${PROJECT_NAME} 
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
  ${process_commu_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} 
  ${robot_info_LIBRARIES}
  ${process_commu_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  robot_info
  process_commu
)

add_library(controller_configure SHARED
  src/controller_configure.cpp
)
target_include_directories(controller_configure
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
  ${process_commu_INCLUDE_DIRS}
  ${robot_fun_INCLUDE_DIRS}
)
target_link_libraries(controller_configure
  ${robot_info_LIBRARIES}
  ${process_commu_LIBRARIES}
  ${robot_fun_LIBRARIES}
)
ament_target_dependencies(controller_configure
  rclcpp
  controller_manager_msgs
  robot_info
  process_commu
  std_msgs
  robot_fun
)

add_executable(start_controller
  src/start_controller.cpp
)
target_include_directories(start_controller
  PRIVATE
  include
)
target_link_libraries(start_controller
  controller_configure
)
ament_target_dependencies(start_controller
  rclcpp
)

add_executable(test_switch_controller
  test/test_switch_controller.cpp
)
target_include_directories(test_switch_controller
  PRIVATE
  include
)
target_link_libraries(test_switch_controller
  controller_configure
)
ament_target_dependencies(test_switch_controller
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface robot_hw_interface.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  ${PROJECT_NAME}
  controller_configure
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  start_controller
  test_switch_controller
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
  controller_configure
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  robot_info
  process_commu
  controller_manager_msgs
  std_msgs
  robot_fun
)

ament_package()
