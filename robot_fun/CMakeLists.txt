cmake_minimum_required(VERSION 3.5)
project(robot_fun)

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

if(END_EFFECTOR)
  add_compile_options(-DUSE_END_EFFECTOR)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_info REQUIRED)
find_package(process_commu REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/robot_fun.cpp
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
  sensor_msgs
  robot_info
  process_commu
)

add_executable(trigger_state_sub
  src/trigger_state_sub.cpp
)
target_include_directories(trigger_state_sub
  PRIVATE
  include
)
target_link_libraries(trigger_state_sub
  ${PROJECT_NAME}
)
ament_target_dependencies(trigger_state_sub
  rclcpp
)

add_executable(test_trigger
  test/test_trigger.cpp
)
target_include_directories(test_trigger
  PRIVATE
  include
  ${robot_info_INCLUDE_DIRS}
)
target_link_libraries(test_trigger
  ${PROJECT_NAME}
)
ament_target_dependencies(test_trigger
  rclcpp
  robot_info
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS 
  ${PROJECT_NAME}
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS
  trigger_state_sub
  test_trigger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  rclcpp
  sensor_msgs
  robot_info
  process_commu
)

ament_package()
