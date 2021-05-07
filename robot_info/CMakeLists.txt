cmake_minimum_required(VERSION 3.5)
project(robot_info)

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

if(END_EFFECTOR)
add_library(${PROJECT_NAME} SHARED
  src/arm_info.cpp
  src/end_eff_info.cpp
  src/robot_info.cpp
)
endif()
if(NOT END_EFFECTOR)
add_library(${PROJECT_NAME} SHARED
  src/arm_info.cpp
  src/robot_info.cpp
)
endif()
target_include_directories(${PROJECT_NAME} 
  PRIVATE 
  include
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

add_executable(test_robot_info
  test/test_robot_info.cpp
)
target_include_directories(test_robot_info
  PRIVATE
  include
)
target_link_libraries(test_robot_info
  ${PROJECT_NAME}
)
ament_target_dependencies(test_robot_info
  rclcpp
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
  test_robot_info
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
)

ament_package()
