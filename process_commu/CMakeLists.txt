cmake_minimum_required(VERSION 3.5)
project(process_commu)

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
find_package(robot_info REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/shm_common.cpp
  src/sem_common.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE 
  include
  ${robot_info_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robot_info
)

add_executable(test_sem_common
  test/test_sem_common.cpp
)
target_include_directories(test_sem_common 
  PRIVATE 
  include
)
target_link_libraries(test_sem_common 
  ${PROJECT_NAME}
)
ament_target_dependencies(test_sem_common
  rclcpp
)

add_executable(test_shm_common
  test/test_shm_common.cpp
)
target_include_directories(test_shm_common 
  PRIVATE 
  include
)
target_link_libraries(test_shm_common 
  ${PROJECT_NAME}
)
ament_target_dependencies(test_shm_common
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
  test_sem_common
  test_shm_common
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
  robot_info
)

ament_package()
