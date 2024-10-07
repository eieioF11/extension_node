cmake_minimum_required(VERSION 3.8)
project(extension_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(data_logger REQUIRED)
# create library
add_library(${PROJECT_NAME} INTERFACE)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp
  std_msgs
  rclcpp_components
  data_logger
)

# install library
install(
  DIRECTORY   include/
  DESTINATION include
)

install(
  TARGETS   ${PROJECT_NAME}
  EXPORT    export_${PROJECT_NAME}
  LIBRARY   DESTINATION lib
  ARCHIVE   DESTINATION lib
  RUNTIME   DESTINATION bin
  INCLUDES  DESTINATION include
)

# export ament library
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  rclcpp_components
  data_logger
)

ament_package()