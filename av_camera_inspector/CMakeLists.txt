cmake_minimum_required(VERSION 3.8)
project(av_camera_inspector)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

# Timestamp inspector component
add_library(timestamp_inspector_component SHARED src/timestamp_inspector.cc)

ament_target_dependencies(timestamp_inspector_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  image_transport
)

rclcpp_components_register_node(
  timestamp_inspector_component
    PLUGIN "camera::TimestampInspector"
    EXECUTABLE timestamp_inspector
)


ament_export_targets(export_timestamp_inspector_component)
install(TARGETS timestamp_inspector_component
        EXPORT export_timestamp_inspector_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
