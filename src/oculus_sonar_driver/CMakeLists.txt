cmake_minimum_required(VERSION 3.8)
project(oculus_sonar_driver)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)

# Include headers
include_directories(include)
include_directories(${CMAKE_SOURCE_DIR}/liboculus)

# Build the driver library
add_library(oculus_driver SHARED
  src/oculus_driver.cpp
)

# Build additional executables
add_executable(oculus_driver_nodelet src/oculus_driver_nodelet.cpp)
add_executable(oculus_node src/oculus_node.cpp)
add_executable(reprocess_oculus_raw_data src/reprocess_oculus_raw_data.cpp)
add_executable(reprocess_oculus_raw_data_node src/reprocess_oculus_raw_data_node.cpp)

ament_target_dependencies(oculus_driver
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  marine_acoustic_msgs
)
ament_target_dependencies(oculus_driver_nodelet rclcpp rclcpp_components sensor_msgs std_msgs marine_acoustic_msgs)
ament_target_dependencies(oculus_node rclcpp rclcpp_components sensor_msgs std_msgs marine_acoustic_msgs)
ament_target_dependencies(reprocess_oculus_raw_data rclcpp rclcpp_components sensor_msgs std_msgs marine_acoustic_msgs)
ament_target_dependencies(reprocess_oculus_raw_data_node rclcpp rclcpp_components sensor_msgs std_msgs marine_acoustic_msgs)

# Register the ROS2 component
rclcpp_components_register_nodes(oculus_driver
  PLUGIN "oculus_sonar_driver::OculusDriver"
)

# Install targets
install(TARGETS oculus_driver oculus_driver_nodelet oculus_node reprocess_oculus_raw_data reprocess_oculus_raw_data_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch and config files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_package()
