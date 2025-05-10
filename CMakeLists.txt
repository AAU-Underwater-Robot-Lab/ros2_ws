cmake_minimum_required(VERSION 3.5)
project(oculus_sonar_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(apl_msgs REQUIRED)
find_package(liboculus REQUIRED)

include_directories(
  include
)

add_library(driver_node SHARED
  src/driver_node.cpp
)
target_compile_features(driver_node PRIVATE c_std_11 cxx_std_17)
target_link_libraries(driver_node
  liboculus::oculus
)
target_include_directories(driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(driver_node
  rclcpp
  std_msgs
  marine_acoustic_msgs
  apl_msgs
)

rclcpp_components_register_node(driver_node
  PLUGIN "oculus_sonar_driver::DriverNode"
  EXECUTABLE driver_node_exe
)

add_executable(driver_node_exe src/main.cpp)
ament_target_dependencies(driver_node_exe rclcpp)
target_link_libraries(driver_node_exe driver_node)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS driver_node driver_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
