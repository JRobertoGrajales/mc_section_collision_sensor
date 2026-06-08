cmake_minimum_required(VERSION 3.14)

project(
  ADCCollisionSensor
  LANGUAGES CXX
  VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
 
# ROS 2 — needed for the voltage subscriber.
# mc_rtc's ROSBridge handles rclcpp::init(); we only need the headers + libs.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
 
# Configure and install the plugin configuration
configure_file(etc/ADCCollisionSensor.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/ADCCollisionSensor.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ADCCollisionSensor.yaml"
        DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")
 
set(plugin_SRC
  src/ADCCollisionSensor.cpp
)
 
set(plugin_HDR
  src/ADCCollisionSensor.h
  src/LpfThreshold.h
)
 
# The add_plugin macro is provided by mc_rtc
add_plugin(ADCCollisionSensor "${plugin_SRC}" "${plugin_HDR}")
 
target_include_directories(ADCCollisionSensor
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
 
ament_target_dependencies(ADCCollisionSensor rclcpp std_msgs)
 
# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()
 

