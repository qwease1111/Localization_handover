cmake_minimum_required(VERSION 3.8)
project(rosmsgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  DEPENDENCIES std_msgs
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
  msg/GenericRanging.msg
  msg/Ranging.msg
  msg/UWBRanging.msg
  msg/PozyxRanging.msg
  msg/DWRanging.msg
  msg/RangingDiff.msg
  msg/PozyxRangingWithCir.msg
  msg/ESP32S2FTMFrame.msg
  msg/ESP32S2FTMRanging.msg
  msg/ESP32S2FTMRangingExtra.msg
  msg/RadarCube.msg
  msg/RadarRangeAzimuth.msg
  msg/RadarRangeDoppler.msg
  msg/RangingArray.msg
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()