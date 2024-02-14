cmake_minimum_required(VERSION 3.5)
project(vox_nav_msgs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Object.msg"
  "msg/ObjectArray.msg"
  "msg/OrientedNavSatFix.msg"
  "msg/SemanticRoadLabel.msg"
  "srv/GetOctomap.srv"
  "srv/GetOSMRoadTopologyMap.srv"
  "srv/GetPointCloud.srv"
  "srv/GetTraversabilityMap.srv"
  "action/ComputePathToPose.action"
  "action/FollowPath.action"
  "action/NavigateToPose.action"
  "action/NavigateThroughGPSPoses.action"
  "action/NavigateThroughPoses.action"
  DEPENDENCIES 
  builtin_interfaces 
  geometry_msgs 
  std_msgs 
  nav_msgs
  sensor_msgs
  shape_msgs
  octomap_msgs
  rosidl_default_generators
)

ament_auto_package()
