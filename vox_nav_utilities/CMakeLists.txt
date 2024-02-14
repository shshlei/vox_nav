cmake_minimum_required(VERSION 3.5)
project(vox_nav_utilities)

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

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)

# Set common system includes
include_directories(include ${EIGEN3_INCLUDE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBFCL REQUIRED "fcl>=0.7.0")
# replace LIBFCL_LIBRARIES with full paths to the libraries
set(LIBFCL_LIBRARIES_FULL "")
foreach(LIBFCL_LIBRARY ${LIBFCL_LIBRARIES})
  find_library(${LIBFCL_LIBRARY}_LIB ${LIBFCL_LIBRARY} ${LIBFCL_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES_FULL ${${LIBFCL_LIBRARY}_LIB})
endforeach()
set(LIBFCL_LIBRARIES "${LIBFCL_LIBRARIES_FULL}")

ament_auto_add_library(tf_helpers SHARED src/tf_helpers.cpp src/pcl_helpers.cpp)
target_link_libraries(tf_helpers ${PCL_LIBRARIES})

ament_auto_add_library(planner_helpers SHARED src/planner_helpers.cpp)
target_link_libraries(planner_helpers ${LIBFCL_LIBRARIES} tf_helpers ompl)

ament_auto_add_library(gps_waypoint_collector SHARED src/gps_waypoint_collector.cpp)

ament_auto_add_library(elevation_state_space SHARED src/elevation_state_space.cpp)
target_link_libraries(elevation_state_space ${PCL_LIBRARIES})

ament_auto_add_executable(gps_waypoint_collector_node src/gps_waypoint_collector_node.cpp)
target_link_libraries(gps_waypoint_collector_node gps_waypoint_collector)

ament_auto_add_executable(pcl2octomap_converter_node  src/pcl_helpers.cpp src/pcl2octomap_converter_node.cpp)
target_link_libraries(pcl2octomap_converter_node ${PCL_LIBRARIES})

ament_auto_add_executable(planner_benchmarking_node src/planner_benchmarking_node.cpp)
target_link_libraries(planner_benchmarking_node ${LIBFCL_LIBRARIES} tf_helpers elevation_state_space planner_helpers ompl)

ament_auto_package(INSTALL_TO_SHARE config launch)
