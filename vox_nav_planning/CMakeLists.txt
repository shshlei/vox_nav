cmake_minimum_required(VERSION 3.5)
project(vox_nav_planning)

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

set(CMAKE_MODULE_PATH
    "${CMAKE_MODULE_PATH}"
    "${CMAKE_ROOT_DIR}/cmake/Modules"
    "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
link_directories(${OMPL_LIBRARY_DIRS})

find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
# Try for OpenCV 4.X, but settle for whatever is installed
find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
Message(STATUS "Found OpenCV version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

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

find_package(adolc REQUIRED)
pkg_check_modules(ipopt REQUIRED IMPORTED_TARGET ipopt)

find_package(Box2DCollision REQUIRED)

# BOX2d PLANNER SERVER ##############################################
ament_auto_add_executable(box2d_planner_server src/box2d_planner_server.cpp src/box2d_planner_core.cpp)
target_link_libraries(box2d_planner_server ompl /opt/ros/humble/lib/libompl_app_base.so ${BULLET_LIBRARIES})

# BOX2d PLANNER CLIENT ##############################################
ament_auto_add_executable(box2d_planner_client src/box2d_planner_client.cpp)

# BOX2d PLANNER PSOPT ##############################################
ament_auto_add_executable(box2d_trajectory_server src/box2d_trajectory_server.cpp)
target_include_directories(box2d_trajectory_server PUBLIC ${Box2DCollision_INCLUDE_DIR})
target_link_libraries(box2d_trajectory_server adolc PkgConfig::ipopt ${Box2DCollision_LIBRARY})

# Native Voxnav sampling-based PLANNERS ######################
ament_auto_add_library(vox_nav_ompl_planners SHARED  src/native_planners/RRTStarF.cpp
  src/native_planners/LQRPlanner.cpp
  src/native_planners/LQRRRTStar.cpp 
  src/native_planners/InformedSGCP.cpp
  src/native_planners/CostTrustKinoPlanner.cpp)
target_link_libraries(vox_nav_ompl_planners ${PCL_LIBRARIES})

# PLANNER SERVER ##############################################
ament_auto_add_executable(planner_server src/planner_server.cpp)
target_link_libraries(planner_server ${LIBFCL_LIBRARIES} ompl)

# PLANNER CLIENT ##############################################
ament_auto_add_executable(planner_client src/planner_client.cpp)

# SE2 PLANNE R##################################################
set(se2_planner vox_nav_se2_planner)
ament_auto_add_library(se2_planner SHARED src/plugins/se2_planner.cpp)
target_link_libraries(se2_planner ${LIBFCL_LIBRARIES} ompl)

# SE3 PLANNE R##################################################
set(se3_planner vox_nav_se3_planner)
ament_auto_add_library(se3_planner SHARED src/plugins/se3_planner.cpp)
target_link_libraries(se3_planner ${LIBFCL_LIBRARIES} ompl)

# ELEVATION PLANNER ############################################
ament_auto_add_library(elevation_planner SHARED src/plugins/elevation_planner.cpp)
target_link_libraries(elevation_planner ${LIBFCL_LIBRARIES} ompl)

# ELEVATION CONTROL PLANNER ############################################
ament_auto_add_library(elevation_control_planner SHARED src/plugins/elevation_control_planner.cpp)
target_link_libraries(elevation_control_planner ${LIBFCL_LIBRARIES} ompl vox_nav_ompl_planners)

# OPTIMAL ELEVATION PLANNER ####################################
ament_auto_add_library(optimal_elevation_planner SHARED src/plugins/optimal_elevation_planner.cpp)
target_link_libraries(optimal_elevation_planner ${LIBFCL_LIBRARIES} ompl)

# OSM ELEVATION PLANNER ####################################
ament_auto_add_library(osm_elevation_planner SHARED src/plugins/osm_elevation_planner.cpp)
target_link_libraries(osm_elevation_planner ${LIBFCL_LIBRARIES} ompl vox_nav_ompl_planners)

# POLYTUNNEL PLANNER ####################################
ament_auto_add_library(polytunnel_planner SHARED src/plugins/polytunnel_planner.cpp)
target_link_libraries(polytunnel_planner ${LIBFCL_LIBRARIES} ${PCL_LIBRARIES} ompl)

# ELEVATION CONTROL PLANNERS BENCHMARKS ####################################
ament_auto_add_executable(car_control_planners_benchmark src/tools/car_control_planners_benchmark.cpp)
target_link_libraries(car_control_planners_benchmark ${LIBFCL_LIBRARIES} vox_nav_ompl_planners ompl)

# QUADROTOR CONTROL PLANNERS BENCHMARKS ####################################
ament_auto_add_executable(quadrotor_control_planners_benchmark src/tools/quadrotor_control_planners_benchmark.cpp)
target_link_libraries(quadrotor_control_planners_benchmark ${LIBFCL_LIBRARIES} vox_nav_ompl_planners ompl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
        
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_auto_package(INSTALL_TO_SHARE config params launch)
