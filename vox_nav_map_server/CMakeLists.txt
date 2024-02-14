cmake_minimum_required(VERSION 3.5)
project(vox_nav_map_server C CXX)

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

include_directories(include)

ament_auto_add_library(map_manager_helpers SHARED src/map_manager_helpers.cpp)

ament_auto_add_executable(map_manager_no_gps src/map_manager_no_gps.cpp)
target_link_libraries(map_manager_no_gps map_manager_helpers )

ament_auto_add_executable(map_manager src/map_manager.cpp)
target_link_libraries(map_manager map_manager_helpers )

ament_auto_add_executable(osm_map_manager src/osm_map_manager.cpp)
target_link_libraries(osm_map_manager map_manager_helpers )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch urdf meshes)
