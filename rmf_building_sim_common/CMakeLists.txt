cmake_minimum_required(VERSION 3.5)
project(rmf_building_sim_common)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(menge_vendor REQUIRED)

include(GNUInstallDirs)

###############################
# Utils                       #
###############################

add_library(rmf_building_sim_utils SHARED ${PROJECT_SOURCE_DIR}/src/utils.cpp)

target_include_directories(rmf_building_sim_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# door stuff
###############################

add_library(door_common SHARED src/door_common.cpp)

ament_target_dependencies(door_common
    rclcpp
    rmf_door_msgs
)

target_include_directories(door_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(door_common
    rmf_building_sim_utils
)

###############################
# lift stuff
###############################

add_library(lift_common SHARED src/lift_common.cpp)

ament_target_dependencies(lift_common
   rclcpp
   rmf_door_msgs
   rmf_lift_msgs
)

target_include_directories(lift_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(lift_common
    rmf_building_sim_utils
)

###############################
# crowd simulator stuff
###############################
add_library(crowd_simulator_common
  SHARED
  src/crowd_simulator_common.cpp
)

target_include_directories(crowd_simulator_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${menge_vendor_INCLUDE_DIRS}
)

ament_target_dependencies(crowd_simulator_common
  menge_vendor
  rclcpp
)

#crowd_simulation_common_install
ament_export_targets(crowd_simulator_common HAS_LIBRARY_TARGET)
install(
  TARGETS crowd_simulator_common
  EXPORT crowd_simulator_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

###############################
# install stuff
###############################
ament_export_dependencies(
  rclcpp
  rmf_door_msgs
  rmf_lift_msgs
  rmf_building_map_msgs
  menge_vendor
)

ament_export_include_directories(include)
ament_export_targets(rmf_building_sim_utils HAS_LIBRARY_TARGET)
ament_export_targets(door_common HAS_LIBRARY_TARGET)
ament_export_targets(lift_common HAS_LIBRARY_TARGET)

install(
  TARGETS rmf_building_sim_utils
  EXPORT rmf_building_sim_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS door_common 
  EXPORT door_common 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS lift_common
  EXPORT lift_common 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/rmf_building_sim_common
  DESTINATION include
)

ament_package()
