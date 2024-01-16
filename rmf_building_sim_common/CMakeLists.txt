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
  menge_vendor
)

ament_export_include_directories(include)

install(
  DIRECTORY include/rmf_building_sim_common
  DESTINATION include
)

ament_package()
