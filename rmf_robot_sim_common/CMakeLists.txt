cmake_minimum_required(VERSION 3.5)
project(rmf_robot_sim_common)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)

include(GNUInstallDirs)

###############################
# Utils                       #
###############################

add_library(rmf_robot_sim_utils SHARED ${PROJECT_SOURCE_DIR}/src/utils.cpp)

target_link_libraries(rmf_robot_sim_utils
  rclcpp::rclcpp
  ${Eigen3_LIBRARIES}
)

target_include_directories(rmf_robot_sim_utils
  PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###################################
# Readonly Plugin Common #
###################################

add_library(readonly_common SHARED ${PROJECT_SOURCE_DIR}/src/readonly_common.cpp)

target_link_libraries(readonly_common
    rmf_robot_sim_utils
)

ament_target_dependencies(readonly_common
    rmf_fleet_msgs
    rmf_building_map_msgs
    rclcpp
    std_msgs
    Eigen3
)

target_include_directories(readonly_common
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###################################
# Slotcar Plugin Common           #
###################################

add_library(slotcar_common SHARED ${PROJECT_SOURCE_DIR}/src/slotcar_common.cpp)

ament_target_dependencies(slotcar_common
    Eigen3
    rmf_fleet_msgs
    rmf_building_map_msgs
    rclcpp
    std_msgs
    geometry_msgs
    tf2_ros
)

target_include_directories(slotcar_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(slotcar_common
    rmf_robot_sim_utils
)

###############################
# Install Targets             #
###############################
ament_export_dependencies(
  Eigen3
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
  rmf_fleet_msgs
  rmf_building_map_msgs
)

ament_export_include_directories(include)

# cmake is sensitive to the order of these ament_export_targets calls
# each library must be listed after any other libraries that it depends on
ament_export_targets(rmf_robot_sim_utils HAS_LIBRARY_TARGET)
ament_export_targets(readonly_common HAS_LIBRARY_TARGET)
ament_export_targets(slotcar_common HAS_LIBRARY_TARGET)

install(
  TARGETS readonly_common
  EXPORT readonly_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS rmf_robot_sim_utils
  EXPORT rmf_robot_sim_utils
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS slotcar_common
  EXPORT slotcar_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
