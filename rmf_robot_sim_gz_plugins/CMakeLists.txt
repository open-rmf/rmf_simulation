cmake_minimum_required(VERSION 3.5)
project(rmf_robot_sim_gz_plugins)

set (CMAKE_AUTOMOC ON)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(gz-cmake3 REQUIRED)

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER 8)
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER 2)
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER 5)
gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER 7)
gz_find_package(gz-gui8 REQUIRED)
set(GZ_GUI_VER 8)
gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER 10)
gz_find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER 13)
gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER 8)
gz_find_package(sdformat14 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_robot_sim_common REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Qml
    Quick
  REQUIRED
)

include(GNUInstallDirs)

###############################
# TeleportIngestor Plugin
###############################

add_library(teleport_ingestor SHARED src/TeleportIngestor.cpp)

ament_target_dependencies(teleport_ingestor
    gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}
    gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}
    rmf_fleet_msgs
    rclcpp
    rmf_robot_sim_common
    Eigen3
)

target_include_directories(teleport_ingestor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# TeleportDispenser Plugin
###############################

add_library(teleport_dispenser SHARED src/TeleportDispenser.cpp)

ament_target_dependencies(teleport_dispenser
    gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}
    gz-msgs${GZ_MSGS_VER}
    gz-transport${GZ_TRANSPORT_VER}
    rmf_fleet_msgs
    rclcpp
    rmf_robot_sim_common
    Eigen3
)

target_include_directories(teleport_dispenser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# Readonly Plugin             #
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

ament_target_dependencies(readonly
    rmf_fleet_msgs
    rmf_building_map_msgs
    rclcpp
    gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}
    rmf_robot_sim_common
    Eigen3
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
)

###############################
# Slotcar Plugin              #
###############################

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

ament_target_dependencies(slotcar
  PUBLIC
    gz-sim${GZ_SIM_VER}
    gz-plugin${GZ_PLUGIN_VER}
    rmf_robot_sim_common
    rmf_fleet_msgs
    rclcpp
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_robot_sim_common_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_building_map_msgs_INCLUDE_DIRS}
)

###############################
# LightTuning
###############################

QT5_ADD_RESOURCES(resources_RCC src/LightTuning/LightTuning.qrc)

add_library(LightTuning SHARED ${headers_MOC}
  src/LightTuning/LightTuning.cpp
  ${resources_RCC}
)

ament_target_dependencies(LightTuning
  gz-gui${GZ_GUI_VER}
  gz-sim${GZ_SIM_VER}
  gz-msgs${GZ_MSGS_VER}
  gz-transport${GZ_TRANSPORT_VER}
  gz-rendering${GZ_RENDERING_VER}
  Qt5Core
  Qt5Qml
  Qt5Quick
  rclcpp
)

target_include_directories(LightTuning
  PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

###############################
# Install Targets
###############################

install(
  TARGETS
    teleport_dispenser
    teleport_ingestor
    readonly
    slotcar
    LightTuning
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

ament_package()
