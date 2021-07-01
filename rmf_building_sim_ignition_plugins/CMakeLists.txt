cmake_minimum_required(VERSION 3.5)
project(rmf_building_sim_ignition_plugins)

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

find_package(ignition-cmake2 QUIET)
if (NOT ignition-cmake2_FOUND)
  message("ignition-cmake not found, skipping ignition gazebo plugins")
  return()
endif()
ign_find_package(ignition-gazebo5 QUIET)
if (NOT ignition-gazebo5_FOUND)
  message("Wrong ignition-gazebo version found. Plugins require ignition-gazebo5 (edifice)")
  # To generate a setup.bash and avoid CMake warnings
  ament_package()
  return()
endif()
set(IGN_GAZEBO_VER 5)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-common4 REQUIRED)
set(IGN_COMMON_VER 4)
ign_find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER 6)
ign_find_package(ignition-gui5 REQUIRED)
set(IGN_GUI_VER 5)
ign_find_package(ignition-msgs7 REQUIRED)
set(IGN_MSGS_VER 7)
ign_find_package(ignition-transport10 REQUIRED)
set(IGN_TRANSPORT_VER 10)
ign_find_package(sdformat11 REQUIRED)

find_package(rmf_building_sim_common REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Qml
    Quick
  REQUIRED
)
find_package(menge_vendor REQUIRED)

include(GNUInstallDirs)

###############################
# door stuff
###############################

add_library(door SHARED src/door.cpp)

target_include_directories(door
  PUBLIC
    ${rmf_building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(door
  PUBLIC
    rclcpp
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rmf_building_sim_common
)

###############################
# lift stuff
###############################

add_library(lift SHARED src/lift.cpp)

target_include_directories(lift
  PUBLIC
    ${rmf_building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(lift
  PUBLIC
    rclcpp
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rmf_building_sim_common
)

###############################
# crowd simulator stuff
###############################
add_library(crowd_simulator
  SHARED
  src/crowd_simulator.cpp
)

target_include_directories(crowd_simulator
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${rmf_building_sim_common_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${menge_vendor_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

ament_target_dependencies(crowd_simulator
  ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}
  rclcpp
  rmf_building_sim_common
  menge_vendor
)

###############################
# toggle charging
###############################

QT5_ADD_RESOURCES(resources_RCC src/toggle_charging/toggle_charging.qrc)

add_library(toggle_charging SHARED ${headers_MOC}
  src/toggle_charging/toggle_charging.cpp
  ${resources_RCC}
)

ament_target_dependencies(toggle_charging
  ignition-gui${IGN_GUI_VER}
  ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}
  Qt5Core
  Qt5Qml
  Qt5Quick
  rclcpp
)

target_include_directories(toggle_charging
  PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

###############################
# install stuff
###############################

install(
  TARGETS door lift crowd_simulator toggle_charging
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
