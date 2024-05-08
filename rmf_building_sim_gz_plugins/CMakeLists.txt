cmake_minimum_required(VERSION 3.5)
project(rmf_building_sim_gz_plugins)

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
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-rendering REQUIRED)

find_package(rmf_fleet_msgs REQUIRED)
find_package(Qt5
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
    ${rmf_door_msgs_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(door
  gz-sim::core
  gz-plugin::core
)

ament_target_dependencies(door
    rclcpp
    rmf_door_msgs
)

###############################
# register_component stuff
###############################

add_library(register_component SHARED src/register_component.cpp)

target_include_directories(register_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(register_component
  gz-sim::core
  gz-plugin::core
)

###############################
# lift stuff
###############################

add_library(lift SHARED src/lift.cpp)

target_include_directories(lift
  PUBLIC
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(lift
  gz-sim::core
  gz-plugin::core
)

ament_target_dependencies(lift
  rclcpp
  rmf_lift_msgs
  rmf_door_msgs
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
  ${rclcpp_INCLUDE_DIRS}
  ${menge_vendor_INCLUDE_DIRS}
)

target_link_libraries(crowd_simulator
  gz-sim::core
  gz-plugin::core
)

ament_target_dependencies(crowd_simulator
  rclcpp
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

target_link_libraries(toggle_charging
  gz-sim::core
  gz-msgs::core
  gz-transport::core
)

ament_target_dependencies(toggle_charging
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
# toggle floors
###############################

QT5_ADD_RESOURCES(resources_RCC src/toggle_floors/toggle_floors.qrc)

add_library(toggle_floors SHARED ${headers_MOC}
  src/toggle_floors/toggle_floors.cpp
  ${resources_RCC}
)

target_link_libraries(toggle_floors
  gz-sim::core
  gz-gui::core
  gz-msgs::core
  gz-plugin::core
  gz-rendering::core
  gz-transport::core
)

ament_target_dependencies(toggle_floors
  Qt5Core
  Qt5Qml
  Qt5Quick
  rclcpp
  rmf_fleet_msgs
)

target_include_directories(toggle_floors
  PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

###############################
# install stuff
###############################

ament_export_include_directories(include)

install(
  TARGETS door lift crowd_simulator toggle_charging toggle_floors register_component
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
