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

find_package(Eigen3 REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_dispenser_msgs REQUIRED)
find_package(rmf_ingestor_msgs REQUIRED)
find_package(rmf_building_map_msgs REQUIRED)
find_package(rmf_robot_sim_common REQUIRED)
find_package(rmf_building_sim_gz_plugins REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Qml
    Quick
  REQUIRED
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

include(GNUInstallDirs)

###############################
# TeleportIngestor Plugin
###############################

add_library(teleport_ingestor SHARED src/TeleportIngestor.cpp)

target_link_libraries(teleport_ingestor
  Eigen3::Eigen
  gz-sim::core
  gz-plugin::core
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
  ${rmf_fleet_msgs_TARGETS}
  ${rmf_ingestor_msgs_TARGETS}
  ${rmf_robot_sim_common_TARGETS}
)

target_include_directories(teleport_ingestor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# TeleportDispenser Plugin
###############################

add_library(teleport_dispenser SHARED src/TeleportDispenser.cpp)

target_link_libraries(teleport_dispenser
  Eigen3::Eigen
  gz-sim::core
  gz-plugin::core
  gz-msgs::core
  gz-transport::core
  rclcpp::rclcpp
  ${rmf_dispenser_msgs_TARGETS}
  ${rmf_fleet_msgs_TARGETS}
  ${rmf_robot_sim_common_TARGETS}
)

target_include_directories(teleport_dispenser
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Readonly Plugin             #
###############################

add_library(readonly SHARED ${PROJECT_SOURCE_DIR}/src/readonly.cpp)

target_link_libraries(readonly
  Eigen3::Eigen
  gz-sim::core
  gz-plugin::core
  rclcpp::rclcpp
  ${rmf_building_map_msgs_TARGETS}
  ${rmf_fleet_msgs_TARGETS}
  ${rmf_robot_sim_common_TARGETS}
)

target_include_directories(readonly
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# Slotcar Plugin              #
###############################

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

target_link_libraries(slotcar
  gz-sim::core
  gz-plugin::core
  rclcpp::rclcpp
  ${rmf_building_sim_gz_plugins_TARGETS}
  ${rmf_dispenser_msgs_TARGETS}
  ${rmf_fleet_msgs_TARGETS}
  ${rmf_ingestor_msgs_TARGETS}
  ${rmf_robot_sim_common_TARGETS}
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf_building_sim_gz_plugins_INCLUDE_DIRS}
)

###############################
# LightTuning
###############################

QT5_ADD_RESOURCES(resources_RCC src/LightTuning/LightTuning.qrc)

add_library(LightTuning SHARED ${headers_MOC}
  src/LightTuning/LightTuning.cpp
  ${resources_RCC}
)

target_link_libraries(LightTuning
  gz-gui::core
  gz-sim::core
  gz-msgs::core
  gz-transport::core
  gz-rendering::core
  Qt5::Core
  Qt5::Qml
  Qt5::Quick
  rclcpp::rclcpp
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
