cmake_minimum_required(VERSION 3.5)
project(rmf_building_sim_gazebo_plugins)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

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
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(OpenCV REQUIRED )
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_building_sim_common REQUIRED)
find_package(menge_vendor REQUIRED)

include(GNUInstallDirs)

###############################
# door stuff
###############################

add_library(door SHARED src/door.cpp)

ament_target_dependencies(door
  rmf_building_sim_common
  rclcpp
  gazebo_ros
)

target_include_directories(door
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# lift stuff
###############################

add_library(lift SHARED src/lift.cpp)

ament_target_dependencies(lift
    rmf_building_sim_common
    rclcpp
    gazebo_ros
)

target_include_directories(lift
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# toggle floor stuff
###############################

add_library(toggle_floors SHARED src/toggle_floors.cpp)

ament_target_dependencies(toggle_floors
    Qt5
    gazebo_ros
    rmf_fleet_msgs
    rclcpp
)

target_include_directories(toggle_floors
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    #${rmf_building_sim_common_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)

###############################
# toggle charging
###############################

add_library(toggle_charging SHARED src/toggle_charging.cpp)

ament_target_dependencies(toggle_charging
    Qt5
    gazebo_ros
    rmf_fleet_msgs
    rclcpp
)

target_include_directories(toggle_charging
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)


###############################
# thumbnail generation stuff
###############################

add_library(thumbnail_generator SHARED src/thumbnail_generator.cpp)

target_include_directories(thumbnail_generator
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(thumbnail_generator
  PUBLIC
    ${OpenCV_LIBS}
)

###############################
# crowd simulator stuff
###############################
add_library(crowd_simulator
  SHARED
    src/crowd_simulator.cpp
)

ament_target_dependencies(crowd_simulator
  rmf_building_sim_common
  rclcpp
  menge_vendor
)

target_include_directories(crowd_simulator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${GAZEBO_INCLUDE_DIRS}
    ${menge_vendor_INCLUDE_DIRS}
    ${rmf_building_sim_common_INCLUDE_DIRS}
)

###############################
# install stuff
###############################

install(
  TARGETS door lift toggle_floors toggle_charging thumbnail_generator crowd_simulator
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
)

ament_package()
