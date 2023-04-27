# rmf\_simulation

This repository contains simulation plugins used in Open-RMF. It currently supports Gazebo Classic 11 and Gazebo Fortress.

Simulation plugins are split into two categories:

1. Building simulation plugins under `rmf_building_sim_*`, are used to simulate different aspect of buildings such as doors, lifts and crowds (currently through the [Menge library](https://github.com/open-rmf/menge_vendor)). Utility plugins to help with simulation, such as UI widgets to toggle battery consumption and set charging behavior, as well as toggling floor visibility, are also placed in this category.

1. Robot simulation plugins under `rmf_robot_sim_*` are used to simulate aspects of robots behavior.
The main plugin is `slotcar` that acts as a simple robot navigation stack, controlling the robot and sending updates to RMF through ROS 2 topics.
A `readonly` plugin is used to simulate vehicles with read only RMF integration. Finally ingestor and dispenser plugins are used for simple delivery tasks by teleporting objects to / from the target robot when commanded to do so.

#### Repository organization

Most plugins are split into a `common`, `gz` and `gz_classic` package. The `common` package contains the simulator independent logic, such as ROS 2 publishers / subscribers, core logic, while the `gz` and `gz_classic` packages contain a wrapper to interface to the specific simulators.

#### Supported versions

Since Gazebo and ROS 2 have a different release schedule, this package will follow the [ros_gz](https://github.com/gazebosim/ros_gz) supported version and, depending on which ROS 2 version RMF is currently targeting, the Gazebo version will be set accordingly to allow using the binary release of the `ros_gz_bridge`.
For example, previous supported versions have been `ROS Galactic <-> Gazebo Edifice` and `ROS Humble <-> Gazebo Fortress`.

Gazebo Classic support through the `gz_classic` plugins is only guaranteed until Gazebo 11 EOL (currently scheduled for January 2025) and might be removed afterwards. Users are encouraged to use Gazebo for future development.
