This document is a declaration of software quality for the `rmf_robot_sim_ignition_plugins` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_robot_sim_ignition_plugins` Quality Declaration

This package claims to be in the **Quality Level 4** category.

Below are the detailed rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

This package uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

This package is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

This package does not have a public API.

### API Stability Policy [1.iv]

This package does not have a public API.

### ABI Stability Policy [1.v]

This package does not have a public API.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

This package does not have a public API.

## Change Control Process [2]

This package follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

This package requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

This package uses DCO as its confirmation of contributor origin policy.
More information can be found in [CONTRIBUTING](../CONTRIBUTING.md).

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.
The CI checks only that the package builds.
The most recent CI results can be seen on [the workflow page](https://github.com/open-rmf/rmf_simulation/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

This package is documented in the [parent repository's README.md file](https://github.com/open-rmf/rmf_simulation/blob/master/README.md).

### Public API Documentation [3.ii]

This package does not have a public API.

### License [3.iii]

The license for this package is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in this package.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

This package does not have any tests.

### Public API Testing [4.ii]

This package does not have a public API.

### Coverage [4.iii]

This package does not track coverage statistics.

### Performance [4.iv]

This package does not have performance tests.

### Linters and Static Analysis [4.v]

This package does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

This package has the following direct runtime ROS dependencies.

#### rclcpp

`rclcpp` is [**Quality Level 1**](https://github.com/ros2/rclcpp/blob/master/rclcpp/QUALITY_DECLARATION.md).

#### rmf\_fleet\_msgs

`rmf_fleet_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_internal_msgs/blob/main/rmf_fleet_msgs/QUALITY_DECLARATION.md).

#### rmf\_building\_map\_msgs

`rmf_building_map_msgs` is [**Quality Level 3**](https://github.com/open-rmf/rmf_building_map_msgs/blob/main/rmf_building_map_msgs/QUALITY_DECLARATION.md).

#### rmf\_robot\_sim\_common

`rmf_robot_sim_common` is [**Quality Level 4**](https://github.com/open-rmf/rmf_simulation/blob/master/rmf_robot_sim_common/QUALITY_DECLARATION.md).

### Optional Direct Runtime ROS Dependencies [5.ii]

This package does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

This package has the following runtime non-ROS dependencies.

#### eigen

`eigen` does not declare a quality level.
It is assumed to be QL1 based on wide-spread use.

#### ignition-gazebo

`ignition-gazebo` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-plugin

`ignition-plugin` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-common

`ignition-common` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-math

`ignition-math` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-gui

`ignition-gui` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-msgs

`ignition-msgs` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-transport

`ignition-transport` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### ignition-rendering

`ignition-rendering` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### sdformat11

`sdformat11` does not declare a quality level.
It is assumed to be at least **Quality Level 3** based on its history, use of CI, and use of testing.

#### libqt5-core

`libqt5-core` is widely-used third-party software for building graphical applications.
Due to its wide use, documentation, and testing, it is assumed to be **Quality Level 3**.

#### libqt5-qml

`libqt5-qml` is widely-used third-party software for building graphical applications.
Due to its wide use, documentation, and testing, it is assumed to be **Quality Level 3**.

#### libqt5-quick

`libqt5-quick` is widely-used third-party software for building graphical applications.
Due to its wide use, documentation, and testing, it is assumed to be **Quality Level 3**.

## Platform Support [6]

This package does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
This package supports ROS Foxy.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
