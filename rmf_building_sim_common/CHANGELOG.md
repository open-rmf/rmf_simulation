## Changelog for package rmf\_building\_sim\_common

2.1.0 (2022-XX-XX)
------------------
* Don't print warning if lift is going to same floor ([#89](https://github.com/open-rmf/rmf_simulation/pull/89))
* Contributors: Luca Della Vedova

2.0.0 (2022-XX-XX)
------------------
* Use ECM instead of traversing sdformat parent for Gazebo doors ([#80](https://github.com/open-rmf/rmf_simulation/pull/80))
* Change default lift current mode to AGV ([#52](https://github.com/open-rmf/rmf_simulation/pull/52))
* Contributors: Arjo Chakravarty, Charayaphan Nakorn Boon Han, Luca Della Vedova

1.3.0 (2021-09-01)
------------------
* Fix dependencies ([#26](https://github.com/open-rmf/rmf_simulation/issues/26))
* Changes for galactic ([#25](https://github.com/open-rmf/rmf_simulation/issues/25))
* Removing unnecessary qt5 dependency from CMakeLists ([#22](https://github.com/open-rmf/rmf_simulation/issues/22))
* Make menge mandatory and fix building\_sim dependencies ([\19](https://github.com/open-rmf/rmf_simulation/issues/19))
* Start using ros-tooling for build and test workflow, added coverage, tsan ([#14](https://github.com/open-rmf/rmf_simulation/issues/14))
* Add quality declaration documents ([#1](https://github.com/open-rmf/rmf_simulation/issues/1))
* Crowd step size fix for large physics step sizes ([#10](https://github.com/open-rmf/rmf_simulation/issues/10))
* Add build and style actions ([#11](https://github.com/open-rmf/rmf_simulation/issues/11))
* Slotcar plugin package move and utils cleanup ([#5](https://github.com/open-rmf/rmf_simulation/issues/5))
* Refactor and cleanup utils ([#6](https://github.com/open-rmf/rmf_simulation/issues/6))
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs ([#3](https://github.com/open-rmf/rmf_simulation/issues/3))
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Yadunund

1.2.0 (2021-01-06)
------------------
* Add animation switch to crowd simulation plugin ([#238](https://github.com/osrf/traffic_editor/pull/238))
* Add pausing feature to slotcar plugin [#267](https://github.com/osrf/traffic_editor/pull/267)
* dropping eloquent support ([#277](https://github.com/osrf/traffic_editor/issues/277))
* Fix undefined initialization of \_old\_ang\_vel and \_old\_lin\_vel ([#274](https://github.com/osrf/traffic_editor/issues/274))
* idle mode when slotcar is stationary ([#264](https://github.com/osrf/traffic_editor/issues/264))
* Add pausing feature ([#267](https://github.com/osrf/traffic_editor/issues/267))
* Debug/slotcar battery and mode ([#261](https://github.com/osrf/traffic_editor/issues/261))
* Added ament exports for crowd simulation common ([#260](https://github.com/osrf/traffic_editor/issues/260))
* Make slotcar rotations follow trajectory yaw angle ([#254](https://github.com/osrf/traffic_editor/issues/254))
* Fixed RobotMode ([#252](https://github.com/osrf/traffic_editor/issues/252))
* Fix namespace for rmf charging plugin ([#253](https://github.com/osrf/traffic_editor/issues/253))
* Control slotcar with model velocity cmds in place of joint velocity cmds ([#236](https://github.com/osrf/traffic_editor/issues/236))
* Implement battery drain and recharge for slotcars ([#242](https://github.com/osrf/traffic_editor/issues/242))
* Implement animation switching in crowd simulation ([#238](https://github.com/osrf/traffic_editor/issues/238))
* Contributors: Geoffrey Biggs, Grey, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Rushyendra Maganty, Yadunund, youliang

1.1.0 (2020-09-24)
------------------
* Crowd simulation plugin ([\#218](https://github.com/osrf/traffic_editor/issues/218))
* Improve lift initial floor definition [\#221](https://github.com/osrf/traffic_editor/issues/221)
* Update lift session id in lift plugin ([\#223](https://github.com/osrf/traffic_editor/issues/223))
* Ignition plugins and modularization of doors and slotcar [\#138](https://github.com/osrf/traffic_editor/issues/138)
* Adding lift plugin for ignition [\#171](https://github.com/osrf/traffic_editor/issues/171)
* stagger door\_state publishing
* Contributors: Charayaphan Nakorn Boon Han, Guoliang (Fred) Shao, Kevin\_Skywalker, Luca Della Vedova, MakinoharaShouko
