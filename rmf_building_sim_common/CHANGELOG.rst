^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf\_building\_sim\_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.2 (2023-12-20)
------------------
* Sanitize node names to avoid plugin exceptions (`#110 <https://github.com/open-rmf/rmf_simulation/pull/110>`_)
* Contributors: Luca Della Vedova

2.2.1 (2023-06-30)
------------------

2.2.0 (2023-06-08)
------------------

2.1.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#101 <https://github.com/open-rmf/rmf_simulation/pull/101>`_)
* Don't print warning if lift is going to same floor (`#89 <https://github.com/open-rmf/rmf_simulation/pull/89>`_)
* Contributors: Luca Della Vedova, Yadunund

2.0.0 (2022-10-04)
------------------
* Use ECM instead of traversing sdformat parent for Gazebo doors (`#80 <https://github.com/open-rmf/rmf_simulation/pull/80>`_)
* Change default lift current mode to AGV (`#52 <https://github.com/open-rmf/rmf_simulation/pull/52>`_)
* Contributors: Arjo Chakravarty, Charayaphan Nakorn Boon Han, Luca Della Vedova

1.3.0 (2021-09-01)
------------------
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/pull/26>`_)
* Changes for galactic (`#25 <https://github.com/open-rmf/rmf_simulation/pull/25>`_)
* Removing unnecessary qt5 dependency from CMakeLists (`#22 <https://github.com/open-rmf/rmf_simulation/pull/22>`_)
* Make menge mandatory and fix building\_sim dependencies (`#19 <https://github.com/open-rmf/rmf_simulation/pull/19>`_)
* Start using ros-tooling for build and test workflow, added coverage, tsan (`#14 <https://github.com/open-rmf/rmf_simulation/pull/14>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/pull/1>`_)
* Crowd step size fix for large physics step sizes (`#10 <https://github.com/open-rmf/rmf_simulation/pull/10>`_)
* Add build and style actions (`#11 <https://github.com/open-rmf/rmf_simulation/pull/11>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/pull/5>`_)
* Refactor and cleanup utils (`#6 <https://github.com/open-rmf/rmf_simulation/pull/6>`_)
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/pull/3>`_)
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Yadunund

1.2.0 (2021-01-06)
------------------
* Add pausing feature to slotcar plugin (`#267 <https://github.com/osrf/traffic_editor/pull/267>`_)
* dropping eloquent support (`#277 <https://github.com/osrf/traffic_editor/pull/277>`_)
* Fix undefined initialization of \_old\_ang\_vel and \_old\_lin\_vel (`#274 <https://github.com/osrf/traffic_editor/pull/274>`_)
* idle mode when slotcar is stationary (`#264 <https://github.com/osrf/traffic_editor/pull/264>`_)
* Debug/slotcar battery and mode (`#261 <https://github.com/osrf/traffic_editor/pull/261>`_)
* Added ament exports for crowd simulation common (`#260 <https://github.com/osrf/traffic_editor/pull/260>`_)
* Make slotcar rotations follow trajectory yaw angle (`#254 <https://github.com/osrf/traffic_editor/pull/254>`_)
* Fixed RobotMode (`#252 <https://github.com/osrf/traffic_editor/pull/252>`_)
* Fix namespace for rmf charging plugin (`#253 <https://github.com/osrf/traffic_editor/pull/253>`_)
* Control slotcar with model velocity cmds in place of joint velocity cmds (`#236 <https://github.com/osrf/traffic_editor/pull/236>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/pull/242>`_)
* Implement animation switching in crowd simulation (`#238 <https://github.com/osrf/traffic_editor/pull/238>`_)
* Contributors: Geoffrey Biggs, Grey, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Rushyendra Maganty, Yadunund, youliang

1.1.0 (2020-09-24)
------------------
* Crowd simulation plugin (`#218 <https://github.com/osrf/traffic_editor/pull/218>`_)
* Improve lift initial floor definition (`#221 <https://github.com/osrf/traffic_editor/pull/221>`_)
* Update lift session id in lift plugin (`#223 <https://github.com/osrf/traffic_editor/pull/223>`_)
* Ignition plugins and modularization of doors and slotcar (`#138 <https://github.com/osrf/traffic_editor/pull/138>`_)
* Adding lift plugin for ignition (`#171 <https://github.com/osrf/traffic_editor/pull/171>`_)
* stagger door\_state publishing
* Contributors: Charayaphan Nakorn Boon Han, Guoliang (Fred) Shao, Kevin\_Skywalker, Luca Della Vedova, MakinoharaShouko
