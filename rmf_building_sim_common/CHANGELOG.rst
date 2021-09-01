^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_building_sim_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2021-09-01)
------------------
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/issues/26>`_)
* Changes for galactic (`#25 <https://github.com/open-rmf/rmf_simulation/issues/25>`_)
* Removing unnecessary qt5 dependency from CMakeLists (`#22 <https://github.com/open-rmf/rmf_simulation/issues/22>`_)
* Make menge mandatory and fix building_sim dependencies (`#19 <https://github.com/open-rmf/rmf_simulation/issues/19>`_)
* Start using ros-tooling for build and test workflow, added coverage, tsan (`#14 <https://github.com/open-rmf/rmf_simulation/issues/14>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/issues/1>`_)
* Crowd step size fix for large physics step sizes (`#10 <https://github.com/open-rmf/rmf_simulation/issues/10>`_)
* Add build and style actions (`#11 <https://github.com/open-rmf/rmf_simulation/issues/11>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/issues/5>`_)
* Refactor and cleanup utils (`#6 <https://github.com/open-rmf/rmf_simulation/issues/6>`_)
* account for renaming from building_map_msgs to rmf_building_map_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/issues/3>`_)
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Yadu

1.2.0 (2021-01-06)
------------------
* Add animation switch to crowd simulation plugin (`#238 <https://github.com/osrf/traffic_editor/pull/238>`_)
* Add pausing feature to slotcar plugin: [`#267 <https://github.com/osrf/traffic_editor/pull/267>`_]
* undo features
* dropping eloquent support (`#277 <https://github.com/osrf/traffic_editor/issues/277>`_)
* Fix undefined initialization of _old_ang_vel and _old_lin_vel (`#274 <https://github.com/osrf/traffic_editor/issues/274>`_)
* idle mode when slotcar is stationary (`#264 <https://github.com/osrf/traffic_editor/issues/264>`_)
* Add pausing feature (`#267 <https://github.com/osrf/traffic_editor/issues/267>`_)
* Debug/slotcar battery and mode (`#261 <https://github.com/osrf/traffic_editor/issues/261>`_)
* Added ament exports for crowd simulation common (`#260 <https://github.com/osrf/traffic_editor/issues/260>`_)
* Merge branch 'release-1.1'
* Make slotcar rotations follow trajectory yaw angle (`#254 <https://github.com/osrf/traffic_editor/issues/254>`_)
* Fixed RobotMode (`#252 <https://github.com/osrf/traffic_editor/issues/252>`_)
* Fix namespace for rmf charging plugin (`#253 <https://github.com/osrf/traffic_editor/issues/253>`_)
* Control slotcar with model velocity cmds in place of joint velocity cmds (`#236 <https://github.com/osrf/traffic_editor/issues/236>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/issues/242>`_)
* Implement animation switching in crowd simulation (`#238 <https://github.com/osrf/traffic_editor/issues/238>`_)
* Contributors: Geoffrey Biggs, Grey, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Rushyendra Maganty, Yadu, youliang

1.1.0 (2020-09-24)
------------------
* Crowd simulation plugin (`#218 <https://github.com/osrf/traffic_editor/issues/218>`_)
* Improve lift initial floor definition `#221 <https://github.com/osrf/traffic_editor/issues/221>`_
* Update lift session id in lift plugin (`#223 <https://github.com/osrf/traffic_editor/issues/223>`_)
* Add field in lift dialog for initial floor, handle invalid initial floor
* Ignition plugins and modularization of doors and slotcar `#138 <https://github.com/osrf/traffic_editor/issues/138>`_
* Adding lift plugin for ignition `#171 <https://github.com/osrf/traffic_editor/issues/171>`_
* spawn lifts at respective reference floors
* stagger door_state publishing
* Contributors: Charayaphan Nakorn Boon Han, Guoliang (Fred) Shao, Kevin_Skywalker, Luca Della Vedova, MakinoharaShouko
