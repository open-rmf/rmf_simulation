## Changelog for package rmf\_building\_sim\_gz\_plugins

2.0.0 (2022-XX-XX)
------------------
* Use ECM instead of traversing sdformat parent for Gazebo doors ([#80](https://github.com/open-rmf/rmf_simulation/pull/80))
* Fix potential segfault in toggle\_floors plugin ([#79](https://github.com/open-rmf/rmf_simulation/pull/79))
* Fix a segfault in toggle floor plugin which causes ignition to crash when scene is not ready ([#78](https://github.com/open-rmf/rmf_simulation/pull/78))
* Renamed to rmf\_building\_sim\_gz\_plugins and Humble migration ([#77](https://github.com/open-rmf/rmf_simulation/pull/77))
* Add an Ignition GUI plugin to toggle floors visibility ([#72](https://github.com/open-rmf/rmf_simulation/pull/72))
* Contributors: Arjo Chakravarty, Luca Della Vedova, Yadunund, chianfern

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix [#100](https://github.com/open-rmf/rmf/issues/100). [#60](https://github.com/open-rmf/rmf_simulation/pull/60)
* Contributors: Marco A. Gutierrez

1.3.0 (2021-09-01)
------------------
* Added rosdep key for ignition-edifice, made it a mandatory dependency ([#31](https://github.com/open-rmf/rmf_simulation/issues/31))
* Skip updating ignition plugins when simulation is paused ([#34](https://github.com/open-rmf/rmf_simulation/issues/34))
* updating for galactic and rolling compatibility ([#29](https://github.com/open-rmf/rmf_simulation/issues/29))
* Fix dependencies ([#26](https://github.com/open-rmf/rmf_simulation/issues/26))
* Make menge mandatory and fix building\_sim dependencies ([#19](https://github.com/open-rmf/rmf_simulation/issues/19))
* Add quality declaration documents ([#1](https://github.com/open-rmf/rmf_simulation/issues/1))
* Crowd step size fix for large physics step sizes ([#10](https://github.com/open-rmf/rmf_simulation/issues/10))
* Update to Ignition Edifice ([#8](https://github.com/open-rmf/rmf_simulation/issues/8))
* Slotcar plugin package move and utils cleanup ([#5](https://github.com/open-rmf/rmf_simulation/issues/5))
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs ([#3](https://github.com/open-rmf/rmf_simulation/issues/3))
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez

1.2.0 (2021-01-06)
------------------
* Remove Slotcar/Lift AABB component when not required to speed up demos ([#271](https://github.com/osrf/traffic_editor/issues/271))
* Migrate ignition plugins to ament\_target\_dependencies ([#262](https://github.com/osrf/traffic_editor/issues/262))
* Enable lifts to work with TPE, modifies lift plugin to issue either joint or model velocity commands based on the physics engine used ([#250](https://github.com/osrf/traffic_editor/issues/250))
* Control slotcar with model velocity cmds in place of joint velocity cmds ([#236](https://github.com/osrf/traffic_editor/issues/236))
* Implement battery drain and recharge for slotcars ([#242](https://github.com/osrf/traffic_editor/issues/242))
* Implement animation switching in crowd simulation ([#238](https://github.com/osrf/traffic_editor/issues/238))
* Add first pass of quality declarations for all packages ([#235](https://github.com/osrf/traffic_editor/issues/235))
* Building\_crowdsim for generating the navmesh file and required configuration files for menge ([#224](https://github.com/osrf/traffic_editor/issues/224))
* Add animation switch to crowd simulation plugin ([#238](https://github.com/osrf/traffic_editor/pull/238))
* Contributors: Geoffrey Biggs, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutierrez, Marco A. Gutiérrez, Rushyendra Maganty

1.0.0 (2020-09-24)
------------------
* Focal / Ignition dome dependencies update ([#230](https://github.com/osrf/traffic_editor/issues/230))
* Ignition crowd simulation plugin ([#218](https://github.com/osrf/traffic_editor/issues/218))
* Create AxisAlignedBox component for slotcar ([#227](https://github.com/osrf/traffic_editor/issues/227))
* Ignition plugins and modularization of doors and slotcar [#138](https://github.com/osrf/traffic_editor/issues/138)
* Adding lift plugin for ignition [#171](https://github.com/osrf/traffic_editor/issues/171)
* Contributors: Guoliang (Fred) Shao, Luca Della Vedova, Rushyendra Maganty, kevinskwk
