^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf\_building\_sim\_gz\_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.2 (2024-06-15)
------------------

2.3.1 (2024-06-12)
------------------
* Fix door drift in simulation (`#126 <https://github.com/open-rmf/rmf_simulation/pull/126>`_)
* Contributors: Luca Della Vedova

2.3.0 (2024-06-01)
------------------
* Port outdated actions to Noble (`#122 <https://github.com/open-rmf/rmf_simulation/pull/122>`_)
* Refactor plugins in an ECS based way and migrate to Harmonic (`#114 <https://github.com/open-rmf/rmf_simulation/pull/114>`_)
* Contributors: Arjo Chakravarty, Grey, Luca Della Vedova

2.2.2 (2023-12-20)
------------------
* Sanitize node names to avoid plugin exceptions (`#110 <https://github.com/open-rmf/rmf_simulation/pull/110>`_)
* Contributors: Luca Della Vedova

2.2.1 (2023-06-30)
------------------
* Use ``JointPositionReset`` for open loop door control (`#98 <https://github.com/open-rmf/rmf_simulation/pull/98>`_)
* Contributors: Luca Della Vedova

2.2.0 (2023-06-08)
------------------

2.1.0 (2023-06-06)
------------------
* Switch to rst changelogs (`#101 <https://github.com/open-rmf/rmf_simulation/pull/101>`_)
* Contributors: Yadunund

2.0.0 (2022-04-10)
------------------
* Use ECM instead of traversing sdformat parent for Gazebo doors (`#80 <https://github.com/open-rmf/rmf_simulation/pull/80>`_)
* Fix potential segfault in toggle\_floors plugin (`#79 <https://github.com/open-rmf/rmf_simulation/pull/79>`_)
* Fix a segfault in toggle floor plugin which causes ignition to crash when scene is not ready (`#78 <https://github.com/open-rmf/rmf_simulation/pull/78>`_)
* Renamed to rmf\_building\_sim\_gz\_plugins and Humble migration (`#77 <https://github.com/open-rmf/rmf_simulation/pull/77>`_)
* Add an Ignition GUI plugin to toggle floors visibility (`#72 <https://github.com/open-rmf/rmf_simulation/pull/72>`_)
* Contributors: Arjo Chakravarty, Luca Della Vedova, Yadunund, chianfern

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix (`#100 <https://github.com/open-rmf/rmf/pull/100). [#60](https://github.com/open-rmf/rmf_simulation/pull/6>`_)
* Contributors: Marco A. Gutierrez

1.3.0 (2021-09-01)
------------------
* Added rosdep key for ignition-edifice, made it a mandatory dependency (`#31 <https://github.com/open-rmf/rmf_simulation/pull/31>`_)
* Skip updating ignition plugins when simulation is paused (`#34 <https://github.com/open-rmf/rmf_simulation/pull/34>`_)
* updating for galactic and rolling compatibility (`#29 <https://github.com/open-rmf/rmf_simulation/pull/29>`_)
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/pull/26>`_)
* Make menge mandatory and fix building\_sim dependencies (`#19 <https://github.com/open-rmf/rmf_simulation/pull/19>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/pull/1>`_)
* Crowd step size fix for large physics step sizes (`#10 <https://github.com/open-rmf/rmf_simulation/pull/10>`_)
* Update to Ignition Edifice (`#8 <https://github.com/open-rmf/rmf_simulation/pull/8>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/pull/5>`_)
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/pull/3>`_)
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez

1.2.0 (2021-01-06)
------------------
* Remove Slotcar/Lift AABB component when not required to speed up demos (`#271 <https://github.com/osrf/traffic_editor/pull/271>`_)
* Migrate ignition plugins to ament\_target\_dependencies (`#262 <https://github.com/osrf/traffic_editor/pull/262>`_)
* Enable lifts to work with TPE, modifies lift plugin to issue either joint or model velocity commands based on the physics engine used (`#250 <https://github.com/osrf/traffic_editor/pull/250>`_)
* Control slotcar with model velocity cmds in place of joint velocity cmds (`#236 <https://github.com/osrf/traffic_editor/pull/236>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/pull/242>`_)
* Add first pass of quality declarations for all packages (`#235 <https://github.com/osrf/traffic_editor/pull/235>`_)
* Building\_crowdsim for generating the navmesh file and required configuration files for menge (`#224 <https://github.com/osrf/traffic_editor/pull/224>`_)
* Add animation switch to crowd simulation plugin (`#238 <https://github.com/osrf/traffic_editor/pull/238>`_)
* Contributors: Geoffrey Biggs, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutierrez, Marco A. Gutiérrez, Rushyendra Maganty

1.0.0 (2020-09-24)
------------------
* Focal / Ignition dome dependencies update (`#230 <https://github.com/osrf/traffic_editor/pull/230>`_)
* Ignition crowd simulation plugin (`#218 <https://github.com/osrf/traffic_editor/pull/218>`_)
* Create AxisAlignedBox component for slotcar (`#227 <https://github.com/osrf/traffic_editor/pull/227>`_)
* Ignition plugins and modularization of doors and slotcar (`#138 <https://github.com/osrf/traffic_editor/pull/13>`_)
* Adding lift plugin for ignition (`#171 <https://github.com/osrf/traffic_editor/pull/17>`_)
* Contributors: Guoliang (Fred) Shao, Luca Della Vedova, Rushyendra Maganty, kevinskwk
