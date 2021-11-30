^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_building_sim_ignition_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix: `#100 <https://github.com/open-rmf/rmf/issues/100>`_
* Contributors: Marco A. Gutierrez

1.3.0 (2021-09-01)
------------------
* Added rosdep key for ignition-edifice, made it a mandatory dependency (`#31 <https://github.com/open-rmf/rmf_simulation/issues/31>`_)
* Skip updating ignition plugins when simulation is paused (`#34 <https://github.com/open-rmf/rmf_simulation/issues/34>`_)
* updating for galactic and rolling compatibility (`#29 <https://github.com/open-rmf/rmf_simulation/issues/29>`_)
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/issues/26>`_)
* Make menge mandatory and fix building_sim dependencies (`#19 <https://github.com/open-rmf/rmf_simulation/issues/19>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/issues/1>`_)
* Crowd step size fix for large physics step sizes (`#10 <https://github.com/open-rmf/rmf_simulation/issues/10>`_)
* Update to Ignition Edifice (`#8 <https://github.com/open-rmf/rmf_simulation/issues/8>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/issues/5>`_)
* account for renaming from building_map_msgs to rmf_building_map_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/issues/3>`_)
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez

1.2.0 (2021-01-06)
------------------
* Remove Slotcar/Lift AABB component when not required to speed up demos (`#271 <https://github.com/osrf/traffic_editor/issues/271>`_)
* Migrate ignition plugins to ament_target_dependencies (`#262 <https://github.com/osrf/traffic_editor/issues/262>`_)
* Enable lifts to work with TPE (`#250 <https://github.com/osrf/traffic_editor/issues/250>`_)
  Modifies lift plugin to issue either joint or model velocity commands
  based on the physics engine used.
* Control slotcar with model velocity cmds in place of joint velocity cmds (`#236 <https://github.com/osrf/traffic_editor/issues/236>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/issues/242>`_)
* Implement animation switching in crowd simulation (`#238 <https://github.com/osrf/traffic_editor/issues/238>`_)
* Add first pass of quality declarations for all packages (`#235 <https://github.com/osrf/traffic_editor/issues/235>`_)
* Building_crowdsim for generating the navmesh file and required configuration files for menge (`#224 <https://github.com/osrf/traffic_editor/issues/224>`_)
* Add animation switch to crowd simulation plugin (`#238 <https://github.com/osrf/traffic_editor/pull/238>`_)
* Contributors: Geoffrey Biggs, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutierrez, Marco A. Gutiérrez, Rushyendra Maganty

1.0.0 (2020-09-24)
------------------
* Focal / Ignition dome dependencies update (`#230 <https://github.com/osrf/traffic_editor/issues/230>`_)
* Ignition crowd simulation plugin (`#218 <https://github.com/osrf/traffic_editor/issues/218>`_)
* Create AxisAlignedBox component for slotcar (`#227 <https://github.com/osrf/traffic_editor/issues/227>`_)
* Ignition plugins and modularization of doors and slotcar `#138 <https://github.com/osrf/traffic_editor/issues/138>`_
* Adding lift plugin for ignition `#171 <https://github.com/osrf/traffic_editor/issues/171>`_
* Contributors: Guoliang (Fred) Shao, Luca Della Vedova, Rushyendra Maganty, kevinskwk
