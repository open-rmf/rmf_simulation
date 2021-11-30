^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_building_sim_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix: `#100 <https://github.com/open-rmf/rmf/issues/100>`_
* Contributors: Marco A. Gutierrez

1.3.0 (2021-09-01)
------------------
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/issues/26>`_)
* fix compile error on qt > 5.14 (`#27 <https://github.com/open-rmf/rmf_simulation/issues/27>`_)
* Changes for galactic (`#25 <https://github.com/open-rmf/rmf_simulation/issues/25>`_)
* Make menge mandatory and fix building_sim dependencies (`#19 <https://github.com/open-rmf/rmf_simulation/issues/19>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/issues/1>`_)
* Crowd step size fix for large physics step sizes (`#10 <https://github.com/open-rmf/rmf_simulation/issues/10>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/issues/5>`_)
* account for renaming to rmf_building_map_tools (`#4 <https://github.com/open-rmf/rmf_simulation/issues/4>`_)
* account for renaming from building_map_msgs to rmf_building_map_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/issues/3>`_)
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Teo Koon Peng, Yadu

1.2.0 (2021-01-06)
------------------
* Merge branch 'release-1.1'
* Control slotcar with model velocity cmds in place of joint velocity cmds (`#236 <https://github.com/osrf/traffic_editor/issues/236>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/issues/242>`_)
* Implement animation switching in crowd simulation (`#238 <https://github.com/osrf/traffic_editor/issues/238>`_)
* Building_crowdsim for generating the navmesh file and required configuration files for menge (`#224 <https://github.com/osrf/traffic_editor/issues/224>`_)
* Contributors: Geoffrey Biggs, Guoliang (Fred) Shao, Marco A. Gutierrez, Marco A. Gutiérrez, Rushyendra Maganty

* Add animation switch to crowd simulation plugin (`#238 <https://github.com/osrf/traffic_editor/pull/238>`_)

1.1.0 (2020-09-24)
------------------
* Implement model visibility toggling (`#226 <https://github.com/osrf/traffic_editor/issues/226>`_)
* Add crowd simulation plugin (`#218 <https://github.com/osrf/traffic_editor/issues/218>`_)
* Foxy support (`#194 <https://github.com/osrf/traffic_editor/issues/194>`_)
* Add field in lift dialog for initial floor, handle invalid initial floor
* Modularization of doors and slotcar `#138 <https://github.com/osrf/traffic_editor/issues/138>`_
* Add gazebo lift plugin
* Contributors: Charayaphan Nakorn Boon Han, Guoliang (Fred) Shao, Kevin_Skywalker, Luca Della Vedova, MakinoharaShouko, Yadu

1.0.0 (2020-06-22)
------------------
* Merge pull request `#151 <https://github.com/osrf/traffic_editor/issues/151>`_ from osrf/fix/slotcar_level_name
  Slotcar update
* Level name inferred from elevation. Set publish rate to 2hz.
* merging master
* Merge pull request `#139 <https://github.com/osrf/traffic_editor/issues/139>`_ from osrf/feature/adapter_error
  Report when a new path request is out of date
* Report when a new path request is out of date
* Merge pull request `#119 <https://github.com/osrf/traffic_editor/issues/119>`_ from osrf/fix/slotcar_level_name
  Level name populated in Slotcar RobotState msg
* Cleanup
* Level name populated in RobotState msg
* Merge pull request `#95 <https://github.com/osrf/traffic_editor/issues/95>`_ from osrf/fix/plugin-link-missing
  changed to using target_link_libraries to be specific
* changed to using target_link_libraries to be specific
* Merge pull request `#90 <https://github.com/osrf/traffic_editor/issues/90>`_ from osrf/feature/single-doors
  Feature/single doors
* WIP open/close positions flipped at -90 and -1
* migrated single door changes to plugin
* Merge branch 'master' into feature/single-doors
* Merge pull request `#89 <https://github.com/osrf/traffic_editor/issues/89>`_ from osrf/add_gazebo_plugins
  add gazebo plugins used by rmf_building_map_tools generators
* add gazebo plugins used by rmf_building_map_tools generators
* Contributors: Aaron, Aaron Chong, Michael X. Grey, Morgan Quigley, Yadu, Yadunund
