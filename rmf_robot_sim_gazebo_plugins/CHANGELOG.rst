^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_robot_sim_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix: `#100 <https://github.com/open-rmf/rmf/issues/100>`_
* Contributors: Marco A. Gutierrez

1.3.0 (2020-09-01)
------------------
* Change parameter tag from holonomic to steering (`#46 <https://github.com/open-rmf/rmf_simulation/issues/46>`_)
* Nonholonomic slotcar turning fixes, add turning multiplier offset parameter, rename compute_ds variables (`#41 <https://github.com/open-rmf/rmf_simulation/issues/41>`_)
* fix typo on gazebo_dev dependency (`#28 <https://github.com/open-rmf/rmf_simulation/issues/28>`_)
* Fix dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/issues/26>`_)
* Changes and corrections to support ROS 2 Galactic (`#23 <https://github.com/open-rmf/rmf_simulation/issues/23>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/issues/1>`_)
* Add build and style actions (`#11 <https://github.com/open-rmf/rmf_simulation/issues/11>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/issues/5>`_)
* account for renaming from building_map_msgs to rmf_building_map_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/issues/3>`_)
* Package renames (`#2 <https://github.com/open-rmf/rmf_simulation/issues/2>`_)
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, ddengster

1.1.0 (2020-09-23)
------------------
* Support cross-compiling with ROS 2 Foxy and Eloquent. [#103](https://github.com/osrf/rmf_demos/pull/103)
* TeleportIngestorPlugin uses rmf_ingestor_msgs. [#117](https://github.com/osrf/rmf_demos/pull/117)
* Modularized Readonly TeleportIngestorPlugin and TeleportDispenserPlugin to support both Gazebo and Ignition simulations. [#124, #134](https://github.com/osrf/rmf_demos/pull/124) 
* Contributors: Aaron Chong, Boon Han, Michael X. Grey, Yadu, mrushyendra

1.0.0 (2020-06-24)
------------------
* Provides the `readonly` Gazebo plugin that emulates the behavior of a `read_only` fleet type. When attached to a steerable vehicle that operates on a known traffic graph, the plugin publishes a prediction of the vehicle's intended path to the `rmf traffic database` via its configured `read_only_fleet_adapter`. Other fleets operating in the space can plan routes to avoid the path of this vehicle.
* Provides `TeleportDispenser` Gazebo plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Gazebo along side a "payload" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestor` Gazebo plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Luca Della Vedova, Yadu, Yadunund
