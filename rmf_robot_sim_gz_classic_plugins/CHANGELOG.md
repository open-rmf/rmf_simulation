## Changelog for package rmf\_robot\_sim\_gz\_classic\_plugins

2.0.0 (2022-XX-XX)
------------------
* Renamed to rmf\_robot\_sim\_gz\_classic\_plugins and Humble migration ([#77](https://github.com/open-rmf/rmf_simulation/pull/77))
* Checking for models with dispensable in names to not be considered an obstacle by slotcar [#74](https://github.com/open-rmf/rmf_simulation/pull/74)
* Contributors: Aaron Chong, Luca Della Vedova, Yadunund

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix [#100](https://github.com/open-rmf/rmf/issues/100). [#60](https://github.com/open-rmf/rmf_simulation/pull/60)
* Contributors: Marco A. Gutierrez

1.3.0 (2020-09-01)
------------------
* Change parameter tag from holonomic to steering ([#46](https://github.com/open-rmf/rmf_simulation/issues/46))
* Nonholonomic slotcar turning fixes, add turning multiplier offset parameter, rename compute\_ds variables ([#41](https://github.com/open-rmf/rmf_simulation/issues/41))
* fix typo on gazebo\_dev dependency ([#28](https://github.com/open-rmf/rmf_simulation/issues/28))
* Fix dependencies ([#26](https://github.com/open-rmf/rmf_simulation/issues/26))
* Changes and corrections to support ROS 2 Galactic ([#23](https://github.com/open-rmf/rmf_simulation/issues/23))
* Add quality declaration documents ([#1](https://github.com/open-rmf/rmf_simulation/issues/1))
* Add build and style actions ([#11](https://github.com/open-rmf/rmf_simulation/issues/11))
* Slotcar plugin package move and utils cleanup ([#5](https://github.com/open-rmf/rmf_simulation/issues/5))
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs ([#3](https://github.com/open-rmf/rmf_simulation/issues/3))
* Package renames ([#2](https://github.com/open-rmf/rmf_simulation/issues/2))
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, ddengster

1.1.0 (2020-09-23)
------------------
* Support cross-compiling with ROS 2 Foxy and Eloquent. [#103](https://github.com/osrf/rmf_demos/pull/103)
* TeleportIngestorPlugin uses rmf\_ingestor\_msgs. [#117](https://github.com/osrf/rmf_demos/pull/117)
* Modularized Readonly TeleportIngestorPlugin and TeleportDispenserPlugin to support both Gazebo and Ignition simulations. [#124, #134](https://github.com/osrf/rmf_demos/pull/124)
* Contributors: Aaron Chong, Boon Han, Michael X. Grey, Yadunund, mrushyendra

1.0.0 (2020-06-24)
------------------
* Provides the `readonly` Gazebo plugin that emulates the behavior of a `read\_only` fleet type. When attached to a steerable vehicle that operates on a known traffic graph, the plugin publishes a prediction of the vehicle\'s intended path to the `rmf traffic database` via its configured `read_only_fleet_adapter`. Other fleets operating in the space can plan routes to avoid the path of this vehicle.
* Provides `TeleportDispenser` Gazebo plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Gazebo along side a \"payload\" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestor` Gazebo plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Contributors: Aaron, Aaron Chong, Charayaphan Nakorn Boon Han, Luca Della Vedova, Yadunund
