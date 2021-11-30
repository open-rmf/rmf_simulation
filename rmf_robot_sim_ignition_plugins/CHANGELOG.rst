^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_robot_sim_ignition_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2021-30-11)
------------------
* Making install location package specific to fix: `#100 <https://github.com/open-rmf/rmf/issues/100>`_
* Contributors: Marco A. Gutierrez

1.3.0 (2021-09-01)
------------------
* Make Ignition Edifice dependency mandatory to fix binary builds(`#45 <https://github.com/open-rmf/rmf_simulation/issues/45>`_)
* Change parameter tag from holonomic to steering (`#46 <https://github.com/open-rmf/rmf_simulation/issues/46>`_)
* Change slotcar control to explicitly open loop (`#43 <https://github.com/open-rmf/rmf_simulation/issues/43>`_)
* Nonholonomic slotcar turning fixes, add turning multiplier offset parameter, rename compute_ds variables (`#41 <https://github.com/open-rmf/rmf_simulation/issues/41>`_)
* Support nonholonomic movement for slotcar (`#33 <https://github.com/open-rmf/rmf_simulation/issues/33>`_)
* Skip updating ignition plugins when simulation is paused (`#34 <https://github.com/open-rmf/rmf_simulation/issues/34>`_)
* Fix/dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/issues/26>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/issues/1>`_)
* Add build and style actions (`#11 <https://github.com/open-rmf/rmf_simulation/issues/11>`_)
* Update to Ignition Edifice (`#8 <https://github.com/open-rmf/rmf_simulation/issues/8>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/issues/5>`_)
* account for renaming from building_map_msgs to rmf_building_map_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/issues/3>`_)
* Package renames (`#2 <https://github.com/open-rmf/rmf_simulation/issues/2>`_)
* Contributors: Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, ddengster

1.2.0 (2021-01-06)
------------------
* Provides `TeleportDispenserPlugin` Ignition plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Ignition Gazebo along side a "payload" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestorPlugin` Ignition plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Ignition Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_ingestor_msgs::IngestorRequest` message with `target_guid` matching its model name.
* Provides `ReadonlyPlugin` Readonly plugin that emulates the behavior of a `read_only` fleet type. When attached to a steerable vehicle that operates on a known traffic graph, the plugin publishes a prediction of the vehicle's intended path to the `rmf traffic database` via its configured `read_only_fleet_adapter`. Other fleets operating in the space can plan routes to avoid the path of this vehicle.
* Contributors: Aaron Chong, Luca Della Vedova, Yadunund, Rushyendra Maganty
