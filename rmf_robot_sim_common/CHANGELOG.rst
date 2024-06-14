^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf\_robot\_sim\_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.2 (2024-06-15)
------------------

2.3.1 (2024-06-12)
------------------

2.3.0 (2024-06-01)
------------------
* Port outdated actions to Noble (`#122 <https://github.com/open-rmf/rmf_simulation/pull/122>`_)
* Explicitly specify all qos depth (`#116 <https://github.com/open-rmf/rmf_simulation/pull/116>`_)
* Contributors: Luca Della Vedova, Teo Koon Peng

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
* Fix eigen not found when building rpm (`#102 <https://github.com/open-rmf/rmf_simulation/pull/102>`_)
* Fix library linking (`#97 <https://github.com/open-rmf/rmf_simulation/pull/97>`_)
* Contributors: Esteban Martinena Guerrero, Grey, Yadunund

2.0.0 (2022-10-04)
------------------
* Fix spurious emergency stop behavior when robot is moving backwards (`#82 <https://github.com/open-rmf/rmf\_simulation/pull/82>`_)
* Reduce terminal printouts from slotcar (`#73 <https://github.com/open-rmf/rmf_simulation/pull/73>`_)
* Change to use a pure pursuit controller for ackermann vehicles (`#71 <https://github.com/open-rmf/rmf_simulation/pull/71>`_)
* Update robot's current level only after the lift has finished moving (`#64 <https://github.com/open-rmf/rmf_simulation/pull/64>`_)
* Make ackermann vehicles standardized with RMF stack (`#49 <https://github.com/open-rmf/rmf_simulation/pull/49>`_)
* Change parameter tag from holonomic to steering (`#46 <https://github.com/open-rmf/rmf_simulation/pull/46>`_)
* Contributors: Grey, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Yadunund, chianfern, ddengster

1.3.0 (2021-09-01)
------------------
* support reversible slotcar (`#38 <https://github.com/open-rmf/rmf_simulation/pull/38>`_)
* Nonholonomic slotcar turning fixes, add turning multiplier offset parameter, rename compute\_ds variables (`#41 <https://github.com/open-rmf/rmf_simulation/pull/41>`_)
* Support nonholonomic movement for slotcar (`#33 <https://github.com/open-rmf/rmf_simulation/pull/33>`_)
* Fix/dependencies (`#26 <https://github.com/open-rmf/rmf_simulation/pull/26>`_)
* Changes and corrections to support ROS 2 Galactic (`#23 <https://github.com/open-rmf/rmf_simulation/pull/23>`_)
* Start using ros-tooling for build and test workflow, added coverage, tsan (`#14 <https://github.com/open-rmf/rmf_simulation/pull/14>`_)
* Add quality declaration documents (`#1 <https://github.com/open-rmf/rmf_simulation/pull/1>`_)
* Do not drain battery when robot is in the vicinity of the charger (`#18 <https://github.com/open-rmf/rmf_simulation/pull/18>`_)
* Skip RobotState if level\_name is empty (`#16 <https://github.com/open-rmf/rmf_simulation/pull/16>`_)
* Log as debug (`#12 <https://github.com/open-rmf/rmf_simulation/pull/12>`_)
* Add build and style actions (`#11 <https://github.com/open-rmf/rmf_simulation/pull/11>`_)
* Slotcar plugin package move and utils cleanup (`#5 <https://github.com/open-rmf/rmf_simulation/pull/5>`_)
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs (`#3 <https://github.com/open-rmf/rmf_simulation/pull/3>`_)
* Package Renames (`#2 <https://github.com/open-rmf/rmf_simulation/pull/2>`_)
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Yadunund, chianfern, ddengster
