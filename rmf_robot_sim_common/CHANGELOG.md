## Changelog for package rmf\_robot\_sim\_common

2.0.0 (2022-XX-XX)
------------------
* Fix spurious emergency stop behavior when robot is moving backwards ([#82](https://github.com/open-rmf/rmf\_simulation/pull/82))
* Reduce terminal printouts from slotcar ([#73](https://github.com/open-rmf/rmf_simulation/pull/73))
* Change to use a pure pursuit controller for ackermann vehicles ([#71](https://github.com/open-rmf/rmf_simulation/pull/71))
* Update robot's current level only after the lift has finished moving ([#64](https://github.com/open-rmf/rmf_simulation/pull/64))
* Make ackermann vehicles standardized with RMF stack ([#49](https://github.com/open-rmf/rmf_simulation/pull/49))
* Change parameter tag from holonomic to steering ([#46](https://github.com/open-rmf/rmf_simulation/pull/46))
* Contributors: Grey, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Yadunund, chianfern, ddengster

1.3.0 (2021-09-01)
------------------
* support reversible slotcar ([#38](https://github.com/open-rmf/rmf_simulation/issues/38))
* Nonholonomic slotcar turning fixes, add turning multiplier offset parameter, rename compute\_ds variables ([#41](https://github.com/open-rmf/rmf_simulation/issues/41))
* Support nonholonomic movement for slotcar ([#33](https://github.com/open-rmf/rmf_simulation/issues/33))
* Fix/dependencies ([#26](https://github.com/open-rmf/rmf_simulation/issues/26))
* Changes and corrections to support ROS 2 Galactic ([#23](https://github.com/open-rmf/rmf_simulation/issues/23))
* Start using ros-tooling for build and test workflow, added coverage, tsan ([#14](https://github.com/open-rmf/rmf_simulation/issues/14))
* Add quality declaration documents ([#1](https://github.com/open-rmf/rmf_simulation/issues/1))
* Do not drain battery when robot is in the vicinity of the charger ([#18](https://github.com/open-rmf/rmf_simulation/issues/18))
* Skip RobotState if level\_name is empty ([#16](https://github.com/open-rmf/rmf_simulation/issues/16))
* Log as debug ([#12](https://github.com/open-rmf/rmf_simulation/issues/12))
* Add build and style actions ([#11](https://github.com/open-rmf/rmf_simulation/issues/11))
* Slotcar plugin package move and utils cleanup ([#5](https://github.com/open-rmf/rmf_simulation/issues/5))
* account for renaming from building\_map\_msgs to rmf\_building\_map\_msgs ([#3](https://github.com/open-rmf/rmf_simulation/issues/3))
* Package Renames ([#2](https://github.com/open-rmf/rmf_simulation/issues/2))
* Contributors: Aaron Chong, Charayaphan Nakorn Boon Han, Geoffrey Biggs, Luca Della Vedova, Marco A. Gutiérrez, Yadunund, chianfern, ddengster
