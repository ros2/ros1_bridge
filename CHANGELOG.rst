^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.2 (2020-01-17)
------------------
* fix building test when ROS 1 diagnostic_msgs is isolated from roscpp (`#236 <https://github.com/ros2/ros1_bridge/issues/236>`_)
* fix service with custom mapped message field (`#234 <https://github.com/ros2/ros1_bridge/issues/234>`_)
* Contributors: Dirk Thomas

0.8.1 (2019-10-23)
------------------
* fix showing duplicate keys in --print-pairs (`#225 <https://github.com/ros2/ros1_bridge/issues/225>`_)
* fix bridging builtin_interfaces Duration and Time (`#224 <https://github.com/ros2/ros1_bridge/issues/224>`_)
* Don't use features that will be deprecated (`#222 <https://github.com/ros2/ros1_bridge/issues/222>`_)
* Contributors: Dirk Thomas, Peter Baughman

0.8.0 (2019-09-27)
------------------
* Promote special CLI rules to flags. (`#217 <https://github.com/ros2/ros1_bridge/issues/217>`_)
* Update __log_rosout_disable workaround to use --ros-args. (`#216 <https://github.com/ros2/ros1_bridge/issues/216>`_)
* Clearer instructions for example (`#211 <https://github.com/ros2/ros1_bridge/issues/211>`_)
* add services bridging to parameter_bridge (`#176 <https://github.com/ros2/ros1_bridge/issues/176>`_)
* Contributors: Jose Luis Blanco-Claraco, Michel Hidalgo, cyrilleberger

0.7.3 (2019-08-02)
------------------
* fix typename in static bridge (`#209 <https://github.com/ros2/ros1_bridge/issues/209>`_)
* fix cosmetic in message (`#207 <https://github.com/ros2/ros1_bridge/issues/207>`_)
* Use %zu print format for size_t (`#204 <https://github.com/ros2/ros1_bridge/issues/204>`_)
* Fix parameter bridge for topic if ros1 and ros2 type have a different name (`#177 <https://github.com/ros2/ros1_bridge/issues/177>`_)
* Contributors: Dirk Thomas, Emerson Knapp, cyrilleberger

0.7.2 (2019-05-29)
------------------
* add note about rostopic echo (`#202 <https://github.com/ros2/ros1_bridge/issues/202>`_)
* add workspace setup documentation (`#201 <https://github.com/ros2/ros1_bridge/issues/201>`_)
* Contributors: Mabel Zhang

0.7.1 (2019-05-20)
------------------
* Disable rosout logging for the bridge (`#197 <https://github.com/ros2/ros1_bridge/issues/197>`_)
* Handle launch_testing assertExitCodes correctly (`#193 <https://github.com/ros2/ros1_bridge/issues/193>`_)
* Support field selection  (`#174 <https://github.com/ros2/ros1_bridge/issues/174>`_)
* Use interface kind names properly in ROS2 interface type names. (`#194 <https://github.com/ros2/ros1_bridge/issues/194>`_)
* Contributors: Juan Rodriguez Hortala, Michel Hidalgo, ivanpauno

0.7.0 (2019-05-08)
------------------
* Adds interface type to ROS2 message type name. (`#191 <https://github.com/ros2/ros1_bridge/issues/191>`_)
* fix build by passing options (`#192 <https://github.com/ros2/ros1_bridge/issues/192>`_)
* changes to avoid deprecated API's (`#189 <https://github.com/ros2/ros1_bridge/issues/189>`_)
* Corrected publish calls with shared_ptr signature, leftovers (`#190 <https://github.com/ros2/ros1_bridge/issues/190>`_)
* Corrected publish calls with shared_ptr signature (`#188 <https://github.com/ros2/ros1_bridge/issues/188>`_)
* Migrate launch tests to new launch_testing features & API (`#179 <https://github.com/ros2/ros1_bridge/issues/179>`_)
* Some small fixes to the README (`#186 <https://github.com/ros2/ros1_bridge/issues/186>`_)
* Fix the generator. (`#185 <https://github.com/ros2/ros1_bridge/issues/185>`_)
* Merge pull request `#183 <https://github.com/ros2/ros1_bridge/issues/183>`_ from ros2/interface_specific_compilation_units
* remove note about memory usage from README
* split into interface specific compilation units
* duplicate template before modifying it to track history
* fix log messages (`#182 <https://github.com/ros2/ros1_bridge/issues/182>`_)
* use safe_load instead of deprecated load (`#180 <https://github.com/ros2/ros1_bridge/issues/180>`_)
* Merge pull request `#178 <https://github.com/ros2/ros1_bridge/issues/178>`_ from ros2/gonzalodepedro/fix-propagate-args-to-rcl-init
* Allows propagations of cmd args to rclcpp::init
* add section about DCO to CONTRIBUTING.md
* Add launch along with launch_testing as test dependencies. (`#171 <https://github.com/ros2/ros1_bridge/issues/171>`_)
* Switch to rclcpp logging and improve messages (`#167 <https://github.com/ros2/ros1_bridge/issues/167>`_)
* invalidate wrong cached result for diagnostic_msgs (`#170 <https://github.com/ros2/ros1_bridge/issues/170>`_)
* Drops legacy launch API usage. (`#163 <https://github.com/ros2/ros1_bridge/issues/163>`_)
* export find_ros1_package cmake (`#164 <https://github.com/ros2/ros1_bridge/issues/164>`_)
* ensure that the diagnostic_msgs package is from ROS 2 (`#169 <https://github.com/ros2/ros1_bridge/issues/169>`_)
* Allow latching for ROS1 pub, and custom qos for ROS2 components (`#162 <https://github.com/ros2/ros1_bridge/issues/162>`_)
* Allow external use of ros1_bridge library factories (`#160 <https://github.com/ros2/ros1_bridge/issues/160>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Gonzalo de Pedro, Gonzo, Karsten Knese, Michel Hidalgo, Mikael Arguedas, Paul Bovbel, William Woodall, ivanpauno

0.6.1 (2018-12-12)
------------------
* exclude ros1 nodelets (`#152 <https://github.com/ros2/ros1_bridge/issues/152>`_)
* fix is_package_mapping check (`#151 <https://github.com/ros2/ros1_bridge/issues/151>`_)
* Contributors: Dirk Thomas, Karsten Knese

0.6.0 (2018-12-08)
------------------
* expose convert function (`#146 <https://github.com/ros2/ros1_bridge/issues/146>`_)
* support for custom field mapping for services (`#147 <https://github.com/ros2/ros1_bridge/issues/147>`_)
* handle idl files correctly (`#145 <https://github.com/ros2/ros1_bridge/issues/145>`_)
* Fix for actions subfolder introduction in ros2 message bridge (`#143 <https://github.com/ros2/ros1_bridge/issues/143>`_)
* use new error handling API from rcutils (`#141 <https://github.com/ros2/ros1_bridge/issues/141>`_)
* changed cmake message logger level (`#138 <https://github.com/ros2/ros1_bridge/issues/138>`_)
* Contributors: Alberto Soragna, Dirk Thomas, Karsten Knese, Samuel Servulo, William Woodall

0.5.1 (2018-08-20)
------------------
* Merge pull request `#136 <https://github.com/ros2/ros1_bridge/issues/136>`_ from ros2/update_docs_135
* update doc to reflect that any mapping combination is supported
* rule can be a message mapping even if a field mapping is provided as well (`#135 <https://github.com/ros2/ros1_bridge/issues/135>`_)
* Contributors: Mikael Arguedas

0.5.0 (2018-06-27)
------------------
* remove --build-tests which is an ament argument from colcon invocation
* print service pairs as well (`#124 <https://github.com/ros2/ros1_bridge/issues/124>`_)
* print message for all ROS 2 message pkgs (`#123 <https://github.com/ros2/ros1_bridge/issues/123>`_)
* update README to use colcon and ROS Melodic (`#122 <https://github.com/ros2/ros1_bridge/issues/122>`_)
* include module name which wasn't found in error message (`#121 <https://github.com/ros2/ros1_bridge/issues/121>`_)
* use catkin_pkg to parse packages (`#119 <https://github.com/ros2/ros1_bridge/issues/119>`_)
* migrate launch -> launch.legacy (`#117 <https://github.com/ros2/ros1_bridge/issues/117>`_)
* Duplicate messages in bidirectional_bridge fix (`#113 <https://github.com/ros2/ros1_bridge/issues/113>`_)
* Fix linter failures from includes (`#110 <https://github.com/ros2/ros1_bridge/issues/110>`_)
* Map duration and time messages (`#106 <https://github.com/ros2/ros1_bridge/issues/106>`_)
* clarify that all field must be listed explicitly (`#109 <https://github.com/ros2/ros1_bridge/issues/109>`_)
* add an error message if the mapping rules are not a list (`#107 <https://github.com/ros2/ros1_bridge/issues/107>`_)
* advise to ask questions on ROS answers
* Contributors: ArkadiuszNiemiec, Dirk Thomas, Mikael Arguedas, Tully Foote, William Woodall, dhood

0.4.0 (2017-12-08)
------------------
* match topic name printed in console (`#102 <https://github.com/ros2/ros1_bridge/issues/102>`_)
* Update for rclcpp namespace removals (`#101 <https://github.com/ros2/ros1_bridge/issues/101>`_)
* cmake 3.10 compatibility: pass absolute path to file(GENERATE) function (`#100 <https://github.com/ros2/ros1_bridge/issues/100>`_)
* depend on rosidl_interfaces_packages group (`#99 <https://github.com/ros2/ros1_bridge/issues/99>`_)
* Fix building of ros1_bridge against newer roscpp. (`#98 <https://github.com/ros2/ros1_bridge/issues/98>`_)
* Merge pull request `#97 <https://github.com/ros2/ros1_bridge/issues/97>`_ from ros2/ament_cmake_pytest
* use ament_cmake_pytest instead of ament_cmake_nose
* Merge pull request `#96 <https://github.com/ros2/ros1_bridge/issues/96>`_ from ros2/print_type_names
* print bridged type names
* Increase timeout waiting for server for ros2 client in tests (`#94 <https://github.com/ros2/ros1_bridge/issues/94>`_)
* update style to match latest uncrustify (`#93 <https://github.com/ros2/ros1_bridge/issues/93>`_)
* Contributors: Brian Gerkey, Chris Lalancette, Dirk Thomas, Esteve Fernandez, Hunter Allen, Jackie Kay, Karsten Knese, Mikael Arguedas, Morgan Quigley, Rafal Kozik, Rafał Kozik, Steven! Ragnarök, Tully Foote, William Woodall, dhood, gerkey
