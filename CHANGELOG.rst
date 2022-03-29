^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros1_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.3 (2022-03-29)
-------------------
* Cleanup of README.md (`#342 <https://github.com/ros2/ros1_bridge/issues/342>`_)
* Parametrizing service execution timeout (`#340 <https://github.com/ros2/ros1_bridge/issues/340>`_)
* Fix cpplint error (`#341 <https://github.com/ros2/ros1_bridge/issues/341>`_)
* Update package maintainers (`#335 <https://github.com/ros2/ros1_bridge/issues/335>`_)
* Contributors: Cem Karan, Geoffrey Biggs, Jorge Perez, Marco Bassa, Tim Clephas, Tomoya Fujita

0.10.2 (2021-11-05)
-------------------
* Example for `parameter_bridge` (`#330 <https://github.com/ros2/ros1_bridge/issues/330>`_)
* Use rcpputils/scope_exit.hpp instead of rclcpp/scope_exit.hpp (`#324 <https://github.com/ros2/ros1_bridge/issues/324>`_)
* Use FindPython3 and make Python dependency explicit (`#322 <https://github.com/ros2/ros1_bridge/issues/322>`_)
* Bump ros-tooling/setup-ros@v0.2 (`#323 <https://github.com/ros2/ros1_bridge/issues/323>`_)
* Add GitHub workflow for CI (`#310 <https://github.com/ros2/ros1_bridge/issues/310>`_)
* Update includes after rcutils/get_env.h deprecation (`#311 <https://github.com/ros2/ros1_bridge/issues/311>`_)
* Contributors: Christophe Bedard, Harsh Deshpande, Loy, Shane Loretz

0.10.1 (2021-01-25)
-------------------
* Fix logging for updated rclcpp interface (`#303 <https://github.com/ros2/ros1_bridge/issues/303>`_)
* Fix typo in comments (`#297 <https://github.com/ros2/ros1_bridge/issues/297>`_)
* Contributors: Michael Carroll, Vicidel

0.9.5 (2020-12-08)
------------------
* Update to use rosidl_parser and .idl files rather than rosidl_adapter and .msg files (`#296 <https://github.com/ros2/ros1_bridge/issues/296>`_)
* Update maintainers (`#286 <https://github.com/ros2/ros1_bridge/issues/286>`_)
* Contributors: Jacob Perron, William Woodall

0.9.4 (2020-09-10)
------------------
* use hardcoded QoS (keep all, transient local) for /tf_static topic in dynamic_bridge (`#282 <https://github.com/ros2/ros1_bridge/issues/282>`_)
* document explicitly passing the topic type to 'ros2 topic echo' (`#279 <https://github.com/ros2/ros1_bridge/issues/279>`_)

0.9.3 (2020-07-07)
------------------
* Fix multiple definition if message with same name as service exists (`#272 <https://github.com/ros2/ros1_bridge/issues/272>`_)
* Contributors: Dirk Thomas

0.9.2 (2020-06-01)
------------------
* When generating service mappings cast pair to set to avoid duplicate pairs (`#268 <https://github.com/ros2/ros1_bridge/issues/268>`_)
* Contributors: Gavin Suddrey

0.9.1 (2020-05-27)
------------------
* Deprecate package key for service parameters, use full type instead (`#263 <https://github.com/ros2/ros1_bridge/issues/263>`_)
* Fix removing obsolete ROS 2 service bridges (`#267 <https://github.com/ros2/ros1_bridge/issues/267>`_)
* Gracefully handle invalid ROS 1 publishers (`#266 <https://github.com/ros2/ros1_bridge/issues/266>`_)
* Use reliable publisher in simple bridge (`#264 <https://github.com/ros2/ros1_bridge/issues/264>`_)
* Remove outdated information on Fast RTPS bug (`#260 <https://github.com/ros2/ros1_bridge/issues/260>`_)
* Contributors: Dirk Thomas, Thom747

0.9.0 (2020-05-18)
------------------
* Avoid new deprecations (`#255 <https://github.com/ros2/ros1_bridge/issues/255>`_)
* Updates since changes to message_info in rclcpp (`#253 <https://github.com/ros2/ros1_bridge/issues/253>`_)
* Assert ROS 1 nodes' stdout (`#247 <https://github.com/ros2/ros1_bridge/issues/247>`_)
* Ignore actionlib_msgs deprecation warning (`#245 <https://github.com/ros2/ros1_bridge/issues/245>`_)
* Drop workaround for https://github.com/ros2/rmw_fastrtps/issues/265. (`#233 <https://github.com/ros2/ros1_bridge/issues/233>`_)
* Code style only: wrap after open parenthesis if not in one line (`#238 <https://github.com/ros2/ros1_bridge/issues/238>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo, William Woodall

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
