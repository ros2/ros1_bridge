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

0.11.0 (2023-02-22)
-------------------
* Tailor: Updating Jenkinsfile
* Tailor: Creating Jenkinsfile
* ROS1 message libraries don't need dynamic linkage
* 0.10.1
* Changelog.
* Fix logging for updated rclcpp interface (#303)
  * Fix logging for updated rclcpp interface
  * Update src/bridge.cpp
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
  * Update to pass char *
  * Uncrustify
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
* Fix typo in comments (#297)
  * very minor typo in comments
  * Fix other copy-paste typos.
  Co-authored-by: Steven! Ragnarök <nuclearsandwich@users.noreply.github.com>
  Co-authored-by: tomoya <Tomoya.Fujita@sony.com>
* 0.10.0
* fix bug with sizeof when type of the arrays differ (#298)
* 0.9.5
* [forward port] update to use rosidl_parser and .idl files rather than rosidl_adapter and .msg files (#296)
  * update to use rosidl_parser and .idl files rather than rosidl_adapter and .msg files
  * fix quote style linting error
* Update maintainers (#286)
* Contributors: Chris Lalancette, Jacob Perron, Michael Carroll, Paul Bovbel, Vicidel, William Woodall, locus-services

0.13.0 (2024-06-17)
-------------------
* Correctly handle fixed length array conversion in services and actions (#4)
* RST-6618 fixes for action bridge (#2)
  Fixed action_bridge changes to work with newer ros2 releases and added support for skipping previously built generated factories to speed up recompiling.
* Implemented action bridge
  Squash of upstream PR: https://github.com/ros2/ros1_bridge/pull/256
  Co-authored-by: tomoya <Tomoya.Fujita@sony.com>
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
* Contributors: Alex Bencz, Harsh Deshpande

0.12.0 (2023-09-25)
-------------------
* 0.11.0
* Update changelogs
* Improve bridge command parser (#1)
  * Refactor command parser to use vector of char*
  * Add argc argv splitter to dynamic bridge
  * Add argc argv splitter to paramter bridge
  * Update github action
  * Fix parameter bridge argv split
  * Remove static argv reading
  * Add command parser to parameter bridge
  * Fix compilation errors
  * Fix compilation errors
  * Fix parameter bidge command parser
  * Refactor to add command parser utils file
  * Add github action issue workaround
  * Ty to run github action inside container
  * Fix ROS2 args when no ROS1 args
  * ⏪ Revert changes on GitHub Action config
  * Change parameter bridge ROS init order
  * Rename parser functions
  * Add get_option_values function
  * Fix wrong help for the parameter bridge
  * Fix get_option_values parser function
  * Refactor bridges to use the get_option_values
  * Add running scetion to README
  * Add print pairs to parameter bridge
  ---------
  Co-authored-by: LucasHaug <lucas.haug@hotmail.com>
* Tailor: Creating Jenkinsfile
* Improved gitignore to exclude build files and similar editor-based tools (#392)
* Provide direct serialization of ROS2 messsage to ROS1 streams (#381)
  * Generated functions for (de)serializing ROS2 messages to ROS1 streams.
  * Partial set of fixes from review.
  * Rename functions based on discussion from review
  * Whitespace fix.
  * Name change for conversion helper fucntion.
  * Use explicit function overloading instead of template for stream types.
  * Fix factory.hpp crustify errors.
* [master] Update maintainers - 2022-11-07 (#386)
* Apply automatic mapping rules in case only package+message mapping exists (#382)
  * Fix message mapping by removing early return so other rules can still be applied
  In determine_field_mapping, there was an early return inside a loop over all mapping rules.
  IF there we any mapping rules but they don't specify field mappings, the early return made the function return without creating mappings automatically.
  For a particular message type, ROS 1's uuid_msgs/UniqueID vs ROS 2's unique_identifier_msgs/UUID, the message definition is exacly the same but type name is not.
  The only mapping fule defined in for unique_identifier_msgs/UUID is that it maps to uuid_msgs/UniqueID, but no field mappings are needed because the definitions are the same.
  But, then we hit the early return (because the for-loop is ran without any rule applying to the message at hand and thus not `continue`-ing in a code branch handling a rule)
  and return without applying the normal automatic field mapping generation rules.
  By removing the early return, the other rules are applied and the mapping rules for handling the exact same message defintions are applied
  * Account for fields mapped by rules when checking for missed fields
  The code after the early return mentioned in the previous commit assumed all fields would match by name,
  which was of course true. But not anymore, so the missing check now only fails when the missing fields are also not already accounted for via a mapping
  * Fix flake8 violations
* New function for converting between ROS1 and ROS2 generic message formats. (#377)
* Fix typo (`services_1_or_2` -> `services_1_to_2`) (#379)
* Move xmlrpcpp find_package so it only searches if ROS 1 is found (#371)
* Implement Foreign Mapping Check Override (#367)
* removing unused slot
* Parametrize Quality of Service in `parameter_bridge`.  (#331)
  * Extend create_bidirectional_bridge to take qos param for ROS2 publisher
  * Busy setting up a way to read QoS parameters from ROS1 params
  * Parse history qos params
  * Call qos_from_params when setting up topics
  * Configure deadline, lifespan, liveliness_lease_durations
  * Configure liveliness
  * Add some basic debug text
  * Print the QoS settings to stdout when setting them up
  * Catch XmlRpc::XmlRpcExceptions when constructing QoS from parameters
  * Parse liveliness as either int enum value or upper/lower case string representation of liveliness enum values
  * Fix formatting with uncrustify
  * Fix cpplint formatting
  In 2 cases the formatting conflicts with what uncrustify wants; // NOLINT got rid of the complaints of cpplint, favouring uncrustify
  * Clearer logging as suggested by code review
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Clarify keep_last vs keep_all setting for history
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
* updated description
* fixed ros2 message index
* added fields_2_to_1 to print message mapping
* Clarify example 4 (#362)
* Fix array static_assertion with newer compilers (#363)
* to correct Examples 4 (#357)
* reverting authors
* updating maintainers
* Update maintainers (#360)
* Suppress warning when packages aren't available (#355)
  * Suppress warning when packages aren't available
  ROS1 packages aren't available on all platforms.  In order to keep
  ros1_bridge in ros2.repos without warnings turning CI yellow, we need to
  suppress this warning.
* Make sure to use fully-qualified boost placeholders. (#351)
  Also make sure to declare a direct dependency on boost.
  While we are always getting it from ROS 1, this code directly
  uses it and so should also declare a dependency.
* 0.10.3
* Changelog
* Cleanup of README.md (#342)
  * Reformatted README.md
  This is an initial cleanup attempt of the README.md file.  All
  non-code lines have been wrapped to an 80 column width, and some
  markdown was cleaned up.  Further work is needed.
  * Added 'bash' info string to appropriate fenced code blocks.
  Numerous fenced code blocks did not have a
  [info string](https://github.github.com/gfm/#info-string), which
  meant that the enclosed code block wouldn't be rendered correctly.
  As it turned out, all of these were intended to be run in a bash
  shell, so I added that as their info strings.
  * WIP: Updated the scripts in the README.md file to use environment variables.
  The scripts weren't directly executable before, requiring manual
  substitution of various 'variables'.  This was unfortunate as Github
  makes copying and pasting code blocks simple and straightforward.
  The changes in this commit make it possible to copy/paste each example
  into it's own shell, which should make it slightly easier for new users
  to start using the bridge.
  * fix: Unwrapped lines per request from @gbiggs.
  @gbiggs commented at https://github.com/ros2/ros1_bridge/pull/342#issuecomment-1029508012
  that files in ROS2 are intentionally not wrapped to make handling
  diffs in PRs easier.  I've copied in the relevant original lines
  to unwrap the lines I wrapped earlier.
  * Update README.md
  Accepted suggestion from @gbiggs
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * Update README.md
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  * The service name might provide more debug information than host:port details (#346)
  * The service name might provide more debug information than host:port details
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
  * fix: Changed 'galactic' to 'rolling'
  Changes due to suggestions from @gbiggs in the discussion of [PR 342](https://github.com/ros2/ros1_bridge/pull/342#discussion_r834287510)
  Co-authored-by: Geoffrey Biggs <gbiggs@killbots.net>
  Co-authored-by: Tim Clephas <tim.clephas@nobleo.nl>
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* The service name might provide more debug information than host:port details (#346)
  * The service name might provide more debug information than host:port details
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* Parametrizing service execution timeout (#340)
  * Parametrizing service execution timeout
* Fix cpplint error (#341)
  * Fix cpplint error
* Update package maintainers (#335)
  * Update package maintainers
* 0.10.2
* Changelog.
* Example for `parameter_bridge` (#330)
  * Add example for using the parameter_bridge
* Use rcpputils/scope_exit.hpp instead of rclcpp/scope_exit.hpp (#324)
* Use FindPython3 and make Python dependency explicit (#322)
* Bump ros-tooling/setup-ros@v0.2 (#323)
  See if that fixes the apt-update part of CI
* Add GitHub workflow for CI (#310)
  * init docker based CI
  * target rolling only
  * use setup-ros and action-ros-ci instead of custom scripts
  * quiet blind except warnings
  https://github.com/ros2/ros1_bridge/pull/310#discussion_r621492261
  * remove ccache
  build times are not a concern and it is not significantly improved when using ros-tooling actions
* Update includes after rcutils/get_env.h deprecation (#311)
* 0.10.1
* Changelog.
* Fix logging for updated rclcpp interface (#303)
  * Fix logging for updated rclcpp interface
  * Update src/bridge.cpp
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
  * Update to pass char *
  * Uncrustify
  Co-authored-by: Ivan Santiago Paunovic <ivanpauno@ekumenlabs.com>
* Fix typo in comments (#297)
  * very minor typo in comments
  * Fix other copy-paste typos.
  Co-authored-by: Steven! Ragnarök <nuclearsandwich@users.noreply.github.com>
  Co-authored-by: tomoya <Tomoya.Fujita@sony.com>
* 0.10.0
* fix bug with sizeof when type of the arrays differ (#298)
* 0.9.5
* [forward port] update to use rosidl_parser and .idl files rather than rosidl_adapter and .msg files (#296)
  * update to use rosidl_parser and .idl files rather than rosidl_adapter and .msg files
  * fix quote style linting error
* Update maintainers (#286)
* Contributors: Andrej Orsula, Audrow Nash, Cem Karan, Chris Lalancette, Christophe Bedard, Derek, Dharini Dutia, Gary Servin, Geoffrey Biggs, Harsh Deshpande, Jacob Perron, Jorge Perez, Loy, Loy van Beek, Marco Bassa, Michael Carroll, Nick Sims, Paul Bovbel, Shane Loretz, Tim Clephas, Vicidel, William Woodall, locus-services, methylDragon, quarkytale, xlla

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
