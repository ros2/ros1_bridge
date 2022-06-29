ros1_bridge
===========

The bridge is a "dual homed" program which acts as a ROS 1 node as well as a ROS 2 node at the same time.
It can therefore subscribe to messages in one ROS version and publish them into the other ROS version.

The specific messages it is able to convert between ROS 1 and ROS 2 are determined at compile time.
Automatic mapping rules are being applied based on the names.
Additionally custom mapping rules can be provided.


How are ROS 1 and 2 messages associated with each other?
--------------------------------------------------------

Automatic mapping between ROS 1 and 2 messages is performed based on the names.

In the first step ROS 1 packages ending with ``_msgs`` are associated with ROS 2 packages ending in either ``_msgs`` or ``_interfaces``.

In the second step messages with the same name are associated with each other.

In the third step fields are associated with each other.
If one of the two associated messages has fields which is are not part of the other message they are being ignored.
If both messages have fields the other message does not contain it is assumed that the mapping is incomplete and no association is established.

If the package name, message name, and all field names and types are the same between a ROS 1 package and a ROS 2 package, and they satisfy the package suffix condition above, then the messages will automatically be associated with each other without additional specification.
The bridge needs to be recompiled, after the ROS 1 and ROS 2 packages have been compiled and sourced, each in a separate workspace, for a total of three workspaces.

How can I specify custom mapping rule for messages?
---------------------------------------------------

Additional mapping rules can be provided by a ROS 2 package using a ``yaml`` file.
Each mapping rule can have one of three types:

1. A package mapping rule is defined by:

   - a ``ros1_package_name``
   - a ``ros2_package_name`` (which, by default, must be the same as the ROS 2 package this mapping rule is defined in)

2. A message mapping rule is defined by the attributes of a package mapping rule and:

   - a ``ros1_message_name``
   - a ``ros2_message_name``

3. A field mapping rule is defined by the attributes of a message mapping rule and:

   - a dictionary ``fields_1_to_2`` mapping ROS 1 field selections to ROS 2 field selections.
     A field selection is a sequence of field names separated by ``.``, that specifies the path to a field starting from a message.
     For example starting from the message ``rosgraph_msgs/Log`` the field selection ``header.stamp`` specifies a
     path that goes through the field ``header`` of ``rosgraph_msgs/Log``, that has a message of type ``std_msgs/Header``,
     and ending up in the field ``stamp`` of ``std_msgs/Header``, that has type ``time``.
     All fields must be listed explicitly - not listed fields are not mapped implicitly when their names match.

Each mapping rule file contains a list of mapping rules.

How are ROS 1 and 2 services associated with each other?
--------------------------------------------------------

Automatic mapping between ROS 1 and 2 services is performed similar to messages.
Except that currently different field names are not supported.

How can I specify custom mapping rule for services?
---------------------------------------------------

In case of services, each mapping rule can have one of two types:

1. A package mapping rule is defined by:

   - a ``ros1_package_name``
   - a ``ros2_package_name`` (which, by default, must be the same as the ROS 2 package this mapping rule is defined in)

2. A service name mapping rule is defined by the attributes of a package mapping rule and:

   - a ``ros1_service_name``
   - a ``ros2_service_name``

A custom field mapping is currently not supported for services.

How can I install mapping rule files?
-------------------------------------

The mapping rule files must be exported in the ROS 2 package's ``package.xml`` in order to be processed by this package::

    <export>
      <ros1_bridge mapping_rules="my_mapping_rules.yaml"/>
    </export>

The yaml files must also be installed in the ROS 2 package's ``CMakeLists.txt``::

    install(
      FILES my_mapping_rules.yaml
      DESTINATION share/${PROJECT_NAME})


Example mapping rules file
--------------------------

Example package mapping rule matching all messages with the same names and fields within a pair of packages::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros2_package_name: 'ros2_pkg_name'

Example message mapping rule matching a specific pair of messages with the same fields::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros1_message_name: 'ros1_msg_name'
      ros2_package_name: 'ros2_pkg_name'
      ros2_message_name: 'ros2_msg_name'

Example message field mapping rule a specific pair of messages with a custom mapping between the fields::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros1_message_name: 'ros1_msg_name'
      ros2_package_name: 'ros2_pkg_name'
      ros2_message_name: 'ros2_msg_name'
      fields_1_to_2:
        foo: 'foo'
        ros1_bar: 'ros2_bar'

Example service mapping rule matching all services with the same names and fields within a pair of packages::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros2_package_name: 'ros2_pkg_name'

Example service mapping rule matching a specific pair of services with the same fields::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros1_service_name: 'ros1_srv_name'
      ros2_package_name: 'ros2_pkg_name'
      ros2_service_name: 'ros2_srv_name'

Example service mapping rule matching a specific pair of services with a custom mapping between the fields.
The mapping can optionally only define ``request_fields_1_to_2`` or ``response_fields_1_to_2`` if the other part has the same fields::

    -
      ros1_package_name: 'ros1_pkg_name'
      ros1_service_name: 'ros1_srv_name'
      ros2_package_name: 'ros2_pkg_name'
      ros2_service_name: 'ros2_srv_name'
      request_fields_1_to_2:
        foo: 'foo'
        ros1_bar: 'ros2_bar'
      response_fields_1_to_2:
        foo: 'foo'
        ros1_bar: 'ros2_bar'


How does the bridge know about custom interfaces?
-------------------------------------------------

The ROS 1 and ROS 2 packages need to be in separate workspaces, so that each workspace can be sourced with its correponding ROS version.
The bridge should be in its own workspace, as it will need to source both ROS 1 and ROS 2 versions.

Example workspace setup
-----------------------

Here we will call the ROS 1 workspace ``ros1_msgs_ws``, the ROS 2 workspace ``ros2_msgs_ws``, the workspace containing the bridge ``bridge_ws``.
For simplification, we will use matching names for the packages, messages, and fields of the custom interfaces on the ROS 1 and ROS 2 sides.
We will call the actual package ``bridge_msgs`` in both the ROS 1 and the ROS 2 workspaces.
That is, the name defined in ``CMakeLists.txt`` and ``package.xml`` in the package.

The directory layout looks like this::

    .
    ├─ ros1_msgs_ws
    │  └─ src
    │     └─ bridge_msgs
    │        └─ msg
    │           └─ JointCommand.msg
    ├─ ros2_msgs_ws
    │  └─ src
    │     └─ bridge_msgs
    │        ├─ msg
    │        │  └─ JointCommand.msg
    │        └─ # YAML file if your custom interfaces have non-matching names
    └─ bridge_ws
       └─ src
          └─ ros1_bridge

The content of JointCommand.msg::

    float64 position

The workspaces can be compiled as follows.

First, build the ROS 1 messages::

    # Shell 1 (ROS 1)
    . /opt/ros/melodic/setup.bash
    # Or, on OSX, something like:
    # . ~/ros_catkin_ws/install_isolated/setup.bash
    cd <workspace-parent-path>/ros1_msgs_ws
    catkin_make_isolated --install

Then build the ROS 2 messages::

    # Shell 2 (ROS 2)
    . /opt/ros/crystal/setup.bash
    cd <workspace-parent-path>/ros2_msgs_ws
    colcon build --packages-select bridge_msgs

Then build the bridge::

    # Shell 3 (ROS 1 and ROS 2)
    . /opt/ros/melodic/setup.bash
    . /opt/ros/crystal/setup.bash
    . <workspace-parent-path>/ros1_msgs_ws/install_isolated/setup.bash
    . <workspace-parent-path>/ros2_msgs_ws/install/local_setup.bash
    cd <workspace-parent-path>/bridge_ws
    colcon build --packages-select ros1_bridge --cmake-force-configure

Verify the custom types were recognized by the bridge, by printing all pairs of bridged types.
The custom types should be listed::

    ros2 run ros1_bridge dynamic_bridge --print-pairs

Run the bridge, reusing shells from above::

    # Shell 1 (ROS 1)
    roscore

    # Shell 2 (ROS 2)
    . <workspace-parent-path>/ros2_msgs_ws/install/local_setup.bash
    ros2 topic pub /joint_command bridge_msgs/JointCommand "{position: 0.123}"

    # Shell 3 (ROS 1 and ROS 2)
    . <workspace-parent-path>/bridge_ws/install/local_setup.bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics

    # Shell 4 (ROS 1)
    . /opt/ros/melodic/setup.bash
    . <workspace-parent-path>/ros1_msgs_ws/install_isolated/setup.bash
    # Verify the topic is listed
    rostopic list
    rostopic echo /joint_command


How can I define a mapping rules for a foreign package, from a foreign package?
-------------------------------------------------------------------------------

In the previous sections, it was stated that the ``ros2_package_name`` mapping rule must be the same as the ROS 2 package the mapping rule was defined in.
While this is **recommended** to prevent conflicting and/or duplicate rules, it is possible to override the check that enforces this with the ``enable_foreign_mappings`` field.

This will mean that, for every package mapping rule defined in the ``yaml`` file, the check for ROS 2 package name equality will be skipped.
Again, note that this is a dark art that should be wielded with responsibility, please be very careful with this!
If there are conflicting mapping rules, the last one that is parsed in sorting order is used!
This is usually hard to predict, so be very careful!

With ``enable_foreign_mappings`` set to ``true``, you can then specify mapping rules for ROS 2 packages that are not the same as the package your mapping rules file resides in::

    -
      enable_foreign_mappings: true
      ros1_package_name: 'ros1_pkg_name'
      ros1_service_name: 'ros1_srv_name'
      ros2_package_name: 'ros2_FOREIGN_pkg_name'  # the package with the message definition
      ros2_service_name: 'ros2_srv_name'

You must also make the ``ros1_bridge_foreign_mapping`` ament resource available to the index in the mapping package's ``CMakeLists.txt``.
A good place to put the line is to do this is before the install rule for your mapping rules, like so::

    -
      ament_index_register_resource("ros1_bridge_foreign_mapping")
      install(
        FILES YOUR_MAPPING_RULE_FILE.yaml
        DESTINATION share/${PROJECT_NAME})

**Note**: In this case, the package name you should put in ```ros2_package_name`` is the name of the package with the message definition.
In effect, it'll be a foreign package, relative to the mapping package you're defining your mapping rules in.

An example directory layout looks like this::

    .
    ├─ ros1_msgs_ws
    │  └─ src
    │     └─ ros1_bridge_msgs
    │        └─ msg
    │           └─ ros1_msg_name.msg
    ├─ ros2_msgs_ws
    │  └─ src
    │     └─ ros2_bridge_msgs
    │     │  ├─ msg
    │     │  │  └─ ros2_msg_name.msg
    │     └─ ros2_bridge_mappings
    │        └─ # YAML file that defines mapping rules for bridge_msgs, or some other foreign msg package
    └─ bridge_ws
       └─ src
          └─ ros1_bridge

In the above example, the mapping rule would look like this::

    -
      enable_foreign_mappings: true
      ros1_package_name: 'ros1_bridge_msgs'
      ros1_message_name: 'ros1_msg_name'
      ros2_package_name: 'ros2_bridge_msgs'  # this is a foreign package, relative to ros2_bridge_mappings!
      ros2_message_name: 'ros2_msg_name'


Known Issues
------------

- Currently, ``--bridge-all-topics`` may be needed to bridge correctly.
  This has been `tested and reported <https://github.com/ros2/ros1_bridge/issues/200>`_ to be required when at least one custom message type is being published in ROS 2 and subscribed in ROS 1.
  It has also been informally reported to be needed for built-in message types under the same condition.
  Once the mapping is established with the ROS 1 master, it may be possible to rerun the bridge without ``--bridge-all-topics``, in order to selectively bridge topics.
  However, this is not guaranteed.
- ``/rosout`` logging, which maps from ``rosgraph_msgs/Log`` to ``rcl_interfaces/Log``, requires `field selection <https://github.com/ros2/ros1_bridge/pull/174>`_.
  This `works with OpenSplice and Connext <https://github.com/ros2/rcl_interfaces/pull/67>`_, but `not with Fast-RTPS <https://github.com/ros2/rcl_interfaces/issues/61>`_.
  For it to work with Fast-RTPS, `this bug <https://github.com/ros2/rmw_fastrtps/issues/265>`_ needs to be fixed.
  As a workaround, run the subscriber with ``--disable-rosout-logs``.
