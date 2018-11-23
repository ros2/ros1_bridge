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

How can I specify custom mapping rule for messages?
---------------------------------------------------

Additional mapping rules can be provided by a ROS 2 package using a `yaml` file.
Each mapping rule can have one of three types:

1. A package mapping rule is defined by:

   - a ``ros1_package_name``
   - a ``ros2_package_name`` (which must be the same as the ROS 2 package this mapping rule is defined in)

2. A message mapping rule is defined by the attributes of a package mapping rule and:

   - a ``ros1_message_name``
   - a ``ros2_message_name``

3. A field mapping rule is defined by the attributes of a message mapping rule and:

   - a dictionary ``fields_1_to_2`` mapping ROS 1 field names to ROS 2 field names.
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
   - a ``ros2_package_name`` (which must be the same as the ROS 2 package this mapping rule is defined in)

2. A service name mapping rule is defined by the attributes of a package mapping rule and:

   - a ``ros1_service_name``
   - a ``ros2_service_name``

A custom field mapping is currently not supported for services.

How can I install mapping rule files?
-------------------------------------

The mapping rule files must be exported in the ``package.xml`` in order to be processed by this package::

    <export>
      <ros1_bridge mapping_rules="my_mapping_rules.yaml"/>
    </export>

The yaml files must also be installed in the ``CMakeLists.txt``::

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
