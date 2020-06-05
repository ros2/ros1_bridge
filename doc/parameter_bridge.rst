
parameter_bridge configuration
==============================

You can launch parameter_bridge with the following command::
  
    ros2 run ros1_bridge parameter_bridge [<topics_name> [<services_1_to_2_name> [<services_2_to_1_name>]]]

Parameter bridge reads ``topics``, ``services_1_to_2``, and ``services_2_to_1`` parameters on ROS1 as default to make your bridge.
You can specify the names of those parameters as arguments of the command.
Here is an example of configuration::

 <rosparam>
 topics:
   -
     topic: /clock
     type: rosgraph_msgs/msg/Clock
     queue_size: 100
     direction: 1to2
   -
     topic: /tf
     type: tf2_msgs/msg/TFMessage
     queue_size: 10
     direction: bidirectional
     reliable: true
   -
     topic: /tf_static
     type: tf2_msgs/msg/TFMessage
     queue_size: 10
     direction: 1to2
     transient_local: true
     reliable: true
   -
     topic: /scan
     type: sensor_msgs/msg/LaserScan
     queue_size: 10
     direction: 1to2
   -
     topic: /cmd_vel
     type: geometry_msgs/msg/Twist
     queue_size: 10
     direction: 2to1
   -
     topic: /map
     type: nav_msgs/msg/OccupancyGrid
     queue_size: 1
     direction: 2to1
     transient_local: true
     reliable: true
     latch: true

 services_1_to_2:
   -
     service: /gazebo/pause_physics
     type: std_srvs/srv/Empty

 services_2_to_1:
   -
     service: /reinitialize_global_localization
     type: std_srvs/srv/Empty
 </rosparam>

Configuration Syntax
--------------------

- **topics**: <array of **topic**>
- **topic**: <dict>

  .. csv-table:: 
   :header: "key", "required", "default", "description"
   :widths: 20, 10, 10, 50

   **topic**, YES, N/A, Name of the topic to bridge which is used in both ROS1/2
   **type**,       YES, N/A, Message type in ROS2 (see `here <./index.rst>`_ how to map ROS1 and ROS2 messages)
   **queue_size**, NO,  100, Queue size used in both ROS1/2 
   **direction**,  NO,  bidirectionl, ``bidirectional`` or ``1to2`` or ``2to1``
   **transient_local**, NO, false, Set ROS2 QoS durability profile transient_local (used both for 1to2 and 2to1)
   **reliable**,        NO, false, Set ROS2 QoS durability profile reliable (used both for 1to2 and 2to1)
   **latch**,           NO, false, Publish ROS1 message with latch option (used only for 2to1)
- **services_1_to_2**: <array of **service**>, ROS1 services will be available in ROS2
- **services_2_to_1**: <array of **service**>, ROS2 services will be available in ROS1
- **service**: <dict>

  .. csv-table:: 
   :header: "key", "required", "description"
   :widths: 20, 10, 60

   **service**, YES, Name of the service to bridge which is used in both ROS1/2
   **type**, YES, Service type in ROS2  (see `here <./index.rst>`_ how to map ROS1 and ROS2 messages)
