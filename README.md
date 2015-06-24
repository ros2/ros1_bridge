# Bridge communication between ROS 1 and ROS 2

The current simple prototype contains:

* a bidirectional bridge which can only pass along a single message type on a fixed topic.
* two unidirectional bridges which can only pass along a single message type on a fixed topic.


## How to build the bridge?

In order to build this ament package you must have both environment sourced:

```
. /opt/ros/indigo/setup.bash
. <workspace-with-ros2>/install/setup.bash
ament build <workspace-with-bridge>
```


## How to setup the environment to run the bridge?

You need to source the environment of the ament install space where the bridge was built in.
Additionally you will need to either source the ROS 1 environment or at least set the `ROS_MASTER_URI`.


## How to run the bridge and the example talker and listener?

### Bidirectional bridge

To communicate information between ROS 1 to ROS 2 in both directions invoke the following:

```
# Shell A:
. /opt/ros/indigo/setup.bash
roscore
```

```
# Shell B:
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
simple_bridge
```

Now you can start publishers / subscribers on either side:

```
# Shell C:
. /opt/ros/indigo/setup.bash
rosrun roscpp_tutorials talker
```

```
# Shell D:
. <workspace-with-bridge>/install/setup.bash
talker
```

```
# Shell E:
. /opt/ros/indigo/setup.bash
rosrun roscpp_tutorials listener
```

```
# Shell F:
. <workspace-with-bridge>/install/setup.bash
listener
```


### Unidirectional bridges

To communicate information from ROS 1 to ROS 2 invoke the following:

```
# Shell A:
. /opt/ros/indigo/setup.bash
roscore
```

```
# Shell B:
. /opt/ros/indigo/setup.bash
rosrun roscpp_tutorials talker
```

```
# Shell C:
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
simple_bridge_1_to_2
```

```
# Shell D:
. <workspace-with-bridge>/install/setup.bash
listener
```

To communicate information from ROS 2 to ROS 1 invoke the following:

```
# Shell A:
. <workspace-with-bridge>/install/setup.bash
talker
```

```
# Shell B:
. /opt/ros/indigo/setup.bash
roscore
```

```
# Shell C:
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
simple_bridge_2_to_1
```

```
# Shell D:
. /opt/ros/indigo/setup.bash
rosrun roscpp_tutorials listener
```
