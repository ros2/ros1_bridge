# Bridge communication between ROS 1 and ROS 2

The current implementation contains:

* a bidirectional bridge which can handle all message types but only creates bridges for new topics available on the ROS 1 side.
* a bidirectional bridge which can only pass along a single message type on a fixed topic.
* two unidirectional bridges which can only pass along a single message type on a fixed topic.


## How to build the bridge?

The bridge uses `pkg-config` to find ROS 1 packages.
ROS 2 packages are located in CMake using `find_package()`.
Therefore the `CMAKE_PREFIX_PATH` must not contain paths from ROS 1 which overlay ROS 2 packages.

In order to build this ament package you must first setup the ROS 1 environment:

```
. /opt/ros/indigo/setup.bash
unset CMAKE_PREFIX_PATH
```

and second source the ROS 2 environment:

```
. <workspace-with-ros2>/install/setup.bash
```

Afterwards you can build the ament package as usual.
Since the compilation step can take a lot of memory for every thread you might want to restrict the number of parallel jobs:

```
ament build <workspace-with-bridge> --make-args -j1
```


## How to setup the environment to run the bridge?

You need to source the environment of the ament install space where the bridge was built in.
Additionally you will need to either source the ROS 1 environment or at least set the `ROS_MASTER_URI`.


## How to run the bridge and the example talker and listener?

### Dynamic bridge

To pass images from ROS 2 to ROS 1 and a change the configuration from ROS 1 to ROS 2 invoke the following:

```
# Shell A:
. /opt/ros/indigo/setup.bash
roscore
```

```
# Shell B:
. <workspace-with-bridge>/install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
dynamic_bridge
```

For now you need to start a ROS 1 publisher since the dynamic bridge cannot introspect the ROS 2 topics yet.

```
# Shell C:
. /opt/ros/indigo/setup.bash
rosrun usb_cam usb_cam_node usb_cam/image_raw:=image
# let it run for a few seconds and then stop it with Ctrl+C
```

Now you can start the ROS 2 camera node:

```
# Shell D:
. <workspace-with-ros2>/install/setup.bash
cam2image
```

Now you can start `rqt` in ROS 1:

```
# Shell E:
. /opt/ros/indigo/setup.bash
rqt
```

In `rqt` you need to open the `Image View` plugin and select the `image` topic.
You can additionally use the `Message Publisher` plugin to publish a `std_msgs/Bool` message on the topic `flip_image`.
By publishing either `true` or `false` the camera node with conditionally flip the image before sending it.


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
