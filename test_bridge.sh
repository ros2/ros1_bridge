
#!/bin/bash
set -e

source ~/colcon_ws/install/setup.bash
cd ~/colcon_ws
colcon test --paths ~/colcon_ws/src/ros1_bridge
colcon test-result --verbose
