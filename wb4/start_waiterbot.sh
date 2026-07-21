#!/bin/bash
source /opt/ros/humble/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models
echo "=== Stopping old processes ==="
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rviz2 2>/dev/null; pkill -9 -f controller_server 2>/dev/null
pkill -9 -f planner_server 2>/dev/null; pkill -9 -f bt_navigator 2>/dev/null
pkill -9 -f map_server 2>/dev/null; pkill -9 -f amcl 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
ros2 daemon stop 2>/dev/null; sleep 2; ros2 daemon start 2>/dev/null; sleep 1
echo "=== Building ==="
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$HOME/ros2_ws/src"
cp -r "$SCRIPT_DIR/src/restaurant_robot" "$HOME/ros2_ws/src/"
cd "$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select restaurant_robot 2>&1 | tail -3
source "$HOME/ros2_ws/install/setup.bash"
cp "$SCRIPT_DIR/src/restaurant_robot/scripts/waiterbot_tour.py" "$HOME/Desktop/waiterbot_tour.py" 2>/dev/null
echo "=== Launching ==="
echo "Wait for: Managed nodes are active"
echo "Then open NEW terminal and run:"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  python3 ~/Desktop/waiterbot_tour.py"
ros2 launch restaurant_robot restaurant_navigation.launch.py
