#!/bin/bash
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f rviz2 2>/dev/null; pkill -9 -f controller_server 2>/dev/null
pkill -9 -f planner_server 2>/dev/null; pkill -9 -f bt_navigator 2>/dev/null
pkill -9 -f map_server 2>/dev/null; pkill -9 -f amcl 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
ros2 daemon stop 2>/dev/null
echo "All stopped."
