#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source $DIR/helpers.sh
trap ctrl_c INT

kill_gazebo

# Deploy turtlebot in my environment (turtlebot_simulator)
exec_roslauch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find pick_objects)/worlds/myworld.world &

sleep 5

# Perform AMCL
exec_roslauch pick_objects amcl.launch map_file:=$(rospack find pick_objects)/map/myworld.yml --wait  &

# Observe the map in RVIZ (turtlebot_interactions)
exec_roslauch pick_objects rviz.launch --wait
