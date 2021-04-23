#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source $DIR/helpers.sh

kill_gazebo

# Deploy turtlebot in my environment (turtlebot_simulator)
exec_roslauch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find pick_objects)/worlds/myworld.world &
sleep 5

# Perform AMCL
exec_roslauch pick_objects amcl.launch map_file:=$(rospack find pick_objects)/map/myworld.yml --wait &

# Observe the map in RVIZ
exec_roslauch pick_objects rviz.launch --wait &

# Launch pick objects
exec_roslauch add_markers add_markers.launch --wait &

modify_marker() {
  x=$1
  y=$2
  id=$3
  action=$4
  rosservice call /add_markers/modify_marker "position:
  x: $x
  y: $y
  z: 0.0
id: $id
scale: 0.3
action: $action"
}

perform_sequence() {
    echo "Adding marker to pickup location..."
    modify_marker -7.0 -2.0 0 0

    echo "Sleeping 5 seconds"
    sleep 5

    echo "Removing marker from pickup location"
    modify_marker -7.0 -2.0 0 2

    echo "Sleeping 5 seconds"
    sleep 5

    echo "Adding marker to dropoff location"
    modify_marker 4.0 -3.0 1 0
}

setup_env
echo "Waiting 30 seconds"
sleep 30
perform_sequence
ctrl_c
