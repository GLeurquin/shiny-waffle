#!/usr/bin/env bash

set -euo pipefail

# Make font bigger, for big screens much easier to read :)
TERMINAL="xterm -fa 'Monospace' -fs 22"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# This assumes the devel folder containing catkin's
# setup.bash is two folder up from this script.
# If this is different in your setup,
# Update the line below
export CATKIN_SOURCE_BASH=$DIR/../../devel/setup.bash

# The ros source bash
export ROS_SOURCE_BASH=/opt/ros/kinetic/setup.bash

wait_for_any_key() {
  read -n 1 -s -r -p 'Press any key to continue'
}
export -f wait_for_any_key

ctrl_c() {
  wait_for_any_key
  kill -- -$$
}
export -f ctrl_c

kill_gazebo() {
  while [ ! -z "$(pgrep -f gazebo)" ]; do
    echo "Killing previous gazebo processes..."
    pkill -f gazebo
    sleep 3
  done

  while [ ! -z "$(pgrep -f ros)" ]; do
    echo "Killing previous ros processes..."
    pkill -f ros
    sleep 3
  done

  while [ ! -z "$(pgrep -f rviz)" ]; do
    echo "Killing previous rviz processes..."
    pkill -f rviz
    sleep 3
  done
}

setup_env() {
  set -euo pipefail

  echo "Sourcing ros"
  source $ROS_SOURCE_BASH

  # Set up ROS ip
  export ROS_IP=`echo $(hostname -I)`

  # Source the catkin setup.bash
  echo "Sourcing $CATKIN_SOURCE_BASH"
  source $CATKIN_SOURCE_BASH

  trap ctrl_c INT
}
export -f setup_env

exec_roslauch() {
  package=$1
  launch_file=$2
  args=$(echo "${@:3}")
  roslaunch_cmd="roslaunch $package $launch_file $args"
  $TERMINAL -e "setup_env && $roslaunch_cmd || wait_for_any_key"
}
