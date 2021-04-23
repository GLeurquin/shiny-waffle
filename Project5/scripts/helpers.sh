#!/usr/bin/env bash

set -euo pipefail

# Make font bigger, for big screens much easier to read :)
TERMINAL="xterm -fa 'Monospace' -fs 22"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
export CATKIN_WS_DIR=$DIR/../..
export PACKAGE_DIR=$DIR/..

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
    pgrep -f gazebo | xargs kill
    sleep 3
  done

  while [ ! -z "$(pgrep -f ros)" ]; do
    echo "Killing previous ros processes..."
    pgrep -f ros | xargs kill
    sleep 3
  done
}

setup_env() {
  set -euo pipefail

  echo "Sourcing ros"
  source /opt/ros/kinetic/setup.bash

  # Set up ROS ip
  export ROS_IP=`echo $(hostname -I)`

  # Source the catkin setup.bash
  echo "Sourcing $CATKIN_WS_DIR/devel/setup.bash"
  source $CATKIN_WS_DIR/devel/setup.bash

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
