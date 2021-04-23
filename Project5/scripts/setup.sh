#!/usr/bin/env bash

set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Move to src folder
cd $DIR/../

# Install xterm
sudo apt-get install -y xterm

# gmapping
git clone https://github.com/ros-perception/slam_gmapping.git
cd slam_gmapping
git checkout hydro-devel
cd ..

# turtlebot teleop
git clone https://github.com/turtlebot/turtlebot.git
cd turtlebot
git checkout kinetic
cd ..

# Turtlebot rviz launchers
git clone https://github.com/turtlebot/turtlebot_interactions
cd turtlebot_interactions
git checkout indigo
cd ..

# turtlebot_simulator
git clone https://github.com/turtlebot/turtlebot_simulator
cd turtlebot_simulator
git checkout indigo
cd ..
