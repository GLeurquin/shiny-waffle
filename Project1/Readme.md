# Project 1

This is the first project of the robotics Engineer nanodegree.

# Summary of Tasks
  * Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure. Make sure there's enough space between the walls for a robot to navigate.

  * Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
  * Import your structure and two instances of your model inside an empty Gazebo World.
  * Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
  * Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

# Build the project

To build the project:
`mkdir build && cd build`
`cmake .. && make && cd ..`
`export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD/build`

# Run the project
`gazebo world/world`
