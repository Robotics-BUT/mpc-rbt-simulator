# MPC-RBT Simulator

This ROS 2 package embodies a Webots-based simulation environment for the lab tasks of the MPC-RBT course. The simulated scenario involves a small warehouse equipped with a compact mobile robot to fulfill tasks such as pick-and-place operations. The overall goal is to tackle basic challenges in mobile robotics, including self-localization, path planning, and motion control, as well as to become familiar with both the ROS 2 framework and the Webots simulator.

![The simulated warehouse](media/warehouse.jpg)

## Package Structure

TODO

## Installation

TODO

## Usage

TODO

## TODO

- Create a static 2D map (occupancy grid) for given world for the path planning purposes OR create an automatic converter .wbt -> .pgm (occupancy grid)
- Add a map server to provide a static map for path planning a rviz visualization
- Prepare templates (source files, nodes..) for students for the individual tasks, eg localization, path planning..
- Validate robot parameters - wheel radius and distance
- Create documentation (install, usage..)

## Issues

- Fix and simplify the robot .urdf model - some transforms don't exist (casters), some frames aren't necessary.
- Fix namespaces - some topics already have /tiago_base namespace provided by Webots 