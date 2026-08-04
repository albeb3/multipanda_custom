# MultiPanda Custom

`multipanda_custom` contains the custom packages developed on top of the original MultiPanda MuJoCo framework. The goal is to provide a clean separation between the original repository and user-developed components, making future updates and maintenance easier.

## Package Overview

```
multipanda_custom/
├── alberto_bringup/
└── alberto_controllers/
```

### alberto_bringup

Launch and configuration package for the custom framework.

It contains:

- Launch files for simulation.
- Controller configuration files.
- Robot bringup utilities.
- MuJoCo plugin configuration.
- Controller YAML files.

Typical responsibilities:

- Launch MuJoCo simulation.
- Spawn ROS 2 controllers.
- Start `robot_state_publisher`.
- Configure controller_manager.
- Configure custom controllers.

---

### alberto_controllers

Collection of custom ROS 2 controllers.

Current organization:

```
alberto_controllers/
├── include/
│   └── alberto_controllers/
│       └── panda/
├── src/
│   └── panda/
└── alberto_controllers.xml
```

Currently implemented controllers include:

- Joint Velocity Controller
- Joint Impedance Controller *(work in progress)*
- Dual Joint Velocity Controller *(work in progress)*
- Dual Joint Impedance Controller *(work in progress)*

The package is organized by robot family, allowing future support for additional robots without changing the existing architecture.

Example future structure:

```
alberto_controllers/
├── panda/
├── ur/
├── iiwa/
├── garmi/
└── ...
```

## Dependencies

The packages depend on:

- ROS 2 Humble
- ros2_control
- controller_interface
- hardware_interface
- pluginlib
- rclcpp
- rclcpp_lifecycle
- Eigen3
- std_msgs
- geometry_msgs

Additional dependencies are inherited from the MultiPanda framework.

## Build

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
    --packages-select alberto_controllers alberto_bringup \
    --symlink-install
```

After compilation:

```bash
source install/setup.bash
```

## Running

The bringup package provides the launch files required to start the simulation together with the custom controllers.

Example:

```bash
ros2 launch alberto_bringup franka_sim.launch.py
```

Controllers can then be loaded using the standard ROS 2 controller manager utilities.

## Design Philosophy

The purpose of `multipanda_custom` is to keep all custom developments separated from the original MultiPanda repository.

This approach provides:

- easier maintenance;
- cleaner code organization;
- simpler upstream updates;
- reusable controllers;
- modular bringup configuration.

The original repository remains untouched, while all custom code is maintained inside this package collection.