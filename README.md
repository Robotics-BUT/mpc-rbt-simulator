# MPC-RBT Simulator

<div align="center">

[![Tests](https://github.com/Robotics-BUT/mpc-rbt-simulator/actions/workflows/test.yml/badge.svg?branch=main)](https://github.com/Robotics-BUT/mpc-rbt-simulator/actions/workflows/test.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

</div>

This ROS 2 package embodies a Webots-based simulation environment for the lab tasks of the MPC-RBT course. The simulated scenario involves a small warehouse equipped with a compact mobile robot to fulfill tasks such as pick-and-place operations. The overall goal is to tackle basic challenges in mobile robotics, including self-localization, path planning, and motion control, as well as to become familiar with both the ROS 2 framework and the Webots simulator.

![The simulated warehouse](media/warehouse.jpg)

> TODO: add some details (robot model, sensors etc)

## Installation

To run the project (use this package) various software requirements must be met. This section briefly describes the installation procedure.

### Ubuntu

The project requires the `Ubuntu 22.04 LTS` Linux distribution. Follow the official installation guide to install it:

https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

> **Warning:** Proceed carefully to prevent possible data loss in case of installation on a PC/Laptop with another operating system.

### ROS 2

The key software framework used in this project is `ROS 2 Humble`. To install it from a pre-built binary package, follow this official guide:

https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html

### Webots

Webots is the desktop application used to simulate robots and the environment in this project. It requires Webots version `2023b`. One way to install it is to download the pre-built `webots_2023b_amd64.deb` package from the release page:

https://github.com/cyberbotics/webots/releases/tag/R2023b

And install it from the same directory manually using:

```
wget https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb && sudo apt install ./webots_2023b_amd64.deb
```

Another option is to set up pre-built binary installation from remote `apt` repositories. All installation options can be found here:

https://cyberbotics.com/doc/guide/installation-procedure?tab-os=linux#installation-on-linux

### ROS 2 Workspace

A ROS 2 workspace is represented by a directory, `mpc_rbt_ws` for example, and packages are typically located in the `src` subdirectory (additional directories `build`, `install`, `log` will be created after compilation). Navigate to the desired location and prepare your workspace:

```
mkdir mpc_rbt_ws
cd mpc_rbt_ws
mkdir src
```

### MPC-RBT Simulator

This repository embodies a single ROS 2 package. Navigate to the workspace directory (`mpc_rbt_ws`) and clone this repository:

```
git clone git@github.com:Robotics-BUT/mpc-rbt-simulator.git src/mpc_rbt_simulator
```

or:

```
git clone https://github.com/Robotics-BUT/mpc-rbt-simulator.git src/mpc_rbt_simulator
```

Package dependencies (mostly other ROS 2 packages) are listed in `package.xml` and can be installed either manually (from source or binary) or automatically via the `rosdep` utility.

Install `rosdep` (if necessary):

```
apt-get install python3-rosdep
```

Initialize it and update the rosdistro index:

```
sudo rosdep init
rosdep update
```

Install all dependencies using the following command:

```
rosdep install --from-paths src -y -r --ignore-src --rosdistro humble
```

Most of the dependencies are a common part of the ROS 2 distribution. An exception is the package `webots_ros2_driver`, which is necessary to connect ROS 2 and the Webots simulator.

### Visual Studio Code

Any IDE may be used to work on this projet. Visual Studio Code, for example, is a good choice. To install it, download its `.deb` package from the official website:

https://code.visualstudio.com/

Install it from the same directory using:

```
sudo apt install ./<file>.deb
```

## Package Structure

TODO

## Usage

TODO

Navigate to the workspace directory (`mpc_rbt_ws`) and build it:

```
colcon build
```

Set up the environment for the workspace:

```
source install/setup.bash
```

Launch the project (including the simulation) using:

```
ros2 launch mpc_rbt_simulator simulation.launch.py
```

## Testing

Navigate to the workspace directory (`mpc_rbt_ws`) and build it:

```
colcon build
```

Change the initial robot position in `src/mpc_rbt_simulator/worlds/mpc-rbt-warehouse.wbt` to `0 2.5 0.095` by modifying the `translation` parameter in `TiagoBase`.

Spawn the simulation with the robot controller using:

```
webots --port=1234 --no-rendering --stdout --stderr --minimize ./src/mpc_rbt_simulator/worlds/mpc-rbt-warehouse.wbt --batch --mode=realtime
```
> [!NOTE]
> If you want to watch the simulation during tests remove the `--no-rendering` and `--minimize` arguments.

Run tests using:

```
WEBOTS_CONTROLLER_PORT=1234 colcon test --ctest-args tests
```

View the results using:
```
colcon test-result --verbose --all
```
