# P3GASUS

[P3GASUS](https://ieeexplore.ieee.org/document/11282966) is a ROS-based framework for path execution in ultra large scale systems, with support for discrete multi-agent path finding (MAPF) and continuous traffic-style coordination.

We are currently refactoring the ROS codebase. This repository contains the raw implementation. Please [raise an issue](https://github.com/marmotlab/P3GASUS/issues) if you encounter any problems, and we'll address them promptly.

If you are only interested in the graph-creation algorithms for P3GASUS, it is also available as a [separate repository](https://github.com/marmotlab/P3GASUS-graph-creation).

## Dependencies

This is a ROS-based simulation framework. Ensure you have one of the following ROS 1 distributions installed:
- [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

For discrete multi-agent path planning, we utilize [LACAM3](https://github.com/Kei18/lacam3/tree/pybind) (specifically the pybind version). Pre-built Python 3.8 and 3.11 bindings are included; you may need to rebuild bindings for other Python versions.

## Installation

Clone this repository into your catkin workspace's source folder and build:

```bash
cd ~/catkin_ws/src
git clone https://github.com/marmotlab/P3GASUS
cd ~/catkin_ws
catkin_make
```

## Quick Start

**Terminal 1 - Initialize:**
```bash
roslaunch marmot driver.py
```

**Terminal 2 - Run simulation:**
```bash
rosrun marmot oneRun.py
```

Configure your environment using `scripts/parameters.py` (see below for details on modes and parameters).

## Modes & Hybrid Demo
P3GASUS supports purely simulated runs as well as a hybrid setup that couples simulated agents with real robots tracked by motion capture.

We use an OptiTrack motion capture system and SparkLife mecanum-wheeled robots for the hybrid (sim + real) demo. The real-robot pipeline is tightly coupled to that hardware; if you use other robots or MoCap systems you will likely need to adapt topics, robot drivers, and IP layout. Key knobs in [scripts/parameters.py](scripts/parameters.py) include:
- `VIRTUAL_TO_REAL_ROBOT_MAPPING`: maps virtual agent indices to physical robot IDs.
- `OPTITRACK_TO_MAP_SHIFT`: aligns the motion-capture coordinate frame with the map frame.
- `VRPN_IP`: address of the OptiTrack VRPN server.
- `ROS_MASTER_IP` / `REAL_ROBOT_IP`: ROS master and real-robot network configuration.

## Parameter Customization
By default the system starts in MAPF mode on a 10x10 open map with 8 robots. You can switch scenarios and tune behavior in [scripts/parameters.py](scripts/parameters.py):
- Set `DriverParameters.SCENARIO` to `0` for discrete MAPF or `1` for continuous traffic; adjust `HYBRID_ROBOT_COUNT` for number of robots in the environment, `GROUP_COUNT` for parallel processing, and `REAL_WORLD_SIZE` for map extents.
- Networking and hardware: update `VRPN_IP`, `ROS_MASTER_IP`, `REAL_ROBOT_IP`, and `VIRTUAL_TO_REAL_ROBOT_MAPPING` to match your MoCap server and robot fleet; toggle `DEBUG=True` for verbose logs.
- Localization alignment: use `OPTITRACK_TO_MAP_SHIFT` to offset MoCap coordinates into the map frame.

### Discrete Space (MAPF)
- Planner inputs: set `MAPFParameters.WORLD` (occupancy grid) and `MAPFParameters.STARTS` (start/goal pairs) or rely on scenario loaders.
- Kinematics and safety: tune `SAFETY_DISTANCE`, virtual speeds (`VIRTUAL_FAST_VEL`, `VIRTUAL_SLOW_VEL`, thresholds), and real speeds/thresholds (`REAL_FAST_VEL`, `REAL_SLOW_VEL`, `REAL_FAR_THRESHOLD`, `REAL_REACH_THRESHOLD`, `REAL_ANGULAR_VEL`, `ANGULAR_REACH_THRESHOLD`).
- Imitation runs: enable `IMITATE=True`, point `IMITATE_FOLDER` to a recorded scenario, and select robots with `IMITATION_LIST`; cap replay length with `IMITATION_LENGTH`.

### Continuous Space (Traffic)
- Scenario core: set `TrafficParameters.ORIGIN` for map anchoring and `FUTURE_TASKS` for the look-ahead task queue.
- Motion tuning: distance thresholds (`FAR_THRESHOLD`, `REACH_THRESHOLD`), virtual velocity window (`VIRTUAL_MIN_VEL`–`VIRTUAL_MAX_VEL`), real velocity caps (`REAL_SLOW_VEL`, `REAL_FAST_VEL`), and angular behavior (`ANG_THRESHOLD`, `MAX_ANG_VEL`, `MIN_ANG_VEL`).
- Global pacing: scale all traffic speeds quickly with `SPEED_MULTIPLIER`.

## Repository Structure
Some useful top-level directories:
- `scripts/`: main Python drivers, controllers, utilities, and `parameters.py` for configuration.
- `launch/`: example ROS launch files for starting the simulation and auxiliary nodes.
- `map/` and `worlds/`: example maps and Gazebo worlds used in the experiments.
- `urdf/`: robot and goal description files.
- `msg/`: custom ROS message definitions used by P3GASUS.
- `rviz/`: RViz configurations for visualizing agents and maps.

## Support
If you encounter issues or have questions, please open an issue on the [GitHub tracker](https://github.com/marmotlab/P3GASUS/issues) with a short description, your ROS version, and the steps to reproduce.



