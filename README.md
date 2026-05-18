# P3GASUS

[P3GASUS](https://ieeexplore.ieee.org/document/11282966) is a ROS-based framework for path execution in ultra-large-scale systems, supporting discrete multi-agent path finding (MAPF) and continuous traffic-style coordination.

> **Note:** We are currently refactoring the ROS codebase. If you encounter any problems, please [raise an issue](https://github.com/p3gasuslab/P3GASUS/issues) and we'll address it promptly.

If you are only interested in the graph-creation algorithms, they are also available as a [standalone repository](https://github.com/p3gasuslab/P3GASUS-graph-creation).

## Dependencies

P3GASUS requires one of the following ROS 1 distributions:

- [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
- [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)

For discrete multi-agent path planning, P3GASUS uses [LACAM3](https://github.com/Kei18/lacam3/tree/pybind) (pybind branch). Pre-built bindings for Python 3.8 and 3.11 are included; other Python versions will require rebuilding the bindings.

## Installation

Clone this repository into your catkin workspace and build:

```bash
cd ~/catkin_ws/src
git clone https://github.com/p3gasuslab/P3GASUS
cd ~/catkin_ws
catkin_make
```

Then set the path to the [P3GASUS-graph-creation](https://github.com/p3gasuslab/P3GASUS-graph-creation) repository in `scripts/parameters.py`:

```python
PATH_TO_P3GASUS_GRAPH_CREATION = "<your path>"
```

If you haven't cloned it yet:

```bash
git clone https://github.com/p3gasuslab/P3GASUS-graph-creation
```

Then set the path as above.

## Quick Start

**Terminal 1 — Initialize:**

```bash
roslaunch p3gasus driver.py
```

> **Note:** If you see an error about a non-executable file, make the relevant script executable first:
> ```bash
> chmod +x <filename.py>
> ```

**Terminal 2 — Run simulation:**

```bash
rosrun p3gasus oneRun.py
```

Configure your environment via `scripts/parameters.py` (see [Parameter Customization](#parameter-customization) below).

## Modes & Hybrid Demo

P3GASUS supports purely simulated runs as well as a hybrid setup that couples simulated agents with real robots tracked by motion capture.

The hybrid demo uses an OptiTrack motion capture system and SparkFun mecanum-wheeled robots. The real-robot pipeline is tightly coupled to that hardware; if you use different robots or a different MoCap system, you will likely need to adapt topics, robot drivers, and network layout. The key parameters in [`scripts/parameters.py`](scripts/parameters.py) are:

- `VIRTUAL_TO_REAL_ROBOT_MAPPING` — maps virtual agent indices to physical robot IDs.
- `OPTITRACK_TO_MAP_SHIFT` — aligns the MoCap coordinate frame with the map frame.
- `VRPN_IP` — address of the OptiTrack VRPN server.
- `ROS_MASTER_IP` / `REAL_ROBOT_IP` — ROS master and real-robot network addresses.

## Parameter Customization

By default, the system starts in MAPF mode on a 10×10 open map with 8 robots. You can switch scenarios and tune behavior in [`scripts/parameters.py`](scripts/parameters.py).

**General:**
- `DriverParameters.SCENARIO` — set to `0` for discrete MAPF or `1` for continuous traffic.
- `HYBRID_ROBOT_COUNT` — number of robots in the environment.
- `GROUP_COUNT` — number of parallel processing groups.
- `REAL_WORLD_SIZE` — map extents.
- `VRPN_IP`, `ROS_MASTER_IP`, `REAL_ROBOT_IP`, `VIRTUAL_TO_REAL_ROBOT_MAPPING` — networking and hardware configuration.
- `DEBUG=True` — enable verbose logging.
- `OPTITRACK_TO_MAP_SHIFT` — offset MoCap coordinates into the map frame.

### Discrete Space (MAPF)

- `MAPFParameters.WORLD` — occupancy grid; `MAPFParameters.STARTS` — start/goal pairs (or use scenario loaders).
- `SAFETY_DISTANCE` — collision buffer.
- Virtual kinematics: `VIRTUAL_FAST_VEL`, `VIRTUAL_SLOW_VEL`, and associated thresholds.
- Real kinematics: `REAL_FAST_VEL`, `REAL_SLOW_VEL`, `REAL_FAR_THRESHOLD`, `REAL_REACH_THRESHOLD`, `REAL_ANGULAR_VEL`, `ANGULAR_REACH_THRESHOLD`.
- Imitation runs: enable `IMITATE=True`, set `IMITATE_FOLDER` to a recorded scenario, select robots with `IMITATION_LIST`, and cap replay length with `IMITATION_LENGTH`.

### Continuous Space (Traffic)

- `TrafficParameters.ORIGIN` — map anchor point; `FUTURE_TASKS` — look-ahead task queue length.
- Distance thresholds: `FAR_THRESHOLD`, `REACH_THRESHOLD`.
- Virtual velocity window: `VIRTUAL_MIN_VEL` – `VIRTUAL_MAX_VEL`.
- Real velocity caps: `REAL_SLOW_VEL`, `REAL_FAST_VEL`.
- Angular behavior: `ANG_THRESHOLD`, `MAX_ANG_VEL`, `MIN_ANG_VEL`.
- `SPEED_MULTIPLIER` — globally scales all traffic speeds.

## Repository Structure

| Directory | Contents |
|-----------|----------|
| `scripts/` | Main Python drivers, controllers, utilities, and `parameters.py` |
| `launch/` | ROS launch files for the simulation and auxiliary nodes |
| `map/` & `worlds/` | Example maps and Gazebo worlds used in experiments |
| `urdf/` | Robot and goal description files |
| `msg/` | Custom ROS message definitions |
| `rviz/` | RViz configurations for visualizing agents and maps |

## Support

If you encounter issues or have questions, please [open an issue](https://github.com/p3gasuslab/P3GASUS/issues) with a short description, your ROS version, and the steps to reproduce.