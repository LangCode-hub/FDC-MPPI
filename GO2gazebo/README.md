# FDC-MPPI Gazebo Simulation

This directory contains the ROS/Gazebo simulation code accompanying the paper
**“FDC-MPPI: Fast Disturbance-Compensated MPPI for Onboard Quadruped
Navigation in Cluttered and Disturbed Environments.”**

- Project page: <https://langcode-hub.github.io/FDC-MPPI/>
- Repository: <https://github.com/LangCode-hub/FDC-MPPI>

The release provides the MPPI-based local obstacle-avoidance simulation used
with a Unitree GO2 model. It includes laser-scan costmap construction, an
Euclidean distance transform (EDT), MPPI trajectory sampling and visualization,
the quadruped controller, and a Gazebo disturbance-force test.

## Repository layout

```text
GO2gazebo/
├── src/
│   ├── go2_costmap_demo/       # LaserScan -> local costmap and EDT
│   ├── unitree_guide/
│   │   └── unitree_guide/      # MPPI and quadruped-control integration
│   ├── unitree_legged_msgs/    # Unitree ROS messages
│   └── unitree_ros/
│       ├── robots/go2_description/
│       ├── unitree_controller/
│       ├── unitree_gazebo/
│       └── unitree_legged_control/
├── .catkin_workspace
└── README.md
```

The main paper-related implementation is located in:

- `src/unitree_guide/unitree_guide/src/control/MPPI.cpp`
- `src/unitree_guide/unitree_guide/include/control/MPPI.h`
- `src/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp`
- `src/go2_costmap_demo/src/laser_to_costmap.cpp`

## Requirements

The code is intended for Linux with ROS 1 and Gazebo. The tested-style setup is:

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- CMake 3.14 or newer and a C++14 compiler
- Eigen3, OpenCV 4, LCM, and the ROS packages declared in each `package.xml`

Install ROS Noetic following the official ROS instructions, then install the
workspace dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake libeigen3-dev liblcm-dev libopencv-dev \
  ros-noetic-cv-bridge ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control \
  ros-noetic-laser-geometry ros-noetic-navigation ros-noetic-robot-state-publisher \
  ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-tf2-geometry-msgs

cd GO2gazebo
rosdep install --from-paths src --ignore-src -r -y
```

Equivalent ROS 1 distributions may work but have not been verified for this
release.

## Build

```bash
cd GO2gazebo
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Run the simulation

Use three terminals. Source ROS and the workspace in every terminal.

Terminal 1 — start Gazebo and the GO2 model:

```bash
cd GO2gazebo
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

Pass `rviz:=false` if RViz is not needed.

Terminal 2 — build the local costmap and EDT from the simulated laser scan:

```bash
cd GO2gazebo
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun go2_costmap_demo laser_to_costmap
```

Terminal 3 — start the controller:

```bash
cd GO2gazebo
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

Focus Terminal 3 and use the following keys:

| Key | Action |
| --- | --- |
| `2` | Switch from Passive to FixedStand |
| `4` | Switch from FixedStand to Trotting and enable MPPI navigation |
| `3` | Apply the configured Gazebo disturbance force |
| `1` | Return to Passive |
| Space | Clear manual command input |

The default navigation goal is `(6.1, 2.0)` metres. Change it in
`State_Trotting::enter()` in
`src/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp` and rebuild.

## ROS interfaces

The main interfaces are:

| Topic/service | Type/purpose |
| --- | --- |
| `/go2/laser/scan` | Laser scan input |
| `/laser_to_costmap/costmap` | Local occupancy grid |
| `/laser_to_costmap/edt_map` | Float32 EDT image |
| `~mppi_trajectories` | Sampled trajectories (`MarkerArray`) |
| `~mppi_optimal_trajectory` | Selected trajectory (`Marker`) |
| `/gazebo/apply_body_wrench` | Disturbance-force service |

## Configuration

Core MPPI parameters such as the sample count, horizon, control bounds, noise,
temperature, and cost weights are initialized in the `MPPI` constructor in
`MPPI.cpp`. Costmap resolution and inflation parameters are defined in
`laser_to_costmap.cpp`. Rebuild the workspace after changing C++ parameters.

## Notes

- This is research code intended for simulation and reproducibility, not a
  safety-certified controller.
- The released controller uses Gazebo model-state feedback and expects the
  simulated model name `go2_gazebo`.
- Start the costmap node before enabling autonomous navigation. Until an EDT is
  available, obstacle-distance queries cannot represent the environment.
- The repository intentionally contains only the GO2-related portion of the
  upstream Unitree ROS assets to keep the release focused and reasonably sized.

## Citation

If this code is useful in your research, please cite:

```bibtex
@article{cai2026fdcmppi,
  title   = {FDC-MPPI: Fast Disturbance-Compensated MPPI for Onboard Quadruped Navigation in Cluttered and Disturbed Environments},
  author  = {Cai, Jialang and Lu, Jiaheng and Su, Jinya and Huang, Lingying and Li, Shihua},
  year    = {2026}
}
```

Please replace the entry above with the final proceedings citation when it is
available.

## License and acknowledgements

The original code in this release is provided under the BSD 3-Clause License;
see `LICENSE`. Portions of `unitree_guide`, `unitree_ros`, and
`unitree_legged_msgs` are derived from Unitree Robotics projects and retain
their upstream copyright notices and licenses. See the license files inside
the corresponding directories for details.

