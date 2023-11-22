# EGO Planner V2.1

This branch is a simple refactor of `main_ws` in EGO Planner V2. No major code modification has been made.

Small packages in the original codebase are combined into two large packages containing multiple nodes, which has the following advantages:
* Faster compilation: because there are less `CMakeLists.txt` to process.
* Cleaner code layout: there are less standalone packages and launch files for demo are all put under the `node_launcher` package.

Please refer to `package.xml` files for details. All credits of the ported packages reamin to original authors.

## Environment Setup

One can follow the steps for system-installed ROS 1 in the EGO Planner V2 repository or use a virtual environment.

```
mamba create -n ego python=3.9 -y
mamba activate ego

mamba install ros-noetic-desktop-full=1.5.0 ros-noetic-joy -c robostack-staging -y
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools armadillo -y

catkin_make
```

## Run

```sh
conda activate ego
source devel/setup.bash

# launch files, e.g.
roslaunch node_launcher multi_drone_interactive.launch
```