# EGO Planner V2.1

This branch represents a streamlined version of the `main_ws` in [EGO Planner V2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2), focusing on enhancing code organization without introducing major modifications.

In this refactor, small packages from the original codebase have been consolidated into two comprehensive packages, each containing multiple nodes. This consolidation offers several advantages:

* Faster Compilation: The reduction in the number of `CMakeLists.txt` files streamlines the compilation process (3x faster on 13900K).
* Cleaner Code Layout: Standalone packages have been minimized, and all demo launch files are now conveniently located under the `node_launcher` package.

For specific details, please refer to the `package.xml` files. It's important to note that all credits for the ported packages remain with the original authors.

## Environment Setup

To set up the environment, follow the steps below. You can either follow the [guide](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/blob/main/swarm-playground/%5BREADME%5D_Brief_Documentation_for_Swarm_Playground.pdf) in the original repository or set up a virtual environment (see below).

```sh
# Create and activate a virtual environment
mamba create -n ego python=3.9 -y
mamba activate ego

# Install ROS and additional dependencies
mamba install ros-noetic-desktop-full=1.5.0 ros-noetic-joy -c robostack-staging -y
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools
mamba install armadillo -y

# Build the workspace
catkin_make
```

## Running Demos

```sh
# Activate the virtual environment and source the setup file
mamba activate ego
source devel/setup.bash

# Run launch files (e.g., multi_drone_interactive.launch)
roslaunch node_launcher multi_drone_interactive.launch
```

## Bibliography

```
@article{
    doi:10.1126/scirobotics.abm5954,
    author = {
        Xin Zhou,
        Xiangyong Wen,
        Zhepei Wang,
        Yuman Gao,
        Haojia Li,
        Qianhao Wang,
        Tiankai Yang,
        Haojian Lu,
        Yanjun Cao,
        Chao Xu,
        Fei Gao
    },
    title = {Swarm of micro flying robots in the wild},
    journal = {Science Robotics},
    volume = {7},
    number = {66},
    pages = {eabm5954},
    year = {2022},
    doi = {10.1126/scirobotics.abm5954}
}
```