# EGO Planner V2.1

This branch builds upon the `main_ws` branch, providing the following new features.

Dec 23, 2023 update:
* Support for terrestrial drones
* Angular Motion (non-holonomic) constraint
* Heterogeneous swarm coordination
* Habitat simulator integration
* Path guided rebound & replan

Please refer to [documents](./doc/) for detailed description of new features and code modifications.

The `main_ws` branch represents a streamlined version of the `main_ws` in [EGO Planner V2](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2), focusing on enhancing code organization and compilation without introducing major modifications.

## Environment Setup

To set up the environment, follow the steps below. We recommend using a virtual environment. But you can also use the system-installed ROS and install `habitat-sim` using source or `pip` associated with the ROS Python interpreter.

```sh
# System dependencies (Ubuntu)
sudo apt install libgl1* 

# Create and activate a virtual environment
mamba create -n ego python=3.9 -y
mamba activate ego

# Install ROS related stuff
mamba install ros-noetic-desktop-full ros-noetic-joy -c robostack-staging -y
mamba install compilers cmake pkg-config make ninja colcon-common-extensions catkin_tools -y

# Other handy tools
mamba install habitat-sim=0.2.4 -c aihabitat -y
mamba install armadillo casadi=3.6.3 ompl=1.5.2 -y

# Install Open3D
pip install open3d==0.17.0

# Install acados
cd $CONDA_PREFIX
git clone https://github.com/acados/acados.git
cd acados
git checkout v0.2.6
git submodule update --init --recursive
mkdir build
cd build
cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON -DACADOS_INSTALL_DIR=$CONDA_PREFIX ..
make install -j 16
cd ..
pip install interfaces/acados_template
```

Now change working directory to `src/ego_planner/scripts` to generate acados code for diffdrive MPC.

```sh
mamba activate ego
python diffdrive_acados.py
```

Go back to the top repo directory and build to workspace.

```sh
mamba activate ego
catkin_make
```

## Running Demos

```sh
# Activate the virtual environment and source the setup file
mamba activate ego
source devel/setup.bash

# Run launch files in the node_launcher package (e.g. single_diffdrive_interactive_habitat.launch)
roslaunch node_launcher single_diffdrive_interactive_habitat.launch
```

## Known & Potential Issues

1. The `SelectedPointsPublisher` RViz plugin does not work well within the virtual environment.

2. Node `moving_obstacles` and `ego_planner/launch/obstacle_run.launch` are not tested due to lack of joy stick. `ego_planner/launch/drone_detect.launch` is also not tested.

3. Aerial agents will only avoid other terrestrial agents horizontally. No flying-over behaviour.

4. Optimizer can fail if free space is blocked by a close-up and slowly-moving agent, when operating in narrow passages. Set identical maximum velocity to relieve this problem.

5. 3D dynamic A* implemented in this repo tends to slow down the entire navigation process and result in crashes. Set reasonable goals when using an aerial robot.

6. Failed planning at the end of the current trajectory may occasionally cause crashes.

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