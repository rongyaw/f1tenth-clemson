# Deep Koopman Representation of Control for autonomous driving in F1TENTH Simulation Environment
# Overview
This repository offer 3 different controllers to fulfill autonomous vehicle control: Pure pursuit, nonlinear model predictive control and deep koopman model predictive control.

# Prerequisite
1. **Go to f1tenth official github page and follow the environment setup instruction: https://github.com/f1tenth/f1tenth_simulator**
2. **Install ROS melodic on ubuntu 18.04: http://wiki.ros.org/melodic/Installation/Ubuntu**
3. **Install required ROS package**
```bash
sudo apt-get install ros-melodic-move-base ros-melodic-navigation ros-melodic-teb-local-planner python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```
4. **Install necessary python package**
```bash
pip3 install numpy cvxpy casadi csv os
```
5. **Build the working directory in python 3**
```bash
roscd && cd ..
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```
6. **Compile workspace with python3**
```bash
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```
7. **Set the racing track to berlin for this experiment**
- Open simulator.launch file in f1tenth_simulator/launch directory
```bash
roscd f1tenth_simulator/launch
nano simulator.launch
```
- **Change the map argument from** ***levine.yaml to berlin.yaml***

8. **Run the predefined rviz environment in the simulator**
- Copy the simulator_2.rviz to f1tenth_simulator/launch directory
- Open the same launch file as in the previous directory and change the simulator.rviz file to simulator_2.rviz at the bottom of the file

<img src = "/f1tenth_simulation.gif" width = "500"><img src = "/augmented_koopman_trajectory.png" width = "300">

# Run controllers
1. **Adaptive Pure pursuit**
```bash
$ roslaunch f1tenth_simulator pure_pursuit.launch
```
2. **Nonlinear MPC based on kinematic bicycle model**
```bash
$ roslaunch f1tenth_simulator kinematic_nonlinear_mpc_global.launch
```
3. **Koopman MPC**
```bash
$ roslaunch f1tenth_simulator koopman_mpc_global.launch
```
