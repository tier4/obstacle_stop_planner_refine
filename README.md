# Obstacle stop planner refine

## Prerequirement

- Ubuntu20.04
- ROS foxy

## Setup

```sh
sudo apt install -y python3-vcstool
git clone git@github.com:tier4/obstacle_stop_planner_refine.git
cd obstacle_stop_planner_refine
mkdir -p src
vcs import src < build_depends.repos
```

## Build

```sh
source /opt/ros/noetic/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --catkin-skip-building-tests --packages-up-to obstacle_stop_planner_refine
```
