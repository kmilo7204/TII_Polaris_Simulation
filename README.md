# TII_Polaris_Simulation
Challenge solution

## Container
The solution includes a container with Ubuntu 20.04, Nvidia capabilities and ROS Noetic, desktop full.

### Build
Build the container with the following command:

```bash
docker build -t ros-gazebo-nvidia-tii:20.04 .
```

### Run
Run the container with the provided bash script `run.sh`

```bash
./run.sh /home/ekumen/Camilo_Repos/TII_Polaris_Simulation/catkin_ws/src
```

## Project structure
The project contains 4 main folders:
1. Polaris simulation
  It was brought as a subtree, since a little modification was needed to draw the path in the simulation.

2. Path Generator
  A ROS node, that reads a `CSV` file and published the found path. A service is running to get a single Trigger request to executed the mentioned actions.

3. Trajectory Navigation
  A set of ROS nodes that includes the all the functionality to do path tracking. Currently the `Pure Pursuit` and the `Stanley` path trackers are implemented. In addition to it, there is a Navigation state machine that can easily run any of the trackers that follow the structure provided by the interface `PathTracker`.

4. Gazebo Plugins
  Gazebo plugins to interact with the models or with the world, currently only onw was implemented to draw a path, when the subcriber to the topic `/path` receives any information.

## Launch Robot stack
The `navigation.launch` is provided under the `trajectory_navigation` package. The launch file will launch all the navigation stack, as well as the path generator node, and the Polaris simulation. To launch all the stack, run the following command:

```bash
roslaunch trajectory_navigation navigation.launch
```

- To generate the path, call the following service from another terminal:

```bash
rosservice call /generate_path
```

Since the main repository for this project was brought as a subtree, feel free to visit it [here](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2).

## Tests
To run the tests:
```bash
catkin_make run_tests
```
