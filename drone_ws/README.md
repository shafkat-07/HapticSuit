# Drone Workspace

This includes the code used to fly the drone. It is based on the [Freyja controller](https://github.com/unl-nimbus-lab/Freyja).

To run the code you first need to install ROS Noetic. Then you can use the following to compile the workspace

```bash
cd drone_ws
catkin_build
```

Once everything is build you need to source your workspace using:

```bash
source devel/setup.bash
```

Finally you will be able to run each of the scenarios. To do that you can use the following:

```bash
# Hover the drone 1m above the origin
roslaunch flight_controller hover_origin.launch 
# Fly back and forth 1m along the origin
roslaunch flight_controller oscilate_straight.launch 
```