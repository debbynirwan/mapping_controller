# mapping_controller
Webots ROS2 Mapping Controller

## Description
The Mapping Controller package starts e-puck robot on Webots Simulator to bounce for mapping the environment.
The maps can be the simple mapping from webots_ros2 or probability mapping.
The maps will be saved at the given path.

## Build
The following instructions assume that a ROS2 workspace is already set up.
```commandline
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --packages-select mapping_controller
. install/setup.bash
```

## Execution
After building and installing the package, you can launch the simulation.

### Simple Mapping with rviz2
```commandline
ros2 launch mapping_controller mapping_controller_launch.py mapper:=true rviz:=true mapping_time:=3
```
mapping_time is the time for the robot to bounce in minute.

### Probability Mapping with rviz2
```commandline
ros2 launch mapping_controller mapping_controller_launch.py probability:=true rviz:=true mapping_time:=3
```

### Specifying path
By default the map is saved at home directory, to change it use path parameter.
```commandline
ros2 launch mapping_controller mapping_controller_launch.py probability:=true rviz:=true mapping_time:=3 path:=path:="/home/$USER/Documents"
```

### Map File Example
the map file looks like:

![alt text](resource/prob_map.png)

## Dependencies
Webots ROS2 package

## Documentation
If you're interested in understanding the details, please read my post [here](https://towardsdatascience.com/occupancy-grid-mapping-with-webots-and-ros2-a6162a644fab)

## Issues
Please report issues if you found bugs or raise a Pull Request.
