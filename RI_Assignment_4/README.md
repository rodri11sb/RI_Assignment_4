# SLAM and Navigation in Flatland

This work is an extension of the tutorial [Ros2 Teleopkeys](https://github.com/FilipeAlmeidaFEUP/ros2_teleopkeys_tutorial). We implemented a SLAM and Navigation algorithm for Flatland Simulator using the [slam_toolbox](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html).
The VM and ros workspace setup instructions can be found [here](https://github.com/FilipeAlmeidaFEUP/ros2_teleopkeys_tutorial).

## Pre-requisites

Start by installing the slam_toolbox package, with this command:
```
sudo apt install ros-humble-slam-toolbox
```

Then, because we decided to be running the SLAM algorithm in the online asynchronous mode, copy the parameters file for online async mode from slam_toolbox configurations to the config folder inside "RI_Assignment_4" package:
```
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml ros2_ws/src/RI_Assignment_4/config
```
Make sure you have a config folder inside your package, if not, please create one.

Finally, enter the ros2_ws folder and source the setup.bash file in your terminal:
```
source ~/.bashrc
```
Just to make sure, everything is settled, build the project:
```
colcon build
```

## Runnig the project
Please open 1 more terminal and navigate to the ros2_ws folder. Make sure to source the setup.bash file.
In one of the terminals, run the robot with this command:
```
ros2 launch serp_teleop serp_teleop.launch.py
```

The rviz program will open and you will see the robot spawned inside the maze. By default, the robot will be running randomly. If you want to control it with the keyboard, please take in consideration [this tutorial](https://github.com/FilipeAlmeidaFEUP/ros2_teleopkeys_publisher)

In the other terminal run the SLAM algorithm:
```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/RI_Assignment_4/navigation/config/mapper_params_online_async.yaml use_sim_time:=true
```
Now you will be able to see the robot mapping its surroundings while it is moving.
Note that you will need to adjust the path for your parameters file in the config folder.



