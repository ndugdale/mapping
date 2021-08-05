# Winter Research 2021: Marine Robotics
###### Using OctoMap to create 3D occupancy grid maps from past AUV Sirius data.

## Summary

This repository provides a ROS package called 'mapping'. This package contains a node called 'rosbag_from_csv.py', which can be run to convert nav, oas and rdi csv data files into rosbags for simulation. These rosbag files can then be launched to replay with an octomap_server node from the OctoMap ROS package to generate a 3D occupancy grid map.

## Dependencies

Ensure catkin, Python 3 and ROS noetic are installed. This package was developed and tested on Ubuntu 20.04.2.0 using Python 3.8.10 and ROS noetic.

<details>
  <summary>ROS dependencies</summary>
  
- octomap_server
- rospy
- rospkg
- rosbag
- ros_numpy
- std_msgs.msg
- geometry_msgs.msg
- sensor_msgs.msg
- tf2_msgs.msg
- tf_conversions
</details>

<details>
  <summary>Python dependencies</summary>
  
- numpy
- pandas
- os
- math
</details>

## Setup

Navigate to where you would like to create your catkin workspace. Make a directory to be the root of your catkin workspace. Change into your catkin workspace directory and make a directory named 'src'.
```
$ mkdir catkin_ws
$ cd catkin_ws
$ mkdir src
```

Run 'catkin_make'. This will create 'build' and 'devel' directories.
```
$ catkin_make
```

Change into the 'src' directory and clone the git repository.
```
$ cd src
$ git clone https://github.com/ndugdale/mapping.git
```

Return to the root of your catkin workspace and run setup before using the package.
```
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## Usage

Navigate to the root directory of the catkin workspace, 'catkin_ws'. Run roscore in terminal.
```
$ roscore
```

In a new terminal tab or window, nodes or launch files from the mapping package can then be run. Make sure that these new tabs and windows have been navigated to the root directory of the catkin workspace.

###### Generating a rosbag from csv data
Open '~/catkin_ws/src/mapping/scripts/rosbag_from_csv.py' in a text editor. There is a look-up table, called 'data_name_list' containing the names of datasets for which rosbags can be generated. The variable 'data_name' contains a single element from this look-up table for use. Choose which dataset you wish to generate a rosbag for by changing the index accessed from the look-up table. E.g. the following would generate a rosbag for the 'scott_reef' dataset.
```python
...

data_name = data_name_list[23]

...
```

Save the file, and then the rosbag is generated upon running the 'rosbag_from_csv.py' node.
```
$ rosrun mapping rosbag_from_csv.py
```

The rosbag is then written into the directory ~/catkin_ws/src/mapping/rosbags/ .

###### Running simulation with OctoMap
Open '~/catkin_ws/src/mapping/launch/sirius_sim.launch' in a text editor. Change the default value of the arg tag to be the name of the dataset you would like to simulate. E.g. the following would simulate a rosbag for the 'scott_reef' dataset. Alternatively, the argument can be set using the command-line.
```xml
<launch>
  <arg name="data_name" default="scott_reef"/>

...

</launch>
```

Save the file, and then the simulation is run upon running the 'sirius_sim.launch' launch file.
```
$ roslaunch mapping sirius_sim.launch
```

When the simulation is complete, if you would like to save the OctoMap as a .bt file **DO NOT** exit the program. Open a new tab or window and make sure you have navigated to the root directory of the catkin workspace, 'catkin_ws'. Then run the following command to save the .bt file.
```
$ rosrun octomap_server octomap_saver file_name.bt
```

To assist in the storage and management of .bt files, a message will print in terminal to suggest an appropriate file name. Please ensure **NOT** to accidentally press CTRL + C whilst copying and pasting the suggested file name. The provided message can be changed manually in '~catkin_ws/src/mapping/scripts/save_helper.py'.

