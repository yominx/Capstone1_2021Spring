# Capstone1 2021 Spring

KAIST 2021 Spring Capstone Design 1

This is a repository for capstone design 1 codes. All the codes are just for your reference, so change the code as necessary (e.g. topic name, ball size and color).

To download the package, write the command below in ~/catkin_ws/src terminal.
```console
git clone https://github.com/kaistcapstone/Capstone1_2021Spring.git
```


## ball_detection

Node for detecting red balls.

It publishes the information of ball position and color by calculating geometry between the robot and balls.

#### Usage

```console
rosrun ball_detection ball_detection_node
```



## coppeliasim_models

map_ver_x.ttt, my_robot_ver_x.ttm files

Map and my_robot can be updated later, so please check the notice board on klms regularly.

Just drag and drop files to coppeliasim window.



## core_msgs

A package for defining custom messages used in all codes.

ex) ball_position.msg



## data_integrate

Subscribing lidar(laser_scan) and camera data and do something

You need to change data type of lidar, topic name of both sensors, publishing cmd_vel type to use the node correctly.

ex) /scan -> /laser_scan, sensor_msgs::laser_scan -> sensor_msgs::PointCloud

#### Usage

```console
# data integrate
rosrun data_integrate data_integrate_node
# data show
rosrun data_integrate data_show_node
```



## robot_teleop

Nodes for manually manipulating the gripper and robot wheels.

#### Usage

```console
# gripper
rosrun robot_teleop prismatic_teleop_key
# wheel
rosrun robot_teleop wheel_teleop_key
```



## Tips

- If the prismatic joint fall down, check "Lock motor when target velocity is zero" in Joint Dynamic Properties in CoppeliaSim.
- Click "Toggle real-time mode" when you test your algorithms in simulator.
- ...



## Troubleshooting

- TBA

  

## Contact

Jonghwi Kim <stkimjh@kaist.ac.kr>

Haggi Do <kevindo@kaist.ac.kr>

Kyungseo Kim <chalseokim@kaist.ac.kr>
