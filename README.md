# README.md

## Starting command

For main mission, open use command below on the terminal. our model is **coppeliasim_models/teamE_robot_final.ttm**

```bash
roslaunch data_integrate capstone_launch.launch
```

For bonus mission, type below command.

```bash
roslaunch data_integrate capstone_launch_bonus.launch
```

## Instructions regarding manual transition

We use "/zone" topic to activate nodes for ball harvesting zone. however, data_integration_node publishes "/zone" topic periodically, so just publishing "/zone" topic manually doesn't work for our system.

Instead, just follow below steps to use ball harvesting zone control.

1. Move robot on the step obstacle in the entrance zone like below picture. 
    <p align="center"><img src="harvest_debug_img.png" width="70%" /><br><br></p>

2. set **DEBUG_HARVEST** as **true**. It is defined in **data_integrate/data_integration.cpp,** line 83 (approximately).
3. Type same command as before: 

```bash
roslaunch data_integrate capstone_launch.launch
```

## Required library list

In main mission, we use basic libraries such as std::msgs, opencv, etc. However, in bonus mission, we use icp algorithm in pcl library, so it requires pcl_conversions, pcl_ros library.

Perhaps all the packages are included in the ros full package, so if the simulation environment has a full ros noetic package, no additional dependency are required.
