# EasonDrone_Mission

The easondrone_mission package.

![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)


## Note

- priority of mission is higher than planner, which means that, if drone is close to goal, cmd from planner will not be executed


## Compilation

```bash
catkin_make install --source Modules/EasonDrone_Mission --build Modules/EasonDrone_Mission/build
```


## Release Note

- v2.0.2: replace `MIN_DIS` with `control_yaw_flag`
- v2.0.1: skip if planner not initialized
- v2.0.0: priority of mission exceed planner
- v1.2.1: support `last_angle` from `SetGoal`
- 

## Acknowledgement

Thanks to following packages:

- [prometheus_mission](https://github.com/amov-lab/Prometheus//Modules/mission)