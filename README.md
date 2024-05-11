# prometheus_mission

![Static Badge](https://img.shields.io/badge/ROS-noetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus)
![Static Badge](https://img.shields.io/badge/Python-3.8.10-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/Ubuntu-20.04.6-E95420?logo=ubuntu)

The prometheus_mission package, modified from [prometheus_mission](https://github.com/amov-lab/Prometheus/tree/v1.1/Modules/mission)

## Note

- priority of mission is higher than planner, which means that, if drone is close to goal, cmd from planner will not be executed


## Release Note

- v2.0.0: priority of mission exceed planner
- v1.2.1: support `last_angle` from `SetGoal`


## Compilation

```bash
catkin_make install --source Modules/prometheus_mission --build build/prometheus_mission
```
