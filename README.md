# Dynamic Obstacle Detector

A simple (but effective) detector of dynamic obstacles in laser scans. 
It detect groups of points in a laser scan that are then tracked using Kalman Filters. Obstacles that do not exceed a minimum velocity threshold are removed. 

## Parameters

- **input_scan_topic**. Name of the scan topic. (Def: "scan").
- **odom_frame**. Name of the odometry frame. (Def: "odom").
- **cluster_max_distance_points**. Maximum distance [m] between consecutive points in the scan to be grouped together. (Def: 0.6).
- **cluster_min_points**. Minimun number of points in a group to be considered as an obstacle. (Def: 3)
- **cluster_max_points**. Maximum number of points in a group to be considered as an obstacle. (Def: 35)
- **min_vel_tracked**. Minimum velocity [m/s] of the obstacles to be considered as a dynamic obstacle. (Def: 0.35).
- **max_vel_tracked**. Maximum velocity [m/s] of the obstacles to be considered as a dynamic obstacle. (Def: 2.0).
- **max_tracked_distance**. Maximum distance [m] between obstacles to be considered the same one. (Def: 0.55).
- **max_tracked_sec**. Maximum time [sec] between detections of an obstacle to be tracked. (Def: 1.0).


## Subscriptions

- */scan*: <sensor_msgs::LaserScan>. Laser scan topic. Modified by the parameter **input_scan_topic**.


## Publications

- */dynamic_obstacles/static* <visualization_msgs::MarkerArray>. RViz marker with the dynamic obstacles candidates.
- */dynamic_obstacles/dynamic* <visualization_msgs::MarkerArray>. Rviz marker with the dynamic obstacles detected.


## ToDo:

- Publishing messages with the obstacles (people_msgs or custom messages).


