This package is a package for ROS2 Humble with the target of implementing Navigation Stack on the Agilex Robots, in particular to Scout Mini, Bunker and Bunker Pro.
Currently the navigation stack in this base configuration is working on the Scout Mini. 

The navigation stack is publish command velocity on **cmd_vel_nav** so you will need to use a *twist_mux* for remapping this to cmd_vel topic. 

### TODO @ 10 / 10:
- [x] Set the correct parameter for all the "services" (amcl, costmaps ecc..) of Nav2
- [ ] Create a BT that is able to switch between indoor and outdoor navigation
- [x] Implement the "Dead man's switch" using twist_mux: joypad and nav2
- [ ] Add controller
- [ ] Add linearization
- [ ] Add Model Predictive Control

Small staff:
- [x] Fix use_sim_time as default to true -> after edit the launch command
- [ ] Add the teleop_joy command.
- [ ] Update the use guide

## Usage

### 1. Download
```bash
mkdir -p <ros2_workspace>/src
cd <ros2_workspace>/src

git clone https://github.com/mattiadutto/scout_navigation.git
```

For using the **simulation** you also need the following packages:
```bash
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/ROSETEA-lab/ugv_gazebo_sim
git clone -b humble https://github.com/ROSETEA-lab/scout_ros2 
```
For using the **real robot** you need the following packages instead:
```bash
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone -b humble https://github.com/ROSETEA-lab/scout_ros2 
```

### 2. Build
```bash
cd ..
colcon build

. install/setup.bash
```

### 3. NAVIGATION: nav2.launch.py
This are usage example, the used file name or parameter values can be different in your case.

#### Usage
```bash
# Simulation case of nav2.launch.py
ros2 launch scout_navigation nav2.launch.py \
      namespace:=scout_mini \
      map_name:=workshop_big_empty_slam.yaml \
      rviz_params_file:=scout_mini_navigation.rviz

# Real robot of nav2.lauch.py
ros2 launch scout_navigation nav2.launch.py use_sim_time:=False map_name:=velodyne_andata_5_destra.yaml nav2_params_file:=nav2_params_scout_mini.yaml rviz_params_file:=scout_mini_robot.rviz 
```

#### Parameters 
- **use_sim_time**: usage of simulation time or not, default *true*
- **use_rviz**: usage of RVIZ2 for telemetry operation, default *true*
- **map_name**: name of the map that will be loaded into Nav2 stack present into the maps folder, default *slam_farm.yaml*
- **namespace**: namespace of the robot, default *empty*
- **ekf_params_file**: configuration file for the Extended Kalman Filter, present in the config folder of this package, default *ekf_localization_with_gps.yaml*
- **nav2_params_file**: configuration file for Navigation 2 stack, present in the config folder of this package, default *nav2_params.yaml*
- **rviz_params_file**: configuration file for RVIZ2, present in the config folder of this package, default *scout_mini_navigation.yaml*

### 4. NAVIGATION: gps.launch.py
TODO: still to test in an extensive way the fusion of data between: GPS / IMU / Robot Odometry.

### 5. MAPPING: slam_offline.launch.py
This launch file can allow you to map an environment that can be later utilized for the navigation stack. This allow us to create a map starting from **pre-recorded** data.

#### Usage
```bash
# Real robot
ros2 launch scout_navigation slam_offline.launch.py
```

#### Parameters
- **namespace**: namespace of the robot, default *empty*
- **slam_params_file**: configuration file for SLAM Toolbox, present in the config folder of this package, default *mapper_params_offline.yaml*
- **autostart**: automatically startup the SLAM Toolbox, ignored when use_lifecycle_manager is true, default *true*
- **use_lifecycle_manager**: enable bond connection during node activation, default *false*

### 6. MAPPING: slam_online_async.launch.py
This launch file can allow you to map an environment that can be later utilized for the navigation stack. This allow us to create a map from **live** data.

#### Usage
```bash
# Real robot
ros2 launch scout_navigation slam_online_async.launch.py use_sim_time:=False
```

#### Parameters
- **namespace**: namespace of the robot, default *"empty"*
- **use_sim_time**: usage of simulation time or not, default *true*
- **slam_params_file**: configuration file for SLAM Toolbox, present in the config folder of this package, default *mapper_params_online_async.yaml*
- **scan**: laser scan remapping of the robot, default *scan*
- **tf**: tf remapping of the robot, default *tf*
- **tf_static**: tf static remapping of the robot, default *tf_static*

