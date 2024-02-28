This package is a package for ROS2 Humble with the target of implementing Navigation Stack on the robot.
Currently the navigation stack in a base configuration is working. 

### TODO @ 15 / 02:
- [x] Set the correct parameter for all the "services" (amcl, costmaps ecc..) of Nav2
- [ ] Create a BT that is able to switch between indoor and outdoor navigation
- [x] Implement the "Dead man's switch" using twist_mux: joypad and nav2
- [ ] Add controller
- [ ] Add linearization
- [ ] Add Model Predictive Control

Small staff:
- [ ] Fix use_sim_time as default to true -> after edit the launch command
- [ ] Add the teleop_joy command.
- [ ] Update the use guide

## How to use the stack
If you don't have set up the Scout Mini for simulation you will need to download also this repositories:
```bash
  git clone https://github.com/bascetta74/ugv_gazebo_sim.git
  git clone https://github.com/agilexrobotics/ugv_sdk.git
  git clone -b humble https://github.com/bascetta74/scout_ros2.git
```

For launching the Gazebo simulation:
```bash
  ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py \
      use_rviz:=false \
      world_name:=<name_of_the_world>.world \
      use_sim_time:=true
```
You will have to put the correct name of the Gazebo world instead of _<name_of_the_world>_.
Now we can activate the navgiation package.
```bash
  ros2 launch scout_navigation nav2.launch.py use_sim_time:=true
```
Last thing is to activate the teleop for control the system with the joypad
```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/scout_mini/cmd_vel
```
