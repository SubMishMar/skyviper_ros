# Ardupilot Gazebo ROS Package

## Requirements
- Ubuntu (16.04 LTS), able to run full 3D graphics.
- Gazebo (version 7.x or 8.x)
- ROS Kinetic
- gazebo_ros_pkgs
- mavros

## Installation:

Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)

Set up a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Install [Gazebo and gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros_installing) (included in `ros-kinetic-desktop-full`)

Set up [Ardupilot SITL](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

Install [Mavros](http://wiki.ros.org/mavros)
```
sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
```

Clone and build this repository
```
cd ~/catkin_ws/src
git clone https://github.com/vincekurtz/ardupilot_gazebo
cd ~/catkin_ws
catkin_make
```

Add models to the Gazebo path
```
echo 'export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}${HOME}/catkin_ws/src/ardupilot_gazebo/gazebo_worlds"' >> ~/.bashrc
source ~/.bashrc
```

Copy world to Gazebo
```
sudo cp -a ardupilot_gazebo/gazebo_worlds/. /usr/share/gazebo-7/worlds
```

## Examples

### Basic IRIS simulation

Start the ardupilot simulation
```
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10 -I0
```

Launch the gazebo environment
```
roslaunch ardupilot_gazebo iris_world.launch
```

After confirming a GPS fix, control the the model through the SITL prompt. You can also connect apmplanner2 or your favorite GCS to the simulated model.

```
> arm throttle
> rc 3 1600
> mode POSHOLD
> rc 3 1500

> mode LAND
```

### Simulate an optitrack motion capture system

In the simulation, make sure GPS and compass are off
```
> param set GPS_TYPE 0
> param set EK2_GPS_TYPE 3
> param set COMPASS_USE 0
> param set COMPASS_USE2 0
> param set COMPASS_USE3 0
```

Restart the ardupilot simulation, using the updated parameters
```
sim_vehicle.py -v ArduCopter -f gazebo-iris -m --mav10 -I0
```

Start gazebo with ros
```
roslaunch ardupilot_gazebo iris_world.launch
```

Connect to the simulated ardupilot via mavros and simulate an optitrack system based
on the actual pose from Gazebo. 
```
roslaunch ardupilot_gazebo mavros_optitrack.launch
```

### Explore simple commands with mavros

Start the simulator and launch the gazebo environment (with or without simulated optitrack, see above)

Run the example script:
```
rosrun ardupilot_gazebo mavros_control.py
```

For more information, see `src/mavros_control.py`

## Troubleshooting

### Missing libArduPilotPlugin.so ... etc

In case you see this message when you launch gazebo with demo worlds, check you have no error after sudo make install.  
If no error use "ls" on the install path given to see if the plugin is really here.  
If this is correct, check with "cat /usr/share/gazebo/setup.sh" the variable GAZEBO_PLUGIN_PATH. It should be the same as the install path. If not use "cp" to copy the lib to right path. 

For Example

```
sudo cp -a /usr/lib/x86_64-linux-gnu/gazebo-7.0/plugins/ /usr/lib/x86_64-linux-gnu/gazebo-7/
```

path mismatch is confirmed as ROS's glitch. It'll be fixed.

### Future(not activated yet)
To use Gazebo gps, you must offset the heading of +90° as gazebo gps is NWU and ardupilot is NED 
(I don't use GPS altitude for now)  
example : for SITL default location
```
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-35.363261</latitude_deg>
      <longitude_deg>149.165230</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>87</heading_deg>
    </spherical_coordinates>
```

