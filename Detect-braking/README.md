# Task Description
Cause the vehicle to brake after detecting a person (answers in teams of no more than five.

# Steps
## Step 1
Open the 1st terminal, launch the joystick controller
```
source devel/setup.bash
roslaunch basic_launch gem_dbw_joystick.launch
```
# Step 2

Open the 2nd terminal, launch the basic sensors
```
source devel/setup.bash
roslaunch basic_launch gem_sensor_init.launch
```

# Step 3
```
source devel/setup.bash
rosrun [PackageName] brake.py
```
