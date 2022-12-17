# Task Description
Under control of the computer, cause the vehicle to flash the SOS pattern on its turn indicator lights. SOS in morse code is dot-dot-dot-dash-dash-dash-dot-dot-dot. Map dot to the left turn indicator and dash to the right, and make the vehicle flash: left-left-left-right-right-right-left-left-left followed by a wait, then repeat.

# Steps
## Step 1
Open the 1st terminal, launch the joystick controller
```
source devel/setup.bash
roslaunch basic_launch gem_dbw_joystick.launch
```

## Step 2
Open the 2nd terminal
```
source devel/setup.bash
rosrun [PackageName] flashing.py
```

