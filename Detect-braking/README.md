# Challenge 2: Detecting and Braking

Cause the vehicle to brake after detecting a person (answers in teams of no more than five, due early Nov; can be done!

## Usage

Open three terminals. In the first terminal, launch the joystick controller

```
source devel/setup.bash
roslaunch basic_launch dbw_joystick.launch
```

In the second terminal, launch the basic sensors

```
source devel/setup.bash
roslaunch basic_launch gnss_sensor_init.launch
```

In the third terminal, directly run `brake.py`

```
python brake.py
```

## Demo
