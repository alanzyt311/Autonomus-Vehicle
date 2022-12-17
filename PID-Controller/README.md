# Description
In this HW, we implemented PID controller to follow waypoints along a path on the simulator. The codes file we modified include `pid.py` and `follow_waypoints.py`.
# Steps
## Step 1. Launch track 1 environment in the simulator
```
source devel/setup.bash
roslaunch gem_launch gem_init.launch world_name:="track1.world" x:=0 y:=-2
```

## Step 2. Run follow_waypoints file to let the car move in the middle of the road
```
source devel/setup.bash
rosrun gem_waypoint_pid follow_waypoints.py
```