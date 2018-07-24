# Flight Startup Sequence

1.  In terminal 1, start distance sensor
	```
    sudo systemctl start aero-teraranger.service
	```

2.  Ensure that log isn't being spammed (run this a few times over a few seconds and ensure that it is not growing by more than 1MB per second)
	```
	sudo du -sh /var/log
	```
	+ If log is being spammed/blowing up, stop service, truncate log, and restart
	```
    sudo systemctl stop aero-teraranger.service
    sudo truncate -s 0 syslog
	```

1.  In terminal 2, start ROS
	```
	roscore
	```

4.  In terminal 3, start _patched_ optical flow and down-camera streaming.
    Note: this is different from _standard_ optical flow service that prevents streaming downward-facing camera.
    Needed for line detection.
    [See here for more info on setting up](https://github.mit.edu/ma23705/aero_downward_ros/blob/master/README.md)
	```
	cd ~/bwsi-uav/catkin_ws/src/aero-optical-flow/build
	sudo -E ./aero-optical-flow
	```

5.  Without arming drone, switch to `POSITION CONTROL` mode to ensure previous steps worked as expect.
    If QGroundControl declares that that position control is rejected, restart process.

6.  In terminal 4, launch mavros, line detector, and line tracker nodes 
	```
	cd ~/bwsi-uav/catkin_ws
	source devel/setup.bash
	roslaunch aero_control detect_track.launch
	```
6.  In `POSITION CONTROL` mode arm, takeoff, position over LED path, then switch to `OFFBOARD` mode to run test

7.  _For visualization_ run `rqt_image_view` in a new terminal (assumes you used `ssh -X` or `ssh -Y` when connecting to drone for graphics forwarding)