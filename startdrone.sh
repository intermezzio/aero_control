roscore &
#cd ~/bwsi-uav/catkin_ws/src/aero-control/nice_on
# play NiceOn1.wav
 
sudo systemctl start aero-teraranger.service
cd ~/bwsi-uav/catkin_ws/src/aero-optical-flow/build
sudo -E ./aero-optical-flow
cd ~/bwsi-uav/catkin_ws
source devel/setup.bash
cd ~/bwsi-uav/catkin_ws/src/aero-control/nice_on
# play NiceApproach1.wav
