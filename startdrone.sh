roscore &

echo hi
sudo systemctl start auto-teraranger.service
cd ~/bwsi-uav/catkin_ws/src/aero-optical-flow/build
sudo -E ./aero-optical-flow
cd ~/bwsi-uav/catkin_ws
source devel/setup.bash