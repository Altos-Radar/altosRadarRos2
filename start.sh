source install/local_setup.bash
gnome-terminal -- bash -c "ros2 launch rviz_launch.py;read"
sleep 3s
gnome-terminal -- bash -c "ros2 run altosradar altosRadarParse;read"
