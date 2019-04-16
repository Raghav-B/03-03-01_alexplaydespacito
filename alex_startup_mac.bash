export LC_ALL="en_SG.utf8"
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t 'export DISPLAY=:0; ./Desktop/pi_workspace/src/03-03-01_alexplaydespacito/pi_bash_files/roscore.bash' && bash"
sleep 10s
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t 'source /opt/ros/kinetic/setup.bash; source ~/Desktop/pi_workspace/devel/setup.bash; rosrun alex_main_pkg main_node;'"
sleep 10s
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t 'export DISPLAY=:0; ./Desktop/pi_workspace/src/03-03-01_alexplaydespacito/pi_bash_files/camera_node.bash' && bash"
sleep 10s
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t 'export DISPLAY=:0; ./Desktop/pi_workspace/src/03-03-01_alexplaydespacito/pi_bash_files/lidar_node.bash' && bash"
sleep 20s
gnome-terminal -- bash -c "export ROS_MASTER_URI=http://192.168.43.232:11311 && export ROS_IP=192.168.43.96 && rosrun rviz rviz -d ~/despacito_ros/src/03-03-01_alexplaydespacito/rplidar_ros/rviz/slam.rviz && bash"
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t '/usr/bin/tvservice -o; /home/pi/hub-ctrl -h 0 -P 1 -p 0;'"
