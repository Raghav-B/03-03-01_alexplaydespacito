export LC_ALL="en_SG.utf8"
gnome-terminal -- bash -c "ssh pi@192.168.43.232 -t 'source /opt/ros/kinetic/setup.bash; source ~/Desktop/pi_workspace/devel/setup.bash; rosrun alex_main_pkg main_node;'"
