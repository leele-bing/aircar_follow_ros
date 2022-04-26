#! /bin/bash

gnome-terminal --tab --title "us" -- sudo bash -c "source ~/hy_ws/devel/setup.bash; 
roslaunch ultra_serial ultra.launch"
sleep 3s
gnome-terminal --tab --title "uwb" -- sudo bash -c "source ~/hy_ws/devel/setup.bash; 
roslaunch uwb_nooploop_aoa_serial uwb.launch"
