# autonomous_turtlebot_navigation
Autonomous Turtlebot Navigation with: Turtlebot, RPLidar, BeagleBone Blue, UP Squared

#Needs
Ubuntu 16.04
working ROS kinetic
Hardware as described under:
https://wiki.ntb.ch/stud/2019_ba_tbosser_sgoeldi/start

#installation
go to home directory
git clone https://github.com/goeldisandro/autonomous_turtlebot_navigation.git
cd autonomous_turtlebot_navigation
catkin build
src devel/setup.bash

#example launch
roslaunch kobuki_nav online_navigation.launch 
