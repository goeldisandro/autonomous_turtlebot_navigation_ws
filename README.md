# autonomous_turtlebot_navigation
Autonomous Turtlebot Navigation with: Turtlebot (drive and odom), RPLidar(laserscan), BeagleBone Blue(IMU), UP Squared(Processing Unit)

# Needs
Ubuntu 16.04 \
working ROS kinetic branch \
Hardware as described under: 
https://wiki.ntb.ch/stud/2019_ba_tbosser_sgoeldi/start

# installation
go to home directory \
git clone https://github.com/goeldisandro/autonomous_turtlebot_navigation_ws.git \
cd autonomous_turtlebot_navigation_ws \
catkin build \
src devel/setup.bash 

# example launch
roslaunch kobuki_nav online_navigation.launch 
