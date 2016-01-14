# JammsterNavigation
This repository contains a ROS-based implementation of navigation programs for the Jammster

To run map building:
1) go to correct folder (cd ~/cunnia3/navigation_ws)
2) rosrun imu_sensor publishNRF.py
3) rosrun odometry imuOdometryPublisher.py
4) roslaunch freenect_launch freenect.launch 
5) rosrun depthimage_to_laserscan depthimage_to_laserscan image:=/camera/depth/image
6) 
