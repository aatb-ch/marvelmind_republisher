# Marvelind Robotics hedgehog pose/imu republisher

Publish Marvelmind Robotics hedgehog's serial data as PoseWithCovarianceStamped and Imu ROS topics for use with localization packages.

# Use

- install marvelmind_nav per instructions in your workspace src folder
- clone this repo in your workspace src folder
- run marvelmind's ros node to publish custom topics `rosrun marvelmind_nav hedge_rcv_bin`
- run this node to republish custom messages as standard ROS topics

`roslaunch marvelmind_republisher hedgehog.launc`