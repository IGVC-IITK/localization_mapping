# Localization and Mapping
Overview of the subsystem:

![SLAM Overview](SLAM_ROS_Stack.png)

## cartographer_ros
[Cartographer](https://github.com/googlecartographer/cartographer) is a system that provides real-time simultaneous localization and mapping ([SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)) in 2D and 3D across multiple platforms and sensor configurations.

![Cartographer SLAM Example](https://j.gifs.com/wp3BJM.gif)

It is an open-source package. Only the parameters have been altered.

_Status: Parameters yet to be tuned properly_

## img_to_laser

This node gives laser scan as read by a virtual LiDAR put at some position on binary classified image. Used for mapping the data obtained from the vision pipeline via mapping nodes that require data to be in laser scan form only.

![Image to Laser-Scan Example](img_to_laser/img_to_laser.png)

Built from scratch.

_Status: Tested and working_

## robot_localization

This node gives sensor-fused odometry. (Currently using wheel encoders, visual odometry and an IMU).

It is an [open-source](https://github.com/cra-ros-pkg/robot_localization) ROS package. Added an extra feature. Measured, calculated and tuned covariance matrices based on the robot dynamics, sensor configuration and the environment conditions.

_Status: Tested and working_

## ros0xrobot

This node gives raw wheel odometry (and also provides a low-level controller) for the Firebird 0xDelta robot. This node is basically a driver for the robot's motors and encoders.

Acquired from NEX Robotics. Added some fixes after that.

_Status: Tested and working_

## scanignore

This node generates filtered scans from the laser scan topics by removing range data that corresponds to points on the robot itself (so that the robot does not map itself).

Built from scratch. Configured for the Firbird 0xDelta.

_Status: Tested and working_

## tf_calibration

This is a **one-time** node used for calibrating sensor frames. It just publishes a tf that can be updated in real time using dynamic_reconfigure. One can visualize the sensor data on Rviz and then tune the tf till the data matches visually. Was used for LiDAR-LiDAR calibration on the Firebird 0xDelta.

![tf Calibration Example](tf_calibration/tf_calibration.png)

Built from scratch.

_Status: Tested and working_