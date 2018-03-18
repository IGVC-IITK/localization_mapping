## scanignore

**scanignore node is no longer needed. Instead, this functionality has now been merged into rplidar_ros. (This reduces delay.)**

It generates filtered scans from the laser scan topics by removing range data that corresponds to points on the robot itself (so that the robot does not map itself).

Configured for the Firbird 0xDelta (Daksh IITK).