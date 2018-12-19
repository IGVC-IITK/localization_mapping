#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

float resolution = 0.05;
int map_width = (int)(400.0/resolution);
int map_limit = map_width*map_width;
float map_origin_position[3] ={-200.0, -200.0, 0.0};
char* map_array = new char[map_limit];
geometry_msgs::TransformStamped lidar1_to_map, lidar2_to_map;

void scanToMap(const sensor_msgs::LaserScan &LaserScan)
{
	geometry_msgs::PointStamped point_lidar, point_robot;
	float angle_min = LaserScan.angle_min, angle_max = LaserScan.angle_max,
		angle_increment = LaserScan.angle_increment, angle_lidar = 0.0, range_current = 0.0;
	int x = 0, y = 0, i = 0;

	for(i = 0; angle_min+angle_increment*i <= angle_max; i++)
	{
		angle_lidar = angle_min+angle_increment*i;
		if ((angle_lidar > -M_PI/2 && angle_lidar < M_PI/2) ||
			(LaserScan.ranges[i] < LaserScan.range_min || LaserScan.ranges[i] > LaserScan.range_max))
			continue;
		
		for (range_current = 0.0; range_current < LaserScan.ranges[i]; range_current += resolution)
		{
			point_lidar.point.x = range_current*cos(angle_lidar);
			point_lidar.point.y = range_current*sin(angle_lidar);
			point_lidar.point.z = 0;	// Scanning plane is same as LiDAR's xy plane.
			if (LaserScan.header.frame_id == "laser_front")
				tf2::doTransform(point_lidar, point_robot, lidar1_to_map);
			else
				tf2::doTransform(point_lidar, point_robot, lidar2_to_map);
			x = (int)((point_robot.point.x - map_origin_position[0])/resolution);
			y = (int)((point_robot.point.y - map_origin_position[1])/resolution);
			if ((0 < y*map_width + x) && (y*map_width + x < map_limit))
				map_array[y*map_width + x] = 0;
		}
		point_lidar.point.x = LaserScan.ranges[i]*cos(angle_lidar);
		point_lidar.point.y = LaserScan.ranges[i]*sin(angle_lidar);
		point_lidar.point.z = 0;		// Scanning plane is same as LiDAR's xy plane.
		if (LaserScan.header.frame_id == "laser_front")
			tf2::doTransform(point_lidar, point_robot, lidar1_to_map);
		else
			tf2::doTransform(point_lidar, point_robot, lidar2_to_map);
		x = (int)((point_robot.point.x - map_origin_position[0])/resolution);
		y = (int)((point_robot.point.y - map_origin_position[1])/resolution);
		if ((0 < y*map_width + x) && (y*map_width + x < map_limit))
			map_array[y*map_width + x] = 100;
	}

	return;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_mapping_node");
	ros::NodeHandle n;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Subscriber sub_scan_1 = n.subscribe("/scan_1", 1, scanToMap);
	// ros::Subscriber sub_scan_2 = n.subscribe("/scan_2", 1, scanToMap);

	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("/lidar_map", 1);

	int i = 0;
	nav_msgs::OccupancyGrid lidar_map;
	lidar_map.header.frame_id = "odom";
	lidar_map.info.width = map_width;
	lidar_map.info.height = map_width;
	lidar_map.info.resolution = resolution;
	lidar_map.info.origin.position.x = map_origin_position[0];
	lidar_map.info.origin.position.y = map_origin_position[1];
	lidar_map.data.resize(map_limit);
	for(i = 0; i < map_limit; i++)
		map_array[i] = -1;

	ros::Rate loop_rate(10);
	int loop_count = 0;
	while(ros::ok())
	{
		try
		{
			// For transforming the coordinates of a fixed point from frame_a to frame_b, we need
			// to use lookupTransform("frame_b", "frame_a",ros::Time(0))
			lidar1_to_map = tfBuffer.lookupTransform("odom", "laser_front", ros::Time(0));
			// lidar2_to_map = tfBuffer.lookupTransform("odom", "laser_rear", ros::Time(0));
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		ros::spinOnce();
		if (loop_count%30==0)
		{
			for(i = 0; i < map_limit; i++)
				lidar_map.data[i]=map_array[i];
			pub_map.publish(lidar_map);
		}
		
		loop_rate.sleep();
		loop_count++;
	}

	delete[] map_array;
	return 0;
}