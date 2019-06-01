#pragma once

#include <vector>
#include <string>
#include <stdlib.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LocalMapper {
public:
  LocalMapper(ros::NodeHandle nh);

  void ScanCallback (const sensor_msgs::LaserScanConstPtr &scan);
  void ImageCallback (const sensor_msgs::ImageConstPtr &img);
  void PublishMap ();

public:
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener * tf_listener;


private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	ros::Subscriber im_sub_, sc_sub_;

	nav_msgs::OccupancyGrid local_map_; // the only map
	geometry_msgs::TransformStamped lidar_to_map_, cam_to_map_;

  double cell_resolution_, map_width_m_, map_height_m_, image_scale_;
  double map_origin_[3];
  int map_width_px_, map_height_px_;

  bool scan_added_ = false, lane_added_ = false; // add lane or scan only once
  bool no_lane_ = false, no_scan_ = false; // only for testing

  std::string map_frame_;

}; // class local_mapper
