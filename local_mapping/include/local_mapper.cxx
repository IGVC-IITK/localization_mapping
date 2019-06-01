#include "local_mapper.h"

LocalMapper::LocalMapper (ros::NodeHandle nh)
  : nh_(nh)
{
  nh_.param<double>("cell_resolution_", cell_resolution_, 0.05);
  nh_.param<double>("real_map_width_m", map_width_m_, 20.0);
  nh_.param<double>("real_map_height_m", map_width_m_, 20.0);
  nh_.param<double>("image_scale", image_scale_, 100.0);
  nh_.param<std::string>("map_frame", map_frame_, "odom");

  nh_.param<bool>("no_lane", no_lane_, false);
  nh_.param<bool>("no_scan", no_scan_, false);

  if(cell_resolution_ < 0) {
    ROS_FATAL("Incorrect cell resolution supplied");
    return;
  }

  map_width_px_ = int(map_width_m_/cell_resolution_);
  map_height_px_= int(map_width_m_/cell_resolution_);

  map_origin_[0] = -map_width_m_/2.0;
  map_origin_[1] = -map_width_m_/2.0;
  map_origin_[2] = 0.0;

  local_map_.header.frame_id = "odom";

  local_map_.info.resolution = cell_resolution_;
  local_map_.info.width      = map_width_px_;
  local_map_.info.height     = map_height_px_;

  local_map_.info.origin.position.x = map_origin_[0];
  local_map_.info.origin.position.y = map_origin_[1];

  tf_listener = new tf2_ros::TransformListener(tf_buffer);

  pub_    = nh_.advertise<nav_msgs::OccupancyGrid>("/real_map", 1);

  im_sub_ = nh_.subscribe("/image", 1, &LocalMapper::ImageCallback, this);
  sc_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1,
                                                  &LocalMapper::ScanCallback,
                                                  this);

  ROS_INFO("Local Mapping Instantiated");
}

void LocalMapper::ScanCallback (const sensor_msgs::LaserScanConstPtr& scan) {

  try{
    lidar_to_map_ = tf_buffer.lookupTransform(map_frame_,
                                              scan->header.frame_id,
                                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "Can't get transform %s", ex.what());
    return;
  }

  // if lane hasn't been added reset the map
  int f = (not lane_added_) or no_lane_;
  if((not lane_added_) or no_lane_) {
    local_map_.data.clear();
    local_map_.data.resize(map_width_px_ * map_height_px_, -1);
  }

  ROS_INFO("Processing Laser Scan");

  double angle_max = scan->angle_max, angle_min = scan->angle_min;
  double angle_inc = scan->angle_increment;

  for(int i = 0; i <= int((angle_max - angle_min) / angle_inc); i++){
    double curr_angle = angle_min + angle_inc * i;
    double scan_range = scan->ranges[i];

    // angle limit check, NOTE: toggle per use
    if(curr_angle > -M_PI/2 and curr_angle < M_PI/2)
      continue;

    if(scan_range > scan->range_max or scan_range < scan->range_min)
      continue;

    int x,y;
    geometry_msgs::PointStamped point_lidar_frame, point_robot_frame;

    double curr_range = 0.0;
    while(curr_range < scan_range + cell_resolution_) {
      curr_range = std::min(curr_range, scan_range);

      point_lidar_frame.point.x = curr_range * cos(curr_angle);
      point_lidar_frame.point.y = curr_range * sin(curr_angle);
      point_lidar_frame.point.z = 0.0;

      tf2::doTransform(point_lidar_frame, point_robot_frame, lidar_to_map_);

      x = (int)((point_robot_frame.point.x - map_origin_[0])/cell_resolution_);
      y = (int)((point_robot_frame.point.y - map_origin_[1])/cell_resolution_);

      if(x >= 0 and x < map_width_px_ and y >= 0 and y < map_height_px_)
        local_map_.data[y * map_width_px_ + x] =
          std::max((int)local_map_.data[y * map_width_px_ + x],
                   100 * (curr_range == scan_range));

      curr_range += cell_resolution_;
    }
  }

  scan_added_ = true;

  if((no_lane_ or lane_added_) and scan_added_) {
    PublishMap();
  }
}


void LocalMapper::ImageCallback(const sensor_msgs::ImageConstPtr& img) {

  try{
    cam_to_map_ = tf_buffer.lookupTransform(map_frame_,
                                            img->header.frame_id,
                                            ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "Can't get transform %s", ex.what());
    return;
  }

  // if laser scan isn't added reset the map
  if((not scan_added_) or no_scan_) {
    local_map_.data.clear();
    local_map_.data.resize(map_width_px_ * map_height_px_, -1);
  }

  cv::Mat im = cv_bridge::toCvShare(img,
                                    sensor_msgs::image_encodings::MONO8)->image;

  geometry_msgs::PointStamped point_cam_frame, point_robot_frame;
  int x,y;
  ROS_INFO("Processing Image");

  for(int i = 0; i < im.rows; ++i) {
    for(int j = 0; j < im.cols; ++j) {
      point_cam_frame.point.x = j/image_scale_;
      point_cam_frame.point.y = i/image_scale_;
      point_cam_frame.point.z = 0.0;

      tf2::doTransform(point_cam_frame, point_robot_frame, cam_to_map_);

      x = (int)((point_robot_frame.point.x - map_origin_[0])/cell_resolution_);
      y = (int)((point_robot_frame.point.y - map_origin_[1])/cell_resolution_);

      if(x >= 0 and x < map_width_px_ and y >= 0 and y < map_height_px_) {

        if(im.at<uchar>(i, j) == 127) {
          local_map_.data[map_width_px_ * y + x] =
            std::max((int)local_map_.data[map_width_px_ * y + x], 0);
        }

        else if(im.at<uchar>(i, j) == 254) {
          local_map_.data[map_width_px_ * y + x] = 100;
        }

      } // x, y condition

    } // loop j
  } // loop i

  lane_added_ = true;

  if(lane_added_ and (no_scan_ or scan_added_)) {
    PublishMap();
  }

}


void LocalMapper::PublishMap() {
  local_map_.header.stamp = ros::Time::now();
  pub_.publish(local_map_);
  lane_added_ = false;
  scan_added_ = false;
}

