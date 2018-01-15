#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.14159265359
#define INCREMENT 0.01745329251
#define NUM_READINGS 360

class img_to_laser
{
public:
  img_to_laser()
  {
    // Take the topics from params.yaml
    std::string sub_topic, pub_topic;
    nh_.getParam("subscribe_topic", sub_topic);
    nh_.getParam("publish_topic", pub_topic);
    pub_ = nh_.advertise<sensor_msgs::LaserScan>(pub_topic, 1000);
    sub_ = nh_.subscribe(sub_topic, 1000, &img_to_laser::range_finder, this);
  }

  void range_finder(const sensor_msgs::ImageConstPtr& msg)
  {
    // Filling up virtual scan data.
    ros::Time scan_time = ros::Time::now();
    scan_.header.stamp = scan_time;
    scan_.header.frame_id = "laser_scan";
    scan_.angle_min = -PI + INCREMENT;
    scan_.angle_max = PI;
    scan_.angle_increment = INCREMENT;
    scan_.time_increment = 0;
    scan_.scan_time = 0.01666666666;
    scan_.range_min = 0;
    scan_.ranges.resize(NUM_READINGS);
    scan_.intensities.resize(NUM_READINGS);

    try
    {
      cv::Mat gs(cv_bridge::toCvShare(msg, "mono8")->image.rows,
                 cv_bridge::toCvShare(msg, "mono8")->image.cols, CV_8UC1);
      gs = cv_bridge::toCvShare(msg, "mono8")->image;

      // Adjust max range from resolution of the image
      scan_.range_max = sqrt(gs.cols * gs.cols + gs.rows * gs.rows);

      // Get the virtual lidar position from params.yaml
      int lid_row = 0, lid_col = 0;
      nh_.getParam("lidar_row", lid_row);
      nh_.getParam("lidar_col", lid_col);

      // Convert from image to scan
      for (int i = 0; i < gs.cols; i++)
      {
        for (int j = 0; j < gs.rows; j++)
        {
          if ((int)gs.at<uchar>(j, i))
          {
            int x_dist = i - lid_col;
            int y_dist = j - lid_row;
            int tot_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
            int theta = floor(atan2(y_dist, x_dist) * 180 / PI);
            if (tot_dist < scan_.ranges[theta + 90])
            {
              scan_.ranges[theta + 90] = tot_dist;
            }
          }
        }
      }
      pub_.publish(scan_);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  sensor_msgs::LaserScan scan_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_to_laser_node");
  img_to_laser sub_and_pub();
  ros::spinOnce();
  return 0;
}
