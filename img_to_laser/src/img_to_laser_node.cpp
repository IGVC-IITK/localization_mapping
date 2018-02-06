#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define PI 3.1415926535897
#define INCREMENT 0.01745329251
#define NUM_READINGS 360
#define SCALING_FACTOR 100

class img_to_laser
{
public:
  img_to_laser()
  {

  	// Constructor Function
    ROS_INFO("Started Running");
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("img_scan", 1000);
    sub_ = nh_.subscribe("final_image", 1000, &img_to_laser::range_finder, this);

    // Take the topics from params.yaml
    nh_.param("virtual_lidar_x", lid_x, 400);
    nh_.param("virtual_lidar_y", lid_y, 400);

    // Filling up virtual scan data.
    ros::Time scan_time = ros::Time::now();
    scan_.header.stamp = scan_time;
    scan_.header.frame_id = "laser_scan";
    scan_.angle_min = -PI + INCREMENT;
    scan_.angle_max = PI;
    scan_.angle_increment = INCREMENT;
    scan_.time_increment = 0;
    scan_.scan_time = 0.1;
    scan_.range_min = 0;
    scan_.ranges.resize(NUM_READINGS);
    scan_.intensities.resize(NUM_READINGS);
  }

  void range_finder(const sensor_msgs::ImageConstPtr& msg)
  {
    // Flush Scan at start of every callback
    for(int i=0;i<360;i++){
      scan_.ranges[i]=std::numeric_limits<float>::infinity();
      scan_.intensities[i] = 0;
    }

    try
    {
      cv::Mat gs1(cv_bridge::toCvShare(msg, "bgr8")->image.rows,
                 cv_bridge::toCvShare(msg, "bgr8")->image.cols, CV_8UC3);
      gs1 = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat gs;
      cv::cvtColor(gs1,gs, cv::COLOR_BGR2GRAY);      // Adjust max range from resolution of the image
      scan_.range_max = sqrt(gs.cols * gs.cols + gs.rows * gs.rows);
      
      ROS_INFO("Params are lidar x : %3d lidar y : %3d", lid_x, lid_y);

      // Convert from image to scan
      for (int i = 0; i < gs.cols; i++)
      {
        for (int j = 0; j < gs.rows; j++)
        {
          if ((bool)gs.at<uchar>(j, i) && (lid_x != i && lid_y != j))
          {

          	/*  
          		Calculate x distance and y distance and 
          	 	invert y (In image convention y is inverted)
          	*/
            int x_dist = i - lid_x;
            int y_dist = -(j - lid_y);
            float tot_dist = sqrt(x_dist*x_dist + y_dist*y_dist)/SCALING_FACTOR;
            int theta = floor(atan2(y_dist, x_dist) * 180 / PI + 180);

            // Safety
            theta = theta%360;

            if (tot_dist < scan_.ranges[theta])
            {
              scan_.ranges[theta] = tot_dist;
              scan_.intensities[theta] = 10;
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
  int lid_x;
  int lid_y;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_to_laser_node");
  img_to_laser sub_and_pub;
  ros::spin();
  return 0;
}
