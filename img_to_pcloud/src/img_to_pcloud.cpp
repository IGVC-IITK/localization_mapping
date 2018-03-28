#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// Scaling factor (in pixels/m)
// (Use 80.0 for realtime low-res output but 160.0 for datasets)
#define SCALING_FACTOR 160.0
#define PIXEL_WIDTH 0.0
class img_to_pcloud
{
public:
  img_to_pcloud()
  {
    // Constructor Function
    ROS_INFO("Started Running");
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/img_pcloud", 100);
    sub_ = nh_.subscribe("final_image", 100, &img_to_pcloud::cloudPublisher, this);

  }

  void cloudPublisher(const sensor_msgs::ImageConstPtr& msg)
  {
   

    try
    {
      cv::Mat gs1(cv_bridge::toCvShare(msg, "bgr8")->image.rows,
                  cv_bridge::toCvShare(msg, "bgr8")->image.cols, CV_8UC3);
      gs1 = cv_bridge::toCvShare(msg, "bgr8")->image;
      cv::Mat gs;
      cv::cvtColor(gs1,gs, cv::COLOR_BGR2GRAY);
      
      //unorganized pointcloud with x,y,z and intensity
      cloud.height = 1;
      for (int i = 0; i < gs.cols; i++)
      {
        for (int j = 0; j < gs.rows; j++)
        {
          if ((bool)gs.at<uchar>(j, i)) //only points with non-zero grayscale values are used for making pointcloud
          {
            pcl::PointXYZI toPush;
            toPush.x = ((double)i)/SCALING_FACTOR + PIXEL_WIDTH/2; 
            toPush.y = ((double)j)/SCALING_FACTOR + PIXEL_WIDTH/2;
            toPush.z = ((double)0.0);
            toPush.intensity = ((float)gs.at<uchar>(j, i))/255;
            cloud.points.push_back(toPush); 
          }
        }
      }
      cloud.width = cloud.points.size();
      
      //converting Pointcloud to pointcloud2 ROS message
      pcl::toROSMsg(cloud, cloud_msg);
      cloud_msg.header.stamp = msg->header.stamp;
      cloud_msg.header.frame_id = msg->header.frame_id;

      //publish ROS message
      pub_.publish(cloud_msg);
      ROS_INFO("\nHeight:%d \nWidth: %d",cloud_msg.height, cloud_msg.width);
      std::cout<<"header:\n\t timestamp = "<<cloud_msg.header.stamp<<"\n\t frame_id = "<<cloud_msg.header.frame_id<<"\n";
      
      cloud.points.clear();
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
  pcl::PointCloud<pcl::PointXYZI> cloud;
  sensor_msgs::PointCloud2 cloud_msg;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "img_to_pcloud");
  img_to_pcloud sub_and_pub;
  ros::spin();
  return 0;
}