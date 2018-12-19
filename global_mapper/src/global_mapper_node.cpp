#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#define cellResolution 0.05 //(meters/cell)
#define real_map_width (int)(400.0/cellResolution)
#define real_map_height (int)(400.0/cellResolution)
float map_origin_position[3] = {-200.0,-200.0,0.0};
#define image_scale 160 //(pixels/meter)
#define mapOriginToImageX 100 //cell no. of pixel
#define mapOriginToImageY 100 // at (x,y)===(columns/2,rows)

using namespace std;

class real_mapper
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub1_,sub2_;
  
  geometry_msgs::TransformStamped lidar_to_map, cam_to_map;
  nav_msgs::OccupancyGrid real_map;

public:
	tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener* tfListener;
	real_mapper(){
	
	real_map.header.frame_id = "odom";
    real_map.header.stamp = ros::Time::now();
    real_map.info.resolution = cellResolution;
    real_map.info.width = real_map_width;
    real_map.info.height = real_map_height;
    real_map.info.origin.position.x = map_origin_position[0];
    real_map.info.origin.position.y = map_origin_position[1];
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/real_map", 1);
    sub1_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_1", 1, &real_mapper::scanCallback, this);
    sub2_ = nh_.subscribe("final_image", 100, &real_mapper::imageCallback, this);
	ROS_INFO("Started real_ma2ppe2r");
	}

	void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg){
		ROS_INFO("Subscribing to scanCallback");
		lidar_to_map = this->tfBuffer.lookupTransform("odom", "laser_front", ros::Time(0));
		geometry_msgs::PointStamped point_lidar, point_robot;
		int x,y;
		ROS_INFO("moving to outer loop");
		for(int i=0;i<=(int)((scan_msg->angle_max - scan_msg->angle_min)/scan_msg->angle_increment);i++){
			float laser_angle = scan_msg->angle_min + scan_msg->angle_increment*i;
			if((laser_angle>-M_PI/2 && laser_angle<M_PI) || scan_msg->ranges[i] < scan_msg->range_min || scan_msg->ranges[i] > scan_msg->range_max)
				continue;
			for(float range_current = 0.0; range_current<scan_msg->ranges[i]; range_current+=cellResolution){
				point_lidar.point.x = scan_msg->ranges[i]*cos(laser_angle);
				point_lidar.point.y = scan_msg->ranges[i]*sin(laser_angle);
				point_lidar.point.z = 0.0;
				tf2::doTransform(point_lidar, point_robot, lidar_to_map);
				x = (int)((point_robot.point.x - map_origin_position[0])/cellResolution);
				y = (int)((point_robot.point.y - map_origin_position[1])/cellResolution);
				if ((0 < y*real_map_width + x) && (y*real_map_width + x < real_map_width*real_map_height))
					real_map.data[y*real_map_width + x] = 0;
			}
			point_lidar.point.x = scan_msg->ranges[i]*cos(laser_angle);
			point_lidar.point.y = scan_msg->ranges[i]*sin(laser_angle);
			point_lidar.point.z = 0.0;
			tf2::doTransform(point_lidar, point_robot, lidar_to_map);
			x = (int)((point_robot.point.x - map_origin_position[0])/cellResolution);
			y = (int)((point_robot.point.y - map_origin_position[1])/cellResolution);
			if ((0 < y*real_map_width + x) && (y*real_map_width + x < real_map_width*real_map_height))
				real_map.data[y*real_map_width + x] = 100;
		}
	}

	void fillWithUnknown(){
		real_map.data.resize(real_map_width*real_map_height);
		 for(int i=0;i<real_map_width*real_map_height;i++){
		 	real_map.data[i] = -1;
		 }
	}

	void imageCallback(const sensor_msgs::ImageConstPtr& img_msg){
		cam_to_map = this->tfBuffer.lookupTransform("odom", img_msg->header.frame_id, ros::Time(0));
		geometry_msgs::PointStamped point_cam, point_robot;
		int x,y,up_flag=0;
		bool right_flag = false,left_flag = false;
		cv::Mat gs1(cv_bridge::toCvShare(img_msg, "bgr8")->image.rows,
                  cv_bridge::toCvShare(img_msg, "bgr8")->image.cols, CV_8UC3);
      	gs1 = cv_bridge::toCvShare(img_msg, "bgr8")->image;
      	cv::Mat gs;
      	cv::cvtColor(gs1,gs, cv::COLOR_BGR2GRAY);
      	for(int j=gs.rows-1;j>=0;j--){
      		if((bool)gs.at<uchar>(j,gs.cols/2) && j!=gs.rows-1){
      			if(!(bool)gs.at<uchar>(j+1,gs.cols/2)){
      				for(int k=0;k<gs.cols/2;k++){
      					if((bool)gs.at<uchar>(j,(gs.cols/2)+k)){
      						up_flag=2;
      						break;
      					}
      					else if((bool)gs.at<uchar>(j,(gs.cols/2)-k)){
      						up_flag=1;
      						break;
      					}
      				}
      			}
      		}

      		for(int i=1;i<gs.cols/2;i++){
      			x = (int)((i/image_scale)/cellResolution);
      			y = (int)(((gs.rows-1-j)/image_scale)/cellResolution);
      			int left_iter = (y+mapOriginToImageY)*real_map_width+mapOriginToImageX-x;
      			int right_iter = (y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX;
      			if(up_flag==0){//between lanes
		  			if((bool)gs.at<uchar>(j,(gs.cols/2)+i)){
		  				right_flag = true;
		  			}
		  			if((bool)gs.at<uchar>(j,(gs.cols/2)-i)){
		  				left_flag = true;
		  			}
		  			
		  			if((0 < (y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX) && ((y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX < real_map_width*real_map_height))
		  				if(right_flag)
		  					real_map.data[(y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX]=100;
		  				else if(real_map.data[(y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX]==-1)
		  					real_map.data[(y+mapOriginToImageY)*real_map_width + x+mapOriginToImageX]=0;

		  			if((0 < (y+mapOriginToImageY)*real_map_width+mapOriginToImageX-x) && ((y+mapOriginToImageY)*real_map_width +mapOriginToImageX-x < real_map_width*real_map_height))
		  				if(left_flag)
		  					real_map.data[(y+mapOriginToImageY)*real_map_width +mapOriginToImageX-x]=100;
		  				else if(real_map.data[(y+mapOriginToImageY)*real_map_width +mapOriginToImageX-x]==-1)
		  					real_map.data[(y+mapOriginToImageY)*real_map_width +mapOriginToImageX-x]=0;
      			}
      			else if(up_flag==1){//left of left lane
      				if(left_iter>0 && left_iter<real_map_width*real_map_height && mapOriginToImageX-x>=0 && y+mapOriginToImageY<real_map_height)
      					real_map.data[left_iter]=100;
      				if((bool)gs.at<uchar>(j,(gs.cols/2)+i) && !(bool)gs.at<uchar>(j,(gs.cols/2)+i-1))
      					right_flag = (!right_flag);
      				if(right_iter>0 && right_iter<real_map_width*real_map_height && x+mapOriginToImageX<real_map_width && y+mapOriginToImageY<real_map_height)
      					if(!right_flag)
      						real_map.data[right_iter]=100;
      					else if(real_map.data[right_iter]==-1)
      						real_map.data[right_iter]=0;
      			}
      			else if(up_flag==2){//right of right lane
      				if(right_iter>0 && right_iter<real_map_width*real_map_height && mapOriginToImageX+x<real_map_width && y+mapOriginToImageY<real_map_height)
      					real_map.data[right_iter]=100;
      				if((bool)gs.at<uchar>(j,(gs.cols/2)-i) && !(bool)gs.at<uchar>(j,(gs.cols/2)-i+1))
      					left_flag = (!left_flag);
      				if(left_iter>0 && left_iter<real_map_width*real_map_height && mapOriginToImageX-x>=0 && y+mapOriginToImageY<real_map_height)
      					if(!left_flag)
      						real_map.data[left_iter]=100;
      					else if(real_map.data[left_iter]==-1)
      						real_map.data[left_iter]=0;
      			}
      		}
      		right_flag = false; left_flag = false;
      	}

	}

	void publishMap(){
		ROS_INFO("here before");
		pub_.publish(real_map);
		ROS_INFO("here after 2222");
	}


};
int main(int argc, char** argv)
{	
  int loop_count=0;
  ros::init(argc, argv, "real_mapper");
  real_mapper sub_and_pub;
  sub_and_pub.fillWithUnknown();
  ros::Rate loop_rate(10);
  while(ros::ok()){
  	if(loop_count%30==0){
  		loop_count=0;
  		sub_and_pub.publishMap();
  	}
  	ros::spinOnce();
  	loop_count++;
  }
  return 0;
}