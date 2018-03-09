#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream> 
#include <limits>
#include <vector>
#define PI 3.14159265359

using namespace std;
const double INF = std::numeric_limits<double>::infinity();

class scanIgnoreNode{
public:
	scanIgnoreNode(){
			sub_scan = n.subscribe("/scan", 10, &scanIgnoreNode::modifyScan, this);
			pub_scan = n.advertise<sensor_msgs::LaserScan>("/scan_modify", 10);
	}

	std::vector<double> ignore_angles;

	void modifyScan(const sensor_msgs::LaserScanConstPtr& lscan){
		modified_scan.ranges.resize(lscan->ranges.size());
		modified_scan.intensities.resize(lscan->intensities.size());
    int counter = 0;
		double start_angle = lscan->angle_min;
		double end_angle = lscan->angle_max;
		double angle_count = start_angle;
		modified_scan.angle_increment = lscan->angle_increment;

    for(int i = 0; i < modified_scan.ranges.size(); i++){
      if(counter < ignore_angles.size() && i == (int)(ignore_angles[counter] - lscan->angle_min*180/PI)){
        counter++;
      }
      if(counter%2 == 0){
        modified_scan.ranges[i] = lscan->ranges[i];
        modified_scan.intensities[i] = lscan->intensities[i];
      }
      else{
        modified_scan.ranges[i] = INF;
        modified_scan.intensities[i] = 0;
      }
//      if(ignore_angles[counter]<angle_count && counter<(ignore_angles.size()-1))
//				counter++;
//			if(counter%2==0){
//				modified_scan.ranges[i]=lscan->ranges[i];
//				if(modified_scan.intensities.size()>1)
//					modified_scan.intensities[i]=lscan->intensities[i];
//			}
//			else{
//				modified_scan.ranges[i] = INF;
//				if(modified_scan.intensities.size()>1)
//					modified_scan.intensities[i]=lscan->intensities[i];
//			}
//			angle_count+=modified_scan.angle_increment;

//			if(angle_count>end_angle)
//				break;
		}
		
		modified_scan.header.stamp = lscan->header.stamp;
	    modified_scan.header.frame_id = lscan->header.frame_id;
      modified_scan.angle_min = lscan->angle_min;
      modified_scan.angle_max = lscan->angle_max;
	    modified_scan.time_increment = lscan->time_increment;
	    modified_scan.range_min = lscan->range_min;
	    modified_scan.range_max = lscan->range_max;
	    modified_scan.scan_time = lscan->scan_time;
	    pub_scan.publish(modified_scan);
	}
private:
	ros::NodeHandle n;
	ros::Subscriber sub_scan;
	ros::Publisher pub_scan;
	sensor_msgs::LaserScan modified_scan;
};

int main(int argc,char* argv[])
{
	ros::init(argc,argv,"scanignore");
	scanIgnoreNode laser;
	for(int j=1;j<argc;j++){
		laser.ignore_angles.push_back(atof(argv[j]));
	}
	ros::spin();

	return 0;
}
