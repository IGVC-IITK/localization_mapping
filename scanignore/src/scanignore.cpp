#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream> 
#include <limits>
#include <vector>
using namespace std;
#define PI 3.14159265
const double INF = std::numeric_limits<double>::infinity();

class scanIgnoreNode{
public:
	std::vector<double> ignore_angles;
	sensor_msgs::LaserScan modified_scan;

	void modifyScan(const sensor_msgs::LaserScanConstPtr& lscan){
		modified_scan.ranges.resize(lscan->ranges.size());
		modified_scan.intensities.resize(lscan->intensities.size());
		unsigned counter = 0;
		double start_angle = lscan->angle_min;
		double end_angle = lscan->angle_max;
		double angle_count = start_angle;
		modified_scan.angle_increment = lscan->angle_increment;
		//double current_angle_to_ignore = ignore_angles[counter];
		for(unsigned int i =0; i<modified_scan.ranges.size();i++){
			if(ignore_angles[counter]<angle_count && counter<(ignore_angles.size()-1))
				counter++;
			if(counter%2==0){
				modified_scan.ranges[i]=lscan->ranges[i];
				if(modified_scan.intensities.size()>1)
					modified_scan.intensities[i]=lscan->intensities[i];
			}
			else{
				modified_scan.ranges[i] = INF;
				if(modified_scan.intensities.size()>1)
					modified_scan.intensities[i]=lscan->intensities[i];
			}
			angle_count+=modified_scan.angle_increment;

			if(angle_count>end_angle)
				break;
		}
		
		modified_scan.header.stamp = lscan->header.stamp;
	    modified_scan.header.frame_id = lscan->header.frame_id;
	    modified_scan.angle_min = start_angle;
	    modified_scan.angle_max = angle_count;
	    modified_scan.time_increment = lscan->time_increment;
	    modified_scan.range_min = lscan->range_min;
	    modified_scan.range_max = lscan->range_max;
	    modified_scan.scan_time = lscan->scan_time;
	    
	}
};
//int test;
int main(int argc,char* argv[])
{
	ros::init(argc,argv,"scanignore");
	ros::NodeHandle n;
	scanIgnoreNode laser;
	for(int j=1;j<argc;j++){
		laser.ignore_angles.push_back(atof(argv[j]));
	}
	ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 50, &scanIgnoreNode::modifyScan,&laser);
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("/scan_modify", 50);
	ros::Rate loop_rate(50);
	while(n.ok()){
		//cin>>test;
		scan_pub.publish(laser.modified_scan);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}