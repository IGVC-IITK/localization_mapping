#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define DISTANCE(x1, y1, x2, y2) std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
#define THRESHOLD 0.2

void odomCallback(const nav_msgs::OdometryConstPtr& odom, nav_msgs::Path& path_utm, 
	tf::TransformListener& listener, ros::Publisher& pub_smooth)
{
	static geometry_msgs::PoseStamped prev_position;
	static double last_dist;
	static int path_iterator = 0;
	if (path_iterator < path_utm.poses.size())
		if (path_iterator == 0 || DISTANCE(odom->pose.pose.position.x, odom->pose.pose.position.y, prev_position.pose.position.x, prev_position.pose.position.y) > last_dist - THRESHOLD)
		{
			prev_position.header = odom->header;
			prev_position.pose = odom->pose.pose;
			geometry_msgs::PoseStamped waypoint_utm = path_utm.poses[path_iterator];
			waypoint_utm.header.stamp = odom->header.stamp;
			waypoint_utm.header.frame_id = "utm";
			geometry_msgs::PoseStamped waypoint_odom;
			listener.waitForTransform("odom", "utm", odom->header.stamp, ros::Duration(0.5));
			listener.transformPose("odom", waypoint_utm, waypoint_odom);

			nav_msgs::Path path_odom;
			path_odom.header = odom->header;
			path_odom.poses.resize(3);
			path_odom.poses[0] = prev_position;
			double yaw = std::atan2(waypoint_odom.pose.position.y - prev_position.pose.position.y,
			 waypoint_odom.pose.position.x - prev_position.pose.position.x);
			double z = std::sin(yaw/2), w = std::cos(yaw/2);
			path_odom.poses[1] = prev_position;
			path_odom.poses[2] = waypoint_odom;
			path_odom.poses[1].pose.orientation.z = z;
			path_odom.poses[1].pose.orientation.w = w;
			path_odom.poses[2].pose.orientation.z = z;
			path_odom.poses[2].pose.orientation.w = w;
			pub_smooth.publish(path_odom);

			double x1 = prev_position.pose.position.x;
			double y1 = prev_position.pose.position.y;
			double x2 = waypoint_odom.pose.position.x;
			double y2 = waypoint_odom.pose.position.y;
			last_dist = DISTANCE(x1, y1, x2, y2);
			path_iterator++;
		}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_to_path_node");
	ros::NodeHandle nh;
	tf::TransformListener listener;

	nav_msgs::Path path_utm;

	ros::Publisher pub_smooth = nh.advertise<nav_msgs::Path>("path/smooth", 1);
	ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(
		&odomCallback, _1,  boost::ref(path_utm), boost::ref(listener), boost::ref(pub_smooth)));

	std::vector<double> coordinates (2, 0.0);	// x,y coordinates in UTM frame
	nh.getParam("utm", coordinates);
	if (coordinates.size()%2)
	{
		ROS_ERROR_STREAM("Expected even number of values (x1, y1, x2, y2, ...) for parameter utm.");
		return -1;
	}
	else
	{
		path_utm.header.stamp = ros::Time::now();
		path_utm.header.frame_id = "utm";
		path_utm.poses.resize(coordinates.size()/2);
		for (int i=0; i<coordinates.size()/2; i++)
		{
			path_utm.poses[i].pose.position.x = coordinates[2*i+0];
			path_utm.poses[i].pose.position.y = coordinates[2*i+1];
			path_utm.poses[i].pose.orientation.w = 1.0;	// for unit quaternion
		}
	}
	ros::Duration(2.0).sleep(); // waiting for controller to start-up
	ros::spin();

	return 0;
}