#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <tf_calibration/TFConfig.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

void dynamicCallback(tf_calibration::TFConfig& config, uint32_t level, tf::Transform& transform_calibrated)
{
	// Updating tf values (which has been passed by reference)
	transform_calibrated.setOrigin(tf::Vector3 (config.x, config.y, config.z));
    transform_calibrated.setRotation(tf::Quaternion(config.roll*(M_PI/180.0), config.pitch*(M_PI/180.0), config.yaw*(M_PI/180.0)));

	return;
}

int main(int argc, char **argv)
{
	// Initialization
	ros::init(argc, argv, "tf_calibration_node");
	ros::NodeHandle nh;
	ros::Time t;

	std::string parent_frame, child_frame;
	nh.param<std::string>("parent_frame", parent_frame, "laser_front");
	nh.param<std::string>("child_frame", child_frame, "laser_rear");

	tf::Transform transform_calibrated;
	tf::TransformBroadcaster br_tfc;
	dynamic_reconfigure::Server<tf_calibration::TFConfig> server_tfc;
	dynamic_reconfigure::Server<tf_calibration::TFConfig>::CallbackType dcParameterized;
	dcParameterized = boost::bind(&dynamicCallback, _1, _2, boost::ref(transform_calibrated));
	server_tfc.setCallback(dcParameterized);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();

		t = ros::Time::now();
		br_tfc.sendTransform(tf::StampedTransform(transform_calibrated, ros::Time::now(), parent_frame, child_frame));

		loop_rate.sleep();
	}

	return 0;
}
