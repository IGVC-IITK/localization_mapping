#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define ka 1.0
#define kb 0.5

#define VELOCITY 0.5

#define a0 -1.0
#define b0 0.0
#define c0 -0.5

#define sampling_rate 10

class pid_control{
	public:
		pid_control(){
		 pub_ = nh_.advertise<geometry_msgs::Twist>("ros0xrobot/cmd_vel", 1);
		 sub_ = nh_.subscribe("ros0xrobot/odom", 1, &pid_control::odomCallback, this);
		}

		void odomCallback(const nav_msgs::OdometryConstPtr& odom){
			pos_x = odom->pose.pose.position.x;
			pos_y = odom->pose.pose.position.y;

			tf::Quaternion q(
				odom->pose.pose.orientation.x,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.z,
				odom->pose.pose.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);

			v_x = odom->twist.twist.linear.x;
			v_y = odom->twist.twist.linear.y;
			v 	= sqrt(v_x*v_x + v_y*v_y);

			error_l 	= (a0*pos_x + b0*pos_y + c0)/norm;
			error_yaw 	= yaw - yaw_d;
			control_u	= - ka*v*error_l*sinc(error_yaw) - kb*error_yaw;
			std::cout<<error_l<<" "<<error_yaw<<" "<<control_u<<std::endl;
			twist_output.linear.x = VELOCITY;
			twist_output.linear.y = 0.0;
			twist_output.linear.z = 0.0;
			twist_output.angular.x = 0.0;
			twist_output.angular.y = 0.0;
			twist_output.angular.z = control_u;

			pub_.publish(twist_output);
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::Time t;
		double pos_x = 0.0, pos_y = 0.0;
		double roll = 0.0, pitch = 0.0, yaw = 0.0; // roll and pitch are dummy vars
		double v_x = VELOCITY, v_y = 0.0, v = VELOCITY;
		double yaw_d = atan2(-a0, b0), norm = sqrt(a0*a0 + b0*b0);
		double error_l = 0.0, error_yaw = 0.0;
		double control_u = 0.0;
		geometry_msgs::Twist twist_output;

		inline double sinc(double x){
			return (abs(x)>0.01)?(sin(x)/x):1.0;
		}
};

int main(int argc,char* argv[]){
	ros::init(argc, argv, "pid_controller");
	pid_control controller;
	ros::Rate loop_rate(sampling_rate);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
