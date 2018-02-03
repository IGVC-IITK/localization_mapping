#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Only for determining path type
#define DIST_THRESH 0.01
#define ANGLE_THRESH 0.01

#define ka 1.0	// responsiveness to distance from path 
#define kb 1.0	// responsiveness to angular deviation

#define SPEED_SETPOINT 0.9
#define YAW_TOLERANCE 0.052

#define sampling_rate 10.0

class UnicycleControl{
	public:
		UnicycleControl(){
		 pub_ = nh_.advertise<geometry_msgs::Twist>("ros0xrobot/cmd_vel", 1);
		 sub_ = nh_.subscribe("ros0xrobot/odom", 1, 
		 	&UnicycleControl::odomCallback, this);

		 // Setting up a fixed path here for testing purpose
		 path.header.stamp = ros::Time::now();
		 path.header.frame_id = "odom";
		 path.poses.resize(10);
		 path.poses[0].header = path.header;
		 path.poses[0].pose.position.x = 0.0;
		 path.poses[0].pose.position.y = 0.0;
		 path.poses[0].pose.orientation.z = 0.0; // sin(yaw/2)
		 path.poses[0].pose.orientation.w = 1.0; // cos(yaw/2)
		 path.poses[1].header = path.header;
		 path.poses[1].pose.position.x = 5.0;
		 path.poses[1].pose.position.y = 0.0;
		 path.poses[1].pose.orientation.z = sin(0*M_PI/360);
		 path.poses[1].pose.orientation.w = cos(0*M_PI/360);
		 path.poses[2].header = path.header;
		 path.poses[2].pose.position.x = 5.0;
		 path.poses[2].pose.position.y = 0.0;
		 path.poses[2].pose.orientation.z = sin(-90*M_PI/360);
		 path.poses[2].pose.orientation.w = cos(-90*M_PI/360);
		 path.poses[3].header = path.header;
		 path.poses[3].pose.position.x = 5.0;
		 path.poses[3].pose.position.y = -5.0;
		 path.poses[3].pose.orientation.z = sin(-90*M_PI/360);
		 path.poses[3].pose.orientation.w = cos(-90*M_PI/360);
		 path.poses[4].header = path.header;
		 path.poses[4].pose.position.x = 5.0;
		 path.poses[4].pose.position.y = -5.0;
		 path.poses[4].pose.orientation.z = sin(-80*M_PI/360);
		 path.poses[4].pose.orientation.w = cos(-80*M_PI/360);
		 path.poses[5].header = path.header;
		 path.poses[5].pose.position.x = 5.4;
		 path.poses[5].pose.position.y = -10.0;
		 path.poses[5].pose.orientation.z = sin(-90*M_PI/360);
		 path.poses[5].pose.orientation.w = cos(-90*M_PI/360);
		 path.poses[6].header = path.header;
		 path.poses[6].pose.position.x = 5.4;
		 path.poses[6].pose.position.y = -20.0;
		 path.poses[6].pose.orientation.z = sin(-90*M_PI/360);
		 path.poses[6].pose.orientation.w = cos(-90*M_PI/360);
		 path.poses[7].header = path.header;
		 path.poses[7].pose.position.x = 1.6;
		 path.poses[7].pose.position.y = -23.8;
		 path.poses[7].pose.orientation.z = sin(-180*M_PI/360);
		 path.poses[7].pose.orientation.w = cos(-180*M_PI/360);
		 path.poses[8].header = path.header;
		 path.poses[8].pose.position.x = -28.4;
		 path.poses[8].pose.position.y = -23.8;
		 path.poses[8].pose.orientation.z = sin(-180*M_PI/360);
		 path.poses[8].pose.orientation.w = cos(-180*M_PI/360);
		 path.poses[9].header = path.header;
		 path.poses[9].pose.position.x = -30.4;
		 path.poses[9].pose.position.y = -21.8;
		 path.poses[9].pose.orientation.z = sin(90*M_PI/360);
		 path.poses[9].pose.orientation.w = cos(90*M_PI/360);
		}

		// Reads current state, calls other functions and publishes control
		void odomCallback(const nav_msgs::OdometryConstPtr& odom){
			// Getting current state
			x = odom->pose.pose.position.x;
			y = odom->pose.pose.position.y;

			q_temp = tf::Quaternion(
				odom->pose.pose.orientation.x,
				odom->pose.pose.orientation.y,
				odom->pose.pose.orientation.z,
				odom->pose.pose.orientation.w);
			m_temp = tf::Matrix3x3(q_temp);
			m_temp.getRPY(roll, pitch, yaw);

			v_x = odom->twist.twist.linear.x;
			v_y = odom->twist.twist.linear.y; // typically, this should be zero
			v 	= sqrt(v_x*v_x + v_y*v_y);

			switchPathPiece();			
			generateControl();

			// Publishing twist
			twist_output.linear.x 	= speed_d;
			twist_output.angular.z 	= omega_d;
			pub_.publish(twist_output);

			ROS_INFO_STREAM("Errors - yaw: " <<yaw_error<<" l: "<<l_error);
			ROS_INFO_STREAM("Control - speed: "<<speed_d<<" omega: "<<omega_d);
		}

	private:
		ros::NodeHandle nh_;
		ros::Publisher pub_;
		ros::Subscriber sub_;

		// Vars representing current state
		double x = 0.0, y = 0.0;
		double roll = 0.0, pitch = 0.0, yaw = 0.0;	// roll and pitch are dummy vars
		double v_x = 0.0, v_y = 0.0, v = 0.0;

		// Vars representing current path-piece to be followed
		nav_msgs::Path path;
		int path_iterator = 0;
		double x_prev = 0.0, y_prev = 0.0, yaw_prev = 0.0;
		double x_next = 0.0, y_next = 0.0, yaw_next = 0.0;
		double length_path = 0.0, yaw_path = 0.0, speed_path = SPEED_SETPOINT;
		double curvature_path = 0.0, cx_path = 0.0, cy_path = 0.0;	// only for arcs
		char path_type = 'l';	// line, arc, point or end ('l', 'a', 'p' or 'e')

		// Vars representing errors and control inputs
		double yaw_d = 0.0, yaw_error = 0.0, l_error = 0.0;
		double speed_d = SPEED_SETPOINT, omega_d = 0.0;
		geometry_msgs::Twist twist_output;

		// Temp vars
		tf::Quaternion q_temp;
		tf::Matrix3x3 m_temp;

		// Switches to next path-piece if required & calculates some path parameters
		// (some parameters are set to safe values even if not required by control)
		void switchPathPiece(){
			if (path_iterator == 0 || 
				(path_type == 'l' && distance(x_prev, y_prev, x, y) > length_path) ||
				(path_type == 'a' && distance(x_prev, y_prev, x, y) > length_path) ||
				(path_type == 'p' && abs(angWrap(yaw-yaw_d)) < YAW_TOLERANCE)){
				path_iterator++;
				if (path_iterator < path.poses.size()){
					// Getting initial and final poses for the path-piece
					x_prev = path.poses[path_iterator-1].pose.position.x;
					y_prev = path.poses[path_iterator-1].pose.position.y;
					q_temp = tf::Quaternion(
						path.poses[path_iterator-1].pose.orientation.x,
						path.poses[path_iterator-1].pose.orientation.y,
						path.poses[path_iterator-1].pose.orientation.z,
						path.poses[path_iterator-1].pose.orientation.w);
					m_temp = tf::Matrix3x3(q_temp);
					m_temp.getRPY(roll, pitch, yaw_prev);

					x_next = path.poses[path_iterator].pose.position.x;
					y_next = path.poses[path_iterator].pose.position.y;
					q_temp = tf::Quaternion(
						path.poses[path_iterator].pose.orientation.x,
						path.poses[path_iterator].pose.orientation.y,
						path.poses[path_iterator].pose.orientation.z,
						path.poses[path_iterator].pose.orientation.w);
					m_temp = tf::Matrix3x3(q_temp);
					m_temp.getRPY(roll, pitch, yaw_next);

					// Identifying path_type and setting up necessary parameters
					if (distance(x_prev, y_prev, x_next, y_next) > DIST_THRESH){
						if (abs(angWrap(yaw_next-yaw_prev)) > ANGLE_THRESH){
							path_type = 'a';	// finite arc with non-zero radius
							ROS_INFO_STREAM("Mode: Arc");
							//////////////////////////////////////////////////////
							// ASSUMING THAT THE ROBOT LIES BETWEEN THE ARMS OF //
							// ANGLE SUBTENDED BY THE ARC AT THE CENTER	& THAT	//
							// THE ARC SUBTENDS AN ANGLE < M_PI AT THE CENTER	//
							//////////////////////////////////////////////////////
							length_path = distance(x_prev, y_prev, x_next, y_next);
							yaw_path 	= (yaw_prev + yaw_next)/2.0;
							speed_path 	= SPEED_SETPOINT;
							// curvature_path is positive for anti-clockwise turns
							curvature_path = 
								2.0*sin((yaw_next-yaw_prev)/2.0)/(length_path);
							cx_path = x_prev - sin(yaw_prev)/curvature_path;
							cy_path = y_prev + cos(yaw_prev)/curvature_path;
						}
						else{
							path_type 	= 'l';	// line segment
							ROS_INFO_STREAM("Mode: Line");
							length_path = distance(x_prev, y_prev, x_next, y_next);
							yaw_path 	= yaw_next;
							speed_path 	= SPEED_SETPOINT;
							curvature_path = 0.0;
							cx_path = std::numeric_limits<double>::infinity();
							cy_path = std::numeric_limits<double>::infinity();
						}
					}
					else{
						path_type 	= 'p';		// on-point-rotation
						ROS_INFO_STREAM("Mode: Point");
						length_path = 0.0;
						yaw_path 	= yaw_next;
						speed_path 	= 0.0;
						// curvature_path is positive for anti-clockwise turns
						curvature_path = sgn(angWrap(yaw_next-yaw_prev))*
							std::numeric_limits<double>::infinity();
						cx_path = x_next;
						cy_path = y_next;
					}
				}
				else{
					path_type 	= 'e';			// end of path reached
					ROS_INFO_STREAM("Mode: End of path reached");
					x_prev 		= x_next;
					y_prev 		= y_next;
					yaw_prev 	= yaw_next;
					length_path = 0.0;
					yaw_path 	= yaw_prev;
					speed_path 	= 0.0;
					curvature_path = 0.0;
					cx_path = std::numeric_limits<double>::infinity();
					cy_path = std::numeric_limits<double>::infinity();
				}
			}
		}

		// Generates control input for low level controller based on the path type
		void generateControl(){
			if (path_type == 'a')
			{
				yaw_d 		= atan2(y-cy_path, x-cx_path) + 
					sgn(curvature_path)*M_PI/2.0;
				yaw_error 	= angWrap(yaw-yaw_d);
				// l_error is positive if (x, y) lies to the left of the path-piece,
				// that is, if it lies towards the 'inside' of a positive-curavature
				// arc or the 'outside' of negative-curvature arc.
				l_error		= 1/curvature_path -
					sgn(curvature_path)*distance(cx_path, cy_path, x, y);
				speed_d 	= speed_path;
				omega_d 	= - ka*v*l_error*sinc(yaw_error) - kb*yaw_error + 
					(v*cos(yaw_error))/(1/curvature_path-l_error);
			}
			else if (path_type == 'l')
			{
				yaw_d 		= yaw_path;
				yaw_error 	= angWrap(yaw-yaw_d);
				// l_error is positive if (x, y) lies to the left of the path-piece,
				// that is, if the area of the triangle formed by (x_prev, y_prev),
				// (x_next, y_next) & (x, y) (in that order) is positive.
				l_error 	= 
					(x_prev*(y_next-y) + x_next*(y-y_prev) + x*(y_prev-y_next))/
					(2.0*length_path);	// height = area/base
				speed_d 	= speed_path;
				omega_d 	= - ka*v*l_error*sinc(yaw_error) - kb*yaw_error;
			}
			else if (path_type == 'p')
			{
				yaw_d 		= yaw_path;
				yaw_error 	= angWrap(yaw-yaw_d);
				l_error 	= distance(x, y, x_next, y_next);
				speed_d 	= speed_path;
				omega_d 	= -kb*yaw_error;
			}
			else // path_type == 'e'
			{
				yaw_d 		= yaw_path;
				yaw_error 	= angWrap(yaw-yaw_d);
				l_error 	= distance(x_prev, y_prev, x, y);
				speed_d 	= 0.0;
				omega_d 	= 0.0;
			}
		}

		// Necessary for the angular differences, not the individual angles
		double angWrap(double x){
			while (x <= -M_PI)
				x = x + 2*M_PI;
			while (x > M_PI)
				x = x - 2*M_PI;
			return x;
		}

		inline double distance(double x1, double y1, double x2, double y2){
			return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
		}

		inline double sinc(double x){
			return (abs(x)>0.001)?(sin(x)/x):1.0;
		}

		inline int sgn(double x) {
			return (0.0 < x) - (x < 0.0);
		}
};

int main(int argc,char* argv[]){
	ros::init(argc, argv, "unicycle_controller");
	UnicycleControl controller;
	
	ros::Rate loop_rate(sampling_rate);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
