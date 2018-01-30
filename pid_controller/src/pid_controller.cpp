#include<ros/ros.h>
#include<vector>
#include<geometry_msgs/Twist.h>
#include<tf/tf.h>
#include<std_msgs/Float64.h>
#include<nav_msgs/Odometry.h>
#define kp 1
#define ki 0
#define kd 0.5

#define PI 3.14159265
#define VELOCITY 1.0

#define a0 1
#define a1 2
#define a2 0
#define a3 0

#define sampling_rate 2000

class pid_control{
    public:
        pid_control(){
         this->previous_y.data = 0.0;
         this->previous_angle.data = 0.0;
         pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 100);
         sub_ = nh_.subscribe("odom", 100, &pid_control::odomCallback, this);
        }

        // nav_msgs::Odometry findDesiredPose(const nav_msgs::OdometryConstPtr& odom){
        //     nav_msgs::Odometry desired;
        //     std_msgs::Float64 X;
        //     X.data = odom->pose.pose.position.x;

        //     desired.pose.pose.position.x = odom->pose.pose.position.x;
        //     desired.pose.pose.position.y = a0 + a1*X.data + a2*X.data*X.data +a3*X.data*X.data*X.data;
        //     desired.pose.pose.position.z = odom->pose.pose.position.x;

        //     tf::Quaternion q(
        //         odom->pose.pose.orientation.x,
        //         odom->pose.pose.orientation.y,
        //         odom->pose.pose.orientation.z,
        //         odom->pose.pose.orientation.w);
        //     tf::Matrix3x3 m(q);
        //     double roll, pitch, yaw;
        //     m.getRPY(roll, pitch, yaw);

        //     desired.angular.x = vel->angular.x;
        //     desired.angular.y = vel->angular.y;
        //     if(desired.linear.y >= vel->linear.y)
        //         desired.angular.z = PI/2;
        //     else
        //         desired.angular.z = -PI/2;

        //     return desired;
        // }

        std_msgs::Float64 findXVel(std_msgs::Float64 y_vel){
            std_msgs::Float64 x_vel;
            x_vel.data = sqrt(VELOCITY*VELOCITY - y_vel.data*y_vel.data);
            return x_vel;
        }

        void odomCallback(const nav_msgs::OdometryConstPtr& odom){
            nav_msgs::Odometry desired;
            std_msgs::Float64 X;
            X.data = odom->pose.pose.position.x;

            desired.pose.pose.position.x = odom->pose.pose.position.x;
            desired.pose.pose.position.y = a0 + a1*X.data + a2*X.data*X.data +a3*X.data*X.data*X.data;
            desired.pose.pose.position.z = odom->pose.pose.position.x;

            tf::Quaternion q(
                odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);


            error_y.data = desired.pose.pose.position.y - odom->pose.pose.position.y;
            if(error_y.data>=0)
                error_angular.data = (PI/2) - yaw;
            else
                error_angular.data = -(PI/2) - yaw;

            output_y.data = (kp*error_y.data) + (kd*(error_y.data-previous_y.data)*sampling_rate);
            output_angle.data = (kp*error_angular.data) + (kd*(error_angular.data-previous_angle.data)*sampling_rate);

            geometry_msgs::Twist vel_output;
            
            vel_output.linear.y = output_y.data;
            vel_output.linear.x = findXVel(output_y).data;
            vel_output.linear.z = vel->linear.z;
            vel_output.angular.x = roll;
            vel_output.angular.y = pitch;
            vel_output.angular.z = output_angle.data;

            pub_.publish(vel_output);

        }
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
        std_msgs::Float64 error_y;
        std_msgs::Float64 error_angular;
        std_msgs::Float64 previous_y;
        std_msgs::Float64 previous_angle;
        std_msgs::Float64 output_y;
        std_msgs::Float64 output_angle;
};

int main(int argc,char* argv[]){
    ros::init(argc,argv,"pid_controller");
    pid_control estimate();
    ros::spinOnce();
    return 0;
}







