#include "ros/ros.h"
#include "0xRobotcpplib.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/Twist.h>
#include <ros0xrobot/gpOutStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>

class Ros0xRobotNode
{
public:
  Ros0xRobotNode(ros::NodeHandle n);
  virtual ~Ros0xRobotNode();

public:
  int init();
  void spin();
  void publish();
  void cmdVelCallback(const geometry_msgs::TwistConstPtr&);
  void gpoutCallback(const std_msgs::UInt8&);

protected:
  ros::NodeHandle n;
  // for odom->base_link transform
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;

  // for resolving tf names.
  std::string tf_prefix;
  std::string frame_id_odom;
  std::string frame_id_base_link;
  std::string frame_id_sonar;
  std::string frame_id_irRangeSensor;
  std::string frame_id_imu;
  std::string frame_id_gpOutStatus;

  // robot instance
  lib0xRobotCpp* OxRobot;
  std::string serial_port;

  int32 leftCount, rightCount, leftCountPrev, rightCountPrev, deltaLeftCount,
      deltaRightCount;
  double x, y, deltaX, deltaY, deltaUpdate;
  double theta, deltaTheta, theta_Deg;
  double axelLength;
  double wheelDiameter;
  double distancePerCount;
  int countsPerRev;
  bool enableSonar;
  bool enableIRRangeSensor;
  bool enableImu;
  uint8 gpOutStatus;
  ros0xrobot::gpOutStatus gpOutStatusMsg;

  uint32 loopCount;
  uint8 updateFrequency;
  double velocityX, velocityY, velocityTheta;
  void getPosition(nav_msgs::Odometry* position);
  void setPoseCovariance(nav_msgs::Odometry* position);
  void setTwistCovariance(nav_msgs::Odometry* position);
  ros::Publisher pose_pub;
  nav_msgs::Odometry position;
  ros::Publisher sonarPub;
  ros::Publisher irRangeSensorPub;
  ros::Subscriber sub;
  ros::Subscriber sub_gpout;
  ros::Publisher gpioOutPub;
  sensor_msgs::Imu imuData;

  // IMU variables
  ros::Publisher imuPub;
  bool imuInit();
  bool getImu(sensor_msgs::Imu* imuData);
  uint8 X, Y, Z;
  float getHeading(int16* magValue);
  float getTiltHeading(float* magValue, float* accelValue);
  int16 gyroXYZ_Zero[3];
  float gyro_angle;
  double orientation_imu;
};

Ros0xRobotNode::Ros0xRobotNode(ros::NodeHandle nh) : n(nh)
{
  // read runtime parameters
  n.param("port", serial_port, std::string("/dev/ttyUSB0"));
  ROS_INFO("Ros0xRobotNode: using port: [%s]", serial_port.c_str());

  n.param("CountsPerRev", countsPerRev, 2011);
  ROS_INFO("Ros0xRobotNode: using counts per revolution: %d", countsPerRev);

  n.param("WheelDiameter", wheelDiameter, 250.0);
  ROS_INFO("Ros0xRobotNode: using Wheel Diameter: %f", wheelDiameter);

  n.param("AxelLength", axelLength, 600.0);
  ROS_INFO("Ros0xRobotNode: using Axel Length: %f", axelLength);

  n.param("enableSonar", enableSonar, false);
  ROS_INFO("Ros0xRobotNode: sonar enabled: %d", enableSonar);

  n.param("enableIRRangeSensor", enableIRRangeSensor, false);
  ROS_INFO("Ros0xRobotNode: IR range sensor enabled: %d", enableIRRangeSensor);

  n.param("enableImu", enableImu, false);
  ROS_INFO("Ros0xRobotNode: Imu enabled: %d", enableImu);

  // One can use name <param> in roslaunch files for Multi Robot Systems
  // This will result in the frame_ids being set to /nameparam/odom etc,
  // rather than /odom. This is useful for Multi Robot Systems.
  // See ROS Wiki for further details.
  tf_prefix = tf::getPrefixParam(n);
  frame_id_odom = tf::resolve(tf_prefix, "odom");
  frame_id_base_link = tf::resolve(tf_prefix, "base_link");
  frame_id_sonar = tf::resolve(tf_prefix, "sonar_frame");
  frame_id_irRangeSensor = tf::resolve(tf_prefix, "irRange_frame");
  frame_id_imu = tf::resolve(tf_prefix, "imu_frame");
  frame_id_gpOutStatus = tf::resolve(tf_prefix, "gpOutStatus_frame");
  gpOutStatusMsg.header.frame_id = frame_id_gpOutStatus;

  sub = n.subscribe("cmd_vel", 1,
                    (boost::function<void(const geometry_msgs::TwistConstPtr&)>)
                        boost::bind(&Ros0xRobotNode::cmdVelCallback, this, _1));

  sub_gpout =
      n.subscribe("gpio/gpout", 1, &Ros0xRobotNode::gpoutCallback, this);

  gpioOutPub = n.advertise<ros0xrobot::gpOutStatus>("gpio/gpOutStatus", 50);

  pose_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  if (enableSonar)
    sonarPub = n.advertise<sensor_msgs::PointCloud>("sonar", 50);
  if (enableIRRangeSensor)
    irRangeSensorPub = n.advertise<sensor_msgs::PointCloud>("irRange", 50);
  if (enableImu)
    imuPub = n.advertise<sensor_msgs::Imu>("imu", 50);

  distancePerCount = (wheelDiameter * 22 / 7.0) / countsPerRev;
  ROS_INFO("Ros0xRobotNode: distancePerCount = %f", distancePerCount);

  // set imu covariance
  imuData.linear_acceleration_covariance[0] = 0.5;
  imuData.linear_acceleration_covariance[1] = 0.0;
  imuData.linear_acceleration_covariance[2] = 0.0;
  imuData.linear_acceleration_covariance[3] = 0.0;
  imuData.linear_acceleration_covariance[4] = 0.5;
  imuData.linear_acceleration_covariance[5] = 0.0;
  imuData.linear_acceleration_covariance[6] = 0.0;
  imuData.linear_acceleration_covariance[7] = 0.0;
  imuData.linear_acceleration_covariance[8] = 0.5;

  imuData.angular_velocity_covariance[0] = 0.001;
  imuData.angular_velocity_covariance[1] = 0.0;
  imuData.angular_velocity_covariance[2] = 0.0;
  imuData.angular_velocity_covariance[3] = 0.0;
  imuData.angular_velocity_covariance[4] = 0.001;
  imuData.angular_velocity_covariance[5] = 0.0;
  imuData.angular_velocity_covariance[6] = 0.0;
  imuData.angular_velocity_covariance[7] = 0.0;
  imuData.angular_velocity_covariance[8] = 0.001;

  imuData.orientation_covariance[0] = 0.1;
  imuData.orientation_covariance[1] = 0.0;
  imuData.orientation_covariance[2] = 0.0;
  imuData.orientation_covariance[3] = 0.0;
  imuData.orientation_covariance[4] = 0.1;
  imuData.orientation_covariance[5] = 0.0;
  imuData.orientation_covariance[6] = 0.0;
  imuData.orientation_covariance[7] = 0.0;
  imuData.orientation_covariance[8] = 0.1;

  X = 0;
  Y = 1;
  Z = 2;
  gyro_angle = 0.0;
  orientation_imu = 0;

  loopCount = 0;
  updateFrequency = 10;
  velocityX = 0.0;
  velocityY = 0.0;
  velocityTheta = 0.0;

  gpOutStatus = 0;

  // Initialize odometry
  x = 0;
  y = 0;
  deltaX = 0;
  deltaY = 0;
  deltaUpdate = 0;

  leftCount = 0;
  rightCount = 0;
  leftCountPrev = 0;
  rightCountPrev = 0;
  deltaLeftCount = 0;
  deltaRightCount = 0;

  theta = 0;
  deltaTheta = 0;
  theta_Deg = 0;
}

Ros0xRobotNode::~Ros0xRobotNode()
{
  // stop motors
  OxRobot->stop(OxRobot->comm_handle);
  // disconnect
  OxRobot->disconnect_comm(OxRobot->comm_handle);
}

int Ros0xRobotNode::init()
{
  // new robot setup
  OxRobot = new lib0xRobotCpp();
  // establish connection
  OxRobot->comm_handle = OxRobot->connect_comm(serial_port.c_str());
  // stop motors
  OxRobot->stop(OxRobot->comm_handle);
  // reset encoders
  OxRobot->resetMotorEncoderCount(OxRobot->comm_handle);
  // set acceleration
  OxRobot->setAcceleration(OxRobot->comm_handle, 4);
  // set safety off
  OxRobot->setSafety(OxRobot->comm_handle, 0);

  if (enableImu)
    OxRobot->imuInit(OxRobot->comm_handle, gyroXYZ_Zero);

  return 0;
}

void Ros0xRobotNode::spin()
{

  float sonarSensorDataX[8];
  float sonarSensorDataY[8];
  float irRangeSensorDataX[8];
  float irRangeSensorDataY[8];
  ros::Rate loop_rate(updateFrequency);

  while (ros::ok())
  {
    // get position
    getPosition(&position);

    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    position.header.stamp = ros::Time::now();

    // publish odometry msg
    pose_pub.publish(position);

    // publishing transform odom->base_link
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = frame_id_odom;
    odom_trans.child_frame_id = frame_id_base_link;
    odom_trans.transform.translation.x = position.pose.pose.position.x;
    odom_trans.transform.translation.y = position.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = position.pose.pose.orientation;

    odom_broadcaster.sendTransform(odom_trans);

    if ((enableSonar) &&
        (OxRobot->getSonarSensorRangeArray(
             OxRobot->comm_handle, sonarSensorDataX, sonarSensorDataY) != 0))
    {
      sensor_msgs::PointCloud cloud;

      cloud.header.stamp = position.header.stamp;
      cloud.header.frame_id = frame_id_sonar;

      // get sonar ranges
      for (int i = 0; i < 8; i++)
      {
        geometry_msgs::Point32 p;
        p.x = sonarSensorDataX[i];
        p.y = sonarSensorDataY[i];
        p.z = 0.0;
        cloud.points.push_back(p);
      }

      sonarPub.publish(cloud);
    }

    if ((enableIRRangeSensor) && (OxRobot->getIRSensorRangeArray(
                                      OxRobot->comm_handle, irRangeSensorDataX,
                                      irRangeSensorDataY) != 0))
    {
      sensor_msgs::PointCloud cloud;

      cloud.header.stamp = position.header.stamp;
      cloud.header.frame_id = frame_id_irRangeSensor;

      // get sonar ranges
      for (int i = 0; i < 8; i++)
      {
        geometry_msgs::Point32 p;
        p.x = irRangeSensorDataX[i];
        p.y = irRangeSensorDataY[i];
        p.z = 0.0;
        cloud.points.push_back(p);
      }

      irRangeSensorPub.publish(cloud);
    }

    if ((enableImu) && (getImu(&imuData)))
    {
      imuData.header.stamp = ros::Time::now();
      imuData.header.frame_id = frame_id_imu;
      imuPub.publish(imuData);
    }

    // GPIO out pins can be controlled by simple uint8 message type
    // example (from command line): rostopic pub -1 /ros0xrobot/gpio/gpout
    // std_msgs/UInt8 8
    // This will send 8 to GPIO out. It will set pin no. 3
    // To set pin no. 0, send 1.

    // get GPIO out status from robot
    OxRobot->getGPIOPannelLED(OxRobot->comm_handle, &gpOutStatus);

    // pupolate message
    gpOutStatusMsg.gpOutStatus = gpOutStatus;
    gpOutStatusMsg.header.stamp = ros::Time::now();
    gpOutStatusMsg.header.seq++;
    // Publisg GPIO out status
    gpioOutPub.publish(gpOutStatusMsg);

    loopCount++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Ros0xRobotNode : Spin Done\n");
}

bool Ros0xRobotNode::imuInit()
{
  int16 gyroXYZ[3];
  int32 gyroX = 0, gyroY = 0, gyroZ = 0;
  for (int i = 0; i < 64; i++)
  {
    if (OxRobot->getGyroXYZ(OxRobot->comm_handle, gyroXYZ) != 0)
    {
      gyroX += gyroXYZ[X];
      gyroY += gyroXYZ[Y];
      gyroZ += gyroXYZ[Z];
    }
    else
      return false;
  }

  gyroXYZ_Zero[X] = (int16)(gyroX >> 6);
  gyroXYZ_Zero[Y] = (int16)(gyroY >> 6);
  gyroXYZ_Zero[Z] = (int16)(gyroZ >> 6);

  return true;
}

bool Ros0xRobotNode::getImu(sensor_msgs::Imu* imuData)
{
  double orientation_angle = 0.0;
  int16 accelerometerXYZ[3], gyroXYZ[3], magXYZ[3];
  float accelerometerXYZ_g[3];
  float magXYZ_gauss[3];

  // initialize imu data
  imuData->linear_acceleration.x = 0.0;
  imuData->linear_acceleration.y = 0.0;
  imuData->linear_acceleration.z = 0.0;
  if (OxRobot->getAccelerometerXYZ(OxRobot->comm_handle, accelerometerXYZ) != 0)
  {
    // Acceleration (g) = (16 bit Raw data) * 0.004   for +/- 8g range
    accelerometerXYZ_g[X] = (accelerometerXYZ[X] >> 4) * 0.004;
    accelerometerXYZ_g[Y] = (accelerometerXYZ[Y] >> 4) * 0.004;
    accelerometerXYZ_g[Z] = (accelerometerXYZ[Z] >> 4) * 0.004;
    imuData->linear_acceleration.x = accelerometerXYZ_g[X] * 9.80665F;
    imuData->linear_acceleration.y = accelerometerXYZ_g[Y] * 9.80665F;
    imuData->linear_acceleration.z = accelerometerXYZ_g[Z] * 9.80665F;

    // tilt compensated orientation requires acceleration data
    if (OxRobot->getMagnetometerXYZ(OxRobot->comm_handle, magXYZ) != 0)
    {
      magXYZ_gauss[X] = magXYZ[X] / 1100.0;
      magXYZ_gauss[Y] = magXYZ[Y] / 1100.0;
      magXYZ_gauss[Z] = magXYZ[Z] / 980.0;
      orientation_angle =
          OxRobot->getTiltHeading(OxRobot->comm_handle, magXYZ_gauss,
                                  accelerometerXYZ_g) *
          M_PI / 180.0;
      imuData->orientation = tf::createQuaternionMsgFromYaw(orientation_angle);
    }
    else
      return false;
  }
  else
    return false;

  imuData->angular_velocity.x = 0.0;
  imuData->angular_velocity.y = 0.0;
  imuData->angular_velocity.z = 0.0;

  if (OxRobot->getGyroXYZ(OxRobot->comm_handle, gyroXYZ) != 0)
  {
    imuData->angular_velocity.x =
        (gyroXYZ[X] - gyroXYZ_Zero[X]) * 0.00175 * M_PI / 180.0;
    imuData->angular_velocity.y =
        (gyroXYZ[Y] - gyroXYZ_Zero[Y]) * 0.00175 * M_PI / 180.0;
    imuData->angular_velocity.z =
        (gyroXYZ[Z] - gyroXYZ_Zero[Z]) * 0.00175 * M_PI / 180.0;
  }
  else
    return false;

  return true;
}

void Ros0xRobotNode::getPosition(nav_msgs::Odometry* position)
{
  int32 leftMotorCount, rightMotorCount;

  // get left and right motor count
  OxRobot->getLeftMotorCount(OxRobot->comm_handle, &leftMotorCount);
  OxRobot->getRightMotorCount(OxRobot->comm_handle, &rightMotorCount);
  leftCount = leftMotorCount;
  rightCount = rightMotorCount;

  // find incremental count
  deltaLeftCount = leftCount - leftCountPrev;
  deltaRightCount = rightCount - rightCountPrev;

  OxRobot->getDeltaPosition(OxRobot->comm_handle, deltaLeftCount,
                            deltaRightCount, theta, distancePerCount,
                            axelLength, &deltaX, &deltaY, &deltaTheta);

  x += deltaX;
  y += deltaY;
  theta += deltaTheta;
  theta_Deg = (180.0 * 7.0 / 22.0) * theta;

  position->pose.pose.position.x = x / 1000;
  position->pose.pose.position.y = y / 1000;
  position->pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
  setPoseCovariance(position);

  velocityX = OxRobot->getVelocityX(
      OxRobot->comm_handle, deltaX, updateFrequency);
  velocityY = OxRobot->getVelocityY(
      OxRobot->comm_handle, deltaY, updateFrequency);
  velocityTheta = OxRobot->getVelocityTheta(
      OxRobot->comm_handle, deltaTheta, updateFrequency);

  position->twist.twist.linear.x = 
      sqrt(velocityX*velocityX + velocityY*velocityY);
  position->twist.twist.angular.z = velocityTheta;
  setTwistCovariance(position);

  leftCountPrev = leftCount;
  rightCountPrev = rightCount;
}

void Ros0xRobotNode::setPoseCovariance(nav_msgs::Odometry* position)
{
  position->pose.covariance[0] = 0.25;
  position->pose.covariance[7] = 0.25;
  position->pose.covariance[14] = 0.25;
  position->pose.covariance[21] = 0.25;
  position->pose.covariance[28] = 0.25;
  position->pose.covariance[35] = 0.25;
}

void Ros0xRobotNode::setTwistCovariance(nav_msgs::Odometry* position)
{
  position->twist.covariance[0] = 0.04;
  position->twist.covariance[7] = 0.04;
  position->twist.covariance[14] = 0.04;
  position->twist.covariance[21] = 0.09;
  position->twist.covariance[28] = 0.09;
  position->twist.covariance[35] = 0.09;
}

void Ros0xRobotNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  float velocityRight, velocityLeft, radiusOfCurv;
  ROS_INFO("%f, %f", msg->linear.x, msg->angular.z);

  OxRobot->setVelocity2D(OxRobot->comm_handle, msg->linear.x, msg->angular.z,
                         axelLength / 1000.0);

  OxRobot->forward(OxRobot->comm_handle);
}

void Ros0xRobotNode::gpoutCallback(const std_msgs::UInt8& msg)
{
  OxRobot->setGPIOPannelLED(OxRobot->comm_handle, msg.data);
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "ros0xRobotNode");
  ros::NodeHandle n(std::string("~"));

  Ros0xRobotNode* node = new Ros0xRobotNode(n);

  if (node->init() != 0)
  {
    ROS_FATAL("Ros0xRobotNode: ROS node setup failed... \n");
    return -1;
  }

  node->spin();

  ROS_INFO("Ros0xRobotNode : Done\n");

  delete node;

  return 0;
}
