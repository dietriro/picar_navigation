#ifndef PICAR_SENSOR_ULTRASONIC_H_
#define PICAR_SENSOR_ULTRASONIC_H_


#include <ros/ros.h>
#include <pigpiod_if2.h>
#include <time.h>
#include <algorithm>

// Include ROS-Msgs
// Publisher
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

// Subscriber
#include <geometry_msgs/Twist.h>

#define HIGH 1
#define LOW 0

static const int PIN_ANG_EN1 = 22, PIN_ANG_EN2 = 23, PIN_LIN_EN1 = 24, PIN_LIN_EN2 = 25,
                 PIN_LIN_PWM = 4, PWM_MAX = 255, PWM_MIN = 0;
static const float THRESHOLD_LIN = 0.25, THRESHOLD_ANG = 0.1, VEL_MAX = 1.0;

/**
 * @class 
 * @brief Server that handles all the path segments
 */
class Driver
{
public:
  /**
   * @brief Constructor
   */
  Driver();

  /**
   * @brief Destructor
   */
  ~Driver();

  void configure();
  void update();

  // Input/Output of Subscriber/Publisher
  void readInput();
  void writeOutput();

  // Subscriber callback functions
  void topicCallback_robotTwist(const geometry_msgs::Twist& msg);

  // Additional Functions
  void driveRobot();
  void stopRobot();
  void computeOdometry();

protected:
  // Nodehandle
  ros::NodeHandle n_;

  // Publisher
  ros::Publisher pub_odom_;
  tf::TransformBroadcaster odom_broadcaster_;

  // Subscriber
  ros::Subscriber sub_robot_twist_;

  // Publishing messages
  geometry_msgs::TransformStamped out_odom_tf_;
  nav_msgs::Odometry out_odom_;

  // Subscribed messages
  geometry_msgs::Twist in_robot_twist_;

  // Usual membervariables
  bool msgReceived_;
  double x_, y_, th_;
  ros::Time lastTime_;
  int pi_;
};

#endif
























