#include "../include/picar_driving/picar_driving.h"


Driver::Driver()
{
  // initialize variables
  msgReceived_ = false;
  lastTime_ = ros::Time::now();
  x_ = y_ = th_ = 0.0;

  // Initialize publisher
  pub_odom_ = n_.advertise<nav_msgs::Odometry>("/odom", 50);

  // Initialize subscriber
  sub_robot_twist_ = n_.subscribe("/cmd_vel", 1, &Driver::topicCallback_robotTwist, this);


  // initialize motor pins
  if ((pi_ = pigpio_start(NULL, NULL)) < 0) 
    ROS_WARN_STREAM("GPIO could not be initialized!");

  if(set_mode(pi_, PIN_ANG_EN1, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'PIN_ANG_EN1' could not be initialized!");
  if(set_mode(pi_, PIN_ANG_EN2, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'PIN_ANG_EN2' could not be initialized!");
  if(set_mode(pi_, PIN_LIN_EN1, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'PIN_LIN_EN1' could not be initialized!");
  if(set_mode(pi_, PIN_LIN_EN2, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'PIN_LIN_EN2' could not be initialized!");
  if(set_mode(pi_, PIN_LIN_PWM, PI_OUTPUT) != 0)
    ROS_WARN_STREAM("PIN 'PIN_LIN_PWM' could not be initialized!");

  ROS_INFO_STREAM("Sensor Ultrasonic successfully initialized.");
}

Driver::~Driver()
{

}

void Driver::configure()
{
//  ROS_INFO_STREAM("Configuration successfully completed.");
}

// update function called by node
void Driver::update()
{
  if (msgReceived_)
    driveRobot();
  else
    stopRobot();

  computeOdometry();

  msgReceived_ = false;
}



// Input/Output of Subscriber/Publisher

void Driver::readInput()
{

}

void Driver::writeOutput()
{

}



// Subscriber callback functions

void Driver::topicCallback_robotTwist(const geometry_msgs::Twist& msg)
{
  in_robot_twist_ = msg;
  msgReceived_ = true;
  update();
}



// Additional Functions
void Driver::driveRobot()
{
  if (in_robot_twist_.linear.x > THRESHOLD_LIN) 
  {
    gpio_write(pi_, PIN_LIN_EN1, HIGH);
    gpio_write(pi_, PIN_LIN_EN2, LOW);
    float pwm_cycle = (std::min(in_robot_twist_.linear.x, 10.0)*PWM_MAX/10.0);
    set_PWM_dutycycle(pi_, PIN_LIN_PWM, abs((int) pwm_cycle)); 
  }
  else if (in_robot_twist_.linear.x < -THRESHOLD_LIN)
  {
    gpio_write(pi_, PIN_LIN_EN1, LOW);
    gpio_write(pi_, PIN_LIN_EN2, HIGH);
    int pwm_cycle = (int) std::min(in_robot_twist_.linear.x, 10.0)*PWM_MAX/10.0;
    set_PWM_dutycycle(pi_, PIN_LIN_PWM, abs(pwm_cycle));
  }
  else 
  {
    gpio_write(pi_, PIN_LIN_EN1, LOW);
    gpio_write(pi_, PIN_LIN_EN2, LOW);
    gpio_write(pi_, PIN_LIN_PWM, LOW);
  }

  if (in_robot_twist_.angular.z > THRESHOLD_ANG) 
  {
    gpio_write(pi_, PIN_ANG_EN1, LOW);
    gpio_write(pi_, PIN_ANG_EN2, HIGH);
  }
  else if (in_robot_twist_.angular.z < -THRESHOLD_ANG)
  {
    gpio_write(pi_, PIN_ANG_EN1, HIGH);
    gpio_write(pi_, PIN_ANG_EN2, LOW);
  }  
  else
  {
    gpio_write(pi_, PIN_ANG_EN1, LOW);
    gpio_write(pi_, PIN_ANG_EN2, LOW);
  }

}

void Driver::stopRobot()
{
  gpio_write(pi_, PIN_LIN_EN1, LOW);
  gpio_write(pi_, PIN_LIN_EN2, LOW);
  gpio_write(pi_, PIN_LIN_PWM, LOW);

  gpio_write(pi_, PIN_ANG_EN1, LOW);
  gpio_write(pi_, PIN_ANG_EN2, LOW);
}

void Driver::computeOdometry()
{
  ros::Time currentTime = ros::Time::now();
  double dt = (currentTime - lastTime_).toSec();

  // compute current velocity
  double vx = in_robot_twist_.linear.x * VEL_MAX;
  double vth = in_robot_twist_.angular.z * VEL_MAX * 0.1;

  //compute odometry in a typical way given the velocities of the robot
  x_ += (vx * cos(th_)) * dt;
  y_ += (vx * sin(th_)) * dt;
  th_ += vth * dt;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

  //first, we'll publish the transform over tf
  out_odom_tf_.header.stamp = currentTime;
  out_odom_tf_.header.frame_id = "odom";
  out_odom_tf_.child_frame_id = "base_link";

  out_odom_tf_.transform.translation.x = x_;
  out_odom_tf_.transform.translation.y = y_;
  out_odom_tf_.transform.translation.z = 0.0;
  out_odom_tf_.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster_.sendTransform(out_odom_tf_);

  //next, we'll publish the odometry message over ROS
  out_odom_.header.stamp = currentTime;
  out_odom_.header.frame_id = "odom";

  //set the position
  out_odom_.pose.pose.position.x = x_;
  out_odom_.pose.pose.position.y = y_;
  out_odom_.pose.pose.position.z = 0.0;
  out_odom_.pose.pose.orientation = odom_quat;

  //set the velocity
  out_odom_.child_frame_id = "base_link";
  out_odom_.twist.twist.linear.x = vx;
  out_odom_.twist.twist.linear.y = 0;
  out_odom_.twist.twist.angular.z = vth;

  //publish the message
  pub_odom_.publish(out_odom_);

  lastTime_ = currentTime;
}



















