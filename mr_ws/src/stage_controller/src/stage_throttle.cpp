/*
 * stage_controller.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: andreyminin
 */

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
#include <algorithm>

double car_length = 1.5;

double steering = 0;
double cmd_steering = 0.0;
double max_steering_rate = 1.0;
double max_steering = 0.5;

double throttle = 0;
double velocity = 0;
double cmd_throttle = 0;
double max_throttle_rate = 200.0;
double max_throttle = 100;
double max_velocity = 10;


double kMass = 500;
double kFriction = 1.0;
double kWindFriction = 0.01;
double kBrake = 3.0;
double kThrottle = 2.5;
double kVelExp = 0.8;

ros::Publisher twist_pub;
ros::Time last_timer_time;

void on_steering(const std_msgs::Float32& msg) {
  cmd_steering = msg.data;

}


void on_throttle(const std_msgs::Float32& msg) {
  cmd_throttle = msg.data;
}

double clamp(double cmd, double max_value)
{
  return copysign(std::min(std::abs(cmd), max_value), cmd);
}

double clamp(double value, double cmd, double max_value, double max_rate, double dt) {
  cmd = clamp(cmd, max_value);
  auto diff = (cmd - value);
  diff = clamp(diff, max_rate * dt);
  return value + diff;
}

double acc_from_throttle(double dt) {
  throttle = clamp(throttle, cmd_throttle, max_throttle, max_throttle_rate, dt);
  ROS_INFO_STREAM("throttle = " << throttle);
 
  double throttle_force = throttle > 0 ? kThrottle * throttle * exp(-velocity * kVelExp) :
                                         kBrake * throttle;
  double acc = 1.0 / kMass * (throttle_force - velocity * velocity * kWindFriction - kFriction * velocity);
  ROS_INFO_STREAM("Acc = " << acc);
  return acc;
}

void on_timer(const ros::TimerEvent& event) {
  auto t = ros::Time::now();
  auto dt = (t - last_timer_time).toSec();
  ROS_INFO_STREAM("dt = " << dt);
  last_timer_time = t;
  geometry_msgs::Twist cmd;

  velocity = std::max(0.0, clamp(velocity  + acc_from_throttle(dt) * dt, max_velocity));
  
  
  steering = clamp(steering, cmd_steering, max_steering, max_steering_rate, dt);
  ROS_INFO_STREAM("v = " << velocity <<" s = " << steering);
  cmd.linear.x = velocity;
  cmd.angular.z = velocity * tan(steering) / car_length;
  twist_pub.publish(cmd);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "stage_throttle_controller");
  ros::NodeHandle nh("~");
  car_length = nh.param("length", 1.5);
  ROS_INFO_STREAM("car length " << car_length);
  max_steering = nh.param("max_steering", 0.5);
  max_steering_rate = nh.param("max_steering_rate", 1.0);
  max_velocity = nh.param("max_velocity", 15);

  kMass = nh.param("mass", 500);
  kFriction = nh.param("friction", 1.0);
  kWindFriction = nh.param("wind_friction", 0.01);
  kBrake = nh.param("brake", 3.0);
  kThrottle = nh.param("throttle", 2.5);
  kVelExp = nh.param("exp", 0.8);


  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  
  auto steer_sub = nh.subscribe("steering", 1, on_steering);
  auto throttle_sub = nh.subscribe("throttle", 1, on_throttle);
  if (nh.param("/use_sim_time", false)) {
    while(ros::ok()) {
      ros::spinOnce();
    
      last_timer_time = ros::Time::now();
      if (!last_timer_time.isZero()) {
        break;
      } 
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  last_timer_time = ros::Time::now();
  ros::spin();
  return 0;
}


