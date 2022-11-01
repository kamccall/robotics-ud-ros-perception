#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS publisher motor commands
ros::Publisher motor_command_publisher;

// defines a handle_drive_request callback function that executes when a drive_bot service is requested.
// function should publish the requested linear_x and angular_z velocities to the robot wheel joints.
// after publishing the requested velocities, a message feedback should be returned with the requested wheel velocities.

// callback function executes whenever command_robot service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  geometry_msgs::Twist motor_command;

  motor_command.linear.x  = req.linear_x;
  motor_command.angular.z = req.angular_z; 

  motor_command_publisher.publish(motor_command);  // sends geometry_msgs/Twist message to /cmd_vel topic

  res.msg_feedback = "setting linear.x: " + std::to_string(motor_command.linear.x) + " angular.z: " + std::to_string(motor_command.angular.z);
  ROS_INFO_STREAM(res.msg_feedback);
  
  return true;
}

int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc, argv, "drive_bot");

  // create roshandle object
  ros::NodeHandle n;

  // inform ROS master that we will publish messages of type geometry/Twist on robot actuation topic
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // advertise service endpoint so other clients can send movement commands to robot
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

  if (service)
  {
    std::cout << "service started: ready to receive movement commands on command_robot endpoint...\n";
    ros::spin();
  }
  else
  {
    ROS_WARN("failed to start drive request service...");
  }
  
  return 0;
}
