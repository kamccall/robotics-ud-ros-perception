#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
using namespace std;

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
  // create service to send movement commands to robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x  = lin_x;
  srv.request.angular_z = ang_z;
  
  if (!client.call(srv))
    ROS_ERROR("failed to call service to drive to target position...");
}
