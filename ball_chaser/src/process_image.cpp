#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
using namespace std;

ros::ServiceClient client;
float velocity_forward =  0.15;
float velocity_left    =  0.20;
float velocity_right   = -0.20;

void drive_robot(float lin_x, float ang_z)
{
  // create service to send movement commands to robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x  = lin_x;
  srv.request.angular_z = ang_z;
  
  if (!client.call(srv))
    ROS_ERROR("failed to call service to drive to target position...");
}

void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  
  // bool found_ball = false;   // robot stationary (unless and until finds white ball)
  float value_x    = 0.0;    
  float value_z    = 0.0;
  int column_found = 0;
  
  // loop through pixels to look for white ball (by looking for white_pixel pixel value)
  for (int i = 0; i < img.height * img.step; i++)
  {
    if (img.data[i] == white_pixel)            // white ball found, so set movement values
    {
      value_x = velocity_forward;              // will move forward in all cases
      column_found = i % img.step;             // identify which 'column' in image ball found
      
      if (column_found < img.step / 3)         // found on left side
        value_z = velocity_left;
      else if (column_found > img.step*2 / 3)  // found on right side
        value_z = velocity_right;
    }
  }
  
  // send movement commands to robot (or zero values if ball not found)
  drive_robot(value_x, value_z);
}

int main(int argc, char** argv)
{
  // initialize process_image node and create nodehandle access to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;
  
  // define client service that will request service from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  
  // define callback that will continously process raw image data on /camera/rgb/image_raw topic
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
  
  ros::spin();
  
  return 0;
}
