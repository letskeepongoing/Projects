#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h> 
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <my_turtlebot_main/dir_out.h>

int main(int argc, char** argv){
  ros::init(argc, argv,"mainClient"); //node for service
  ros::NodeHandle nh;
  
  ros::ServiceClient turtle_giver =nh.serviceClient<my_turtlebot_main::dir_out>("/escaping_maze");
  my_turtlebot_main::dir_out srv;
  
   if (turtle_giver.call(srv)){
       ROS_INFO("requesting direction");
   }
   else
  {
    ROS_ERROR("Failed to call service /escaping_maze");
    return 1;
  }
  
 //ros::spin();
 return 0;
   
}

