#include<iostream.h>
#include<geometry_msgs/Point.h>
#include<ros/ros.h>

geometry_msgs::Point p;

void PointCallBack(const geometry_msgs::Point& point)
{
  p=point;
}

int main(int argc,char **argv)
{
  ros::init(argc,argv,"subscribe_position");
  ros::NodeHandle nh;

  ros::Subscriber s=nh.subscribe("odometry",10,PointCallBack);

  ros::Spin();

  return 0;
}
