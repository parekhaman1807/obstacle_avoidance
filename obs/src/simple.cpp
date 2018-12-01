#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple");
	ros::NodeHandle nh;
	ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
	ros::Rate loopRate(1);
	unsigned int count=0;
	while(ros::ok())
	{
		geometry_msgs::PoseStamped coords;
		if (count<10)		
		{
			coords.pose.position.z=2;
			coords.pose.position.x=2;
			coords.pose.position.y=2;
		}
		else
		{
			coords.pose.position.z=0;
			coords.pose.position.x=0;
			coords.pose.position.y=0;
		}
		coords.pose.orientation.z=0;		
		coords.pose.orientation.x=0;
		coords.pose.orientation.y=0;
		coords.pose.orientation.w=0;
		p.publish(coords);		
		ros::spinOnce();
		loopRate.sleep();
		count++;
	}
return 0;
}
