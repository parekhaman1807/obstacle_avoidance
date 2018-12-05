#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<ros/ros.h>
#include<math.h>
// #include<trajectory_msgs/MultiDOFJointTrajectoryPoint>

geometry_msgs::Point p1, p2, v1, vg;

float a = 1.4, k1 = 1;
float b = 10, c = 27, k2;
float modV1;
float modVg;

void getHummingbird(const geometry_msgs::Pose& point)
{
	v1.x = 100*(-p1.x + point.position.x);
	v1.y = 100*(-p1.y + point.position.y);
	v1.z = 100*(-p1.z + point.position.z);
	p1 = point.position;
}

void getFirefly(const geometry_msgs::Pose& point)
{
    p2 = point.position;
}

float dot(geometry_msgs::Point diff3, geometry_msgs::Point vg1){
	return (vg1.x * diff3.x +  vg1.y * diff3.y + vg1.z * diff3.z);
}
float cosy(){
	return (k1 * v1.x / modV1 + k2 * vg.x / modVg) / (k1 + k2);
}
float sosy(){
	return (k1 * v1.y / modV1 + k2 * vg.y / modVg) / (k1 + k2);
}
int main(int argc, char **argv)
{
  ros::init(argc,argv,"subscribe_position");
  ros::NodeHandle nh;

  ros::Subscriber s1=nh.subscribe("/firefly/ground_truth/pose",3,getFirefly );
  ros::Subscriber s2=nh.subscribe("/hummingbird/ground_truth/pose",3,getHummingbird);
  ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);
  geometry_msgs::Point diff2;
	diff2.x = p2.x - p1.x;
	diff2.y = p2.y - p1.y;
	diff2.z = p2.z - p1.z;
  ros::Rate loopRate(100);
  loopRate.sleep();

  while(ros::ok())
  {
		geometry_msgs::Point diff;
		diff.x = p2.x - p1.x;
		diff.y = p2.y - p1.y;
		diff.z = p2.z - p1.z;

		vg.x = -100*(diff.x-diff2.x);
		vg.y = -100*(diff.y-diff2.y);
		vg.z = -100*(diff.z-diff2.z);

		diff2 = diff;


	    float distancesq =diff.x*diff.x + diff.y*diff.y + diff.z*diff.z + 0.01;
	    float e = pow(distancesq, 0.5);
	    float r;

		// float c = (vg.x * diff.x +  vg.y * diff.y + vg.z * diff.z);
	    k2 = b / (c * dot(diff, v1) + 1);
		r = a + k1 * dot(diff, v1) / sqrt(distancesq) + k2 * dot(diff, vg) / sqrt(distancesq);
		modV1 = sqrt(v1.x * v1.x + v1.y * v1.y) + 0.00001;
		modVg = sqrt(vg.x * vg.x + vg.y * vg.y) + 0.00001;
		float R = pow (pow(diff.x, 2) + pow(diff.y, 2), 0.5);
		float f = sqrt(p1.x*p1.x + p1.y*p1.y + (p1.z-3)*(p1.z-3));
		// float V = sqrt(vg.x*vg.x + vg.y*vg.y);
		// float V1 = sqrt(v1.x*v1.x + v1.y*v1.y);

	    geometry_msgs::PoseStamped coords;
	    coords.header.stamp = ros::Time::now();
	    coords.pose.position.z=3;

	    // if (distancesq<=r && diff.x!=0 && diff.y!=0)
	    // {
	    //   coords.pose.position.x=p2.x+(diff.y/r);
	    //   coords.pose.position.y=p2.y-(diff.x/r);
	    // }

	    // else if ((diff.x==0 || diff.y==0))
	    // {
	    //   coords.pose.position.x=0;
	    //   coords.pose.position.y=0;
	    // }
	    // if (e<r)
	    // 	{  
	    // 		// ROS_INFO_STREAM("Hello");
	    // 		coords.pose.position.x=p1.x-(diff.y*(r+0.3))/R;
	    // 	   coords.pose.position.y=p1.y+(diff.x*(r+0.3))/R;
	    // 	   // ROS_INFO_STREAM(coords.pose.position);
	    // 		}
	    // if()

	     if(modV1 < 0.001 && abs(v1.z) < 0.001 && f<r && f>0.1)
	     {	
	     	coords.pose.position.x=p1.x-(p1.x*(r+0.3))/sqrt(p1.x*p1.x + p1.y*p1.y);
	     	coords.pose.position.y=p1.y-(p1.y*(r+0.3))/sqrt(p1.x*p1.x + p1.y*p1.y);
	     }
		else if (e < r)
		{  
			if (modV1 < 0.001 && abs(v1.z) > 0.001)
			{
				coords.pose.position.x=p2.x + 15*r;
		    	coords.pose.position.y=p2.y;
		    	ROS_INFO_STREAM("Here");
		    	ROS_INFO_STREAM(coords.pose.position);
			}
			if(modV1  < 0.001 && abs(v1.z) < 0.001 && f > r)
			{
				ROS_INFO_STREAM("Hello");
				coords.pose.position.x=p2.x-(diff.y*(r+0.3))/R;
		    	coords.pose.position.y=p2.y+(diff.x*(r+0.3))/R;	
			}
			else
			{  	// ROS_INFO_STREAM("Hello");
				// coords.pose.position.x=p1.x-(diff.y*(r+0.3))/R;
		  		// coords.pose.position.y=p1.y+(diff.x*(r+0.3))/R;

		    	if (-diff.y * v1.x + diff.x * v1.y < 0)
		    	{
		    		coords.pose.position.x = p2.x - (r + 0.3) * sosy();
		    	 	coords.pose.position.y = p2.y + (r + 0.3) * cosy();
		    	}
		    	else 
		    	{		
		    	 	coords.pose.position.x = p2.x + (r + 0.3) * sosy();
		    		coords.pose.position.y = p2.y - (r + 0.3) * cosy();
				}   // ROS_INFO_STREAM(coords.pose.position);
			}
		}
	    else
	    {
	    	coords.pose.position.x=0;
	    	coords.pose.position.y=0;
	    }
	    

	    coords.pose.orientation.z=0;
	    coords.pose.orientation.x=0;
	    coords.pose.orientation.y=0;
	    coords.pose.orientation.w=0;
	     
	    ROS_INFO_STREAM(r);
	    // ROS_INFO_STREAM("Hello");
	    //ROS_INFO_STREAM(coords.pose.position);
	    p.publish(coords);
	    ros::spinOnce();
	    loopRate.sleep();
	}

	return 0;
}



