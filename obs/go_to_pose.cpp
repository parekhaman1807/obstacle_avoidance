#include<geometry_msgs/Point.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<ros/ros.h>
#include<math.h>
// #include<trajectory_msgs/MultiDOFJointTrajectoryPoint>

geometry_msgs::Point p1, p2, vg;

float a = 0.6, b = 0.6, k = 1;

void getHummingbird(const geometry_msgs::Pose& point)
{
	vg.x = 100*(-p1.x + point.position.x);
	vg.y = 100*(-p1.y + point.position.y);
	vg.z = 100*(-p1.z + point.position.z);
	p1 = point.position;
}

void getFirefly(const geometry_msgs::Pose& point)
{
    p2 = point.position;
}

float dot(geometry_msgs::Point diff, geometry_msgs::Point vg){
	return (vg.x * diff.x +  vg.y * diff.y + vg.z * diff.z);
}
int main(int argc,char **argv)
{
  ros::init(argc,argv,"subscribe_position");
  ros::NodeHandle nh;

  ros::Subscriber s1=nh.subscribe("/hummingbird/ground_truth/pose",2,getHummingbird );
  ros::Subscriber s2=nh.subscribe("/firefly/ground_truth/pose",2,getFirefly);
  ros::Publisher p = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

  ros::Rate loopRate(100);

  while(ros::ok())
  {
		geometry_msgs::Point diff;
		diff.x = p2.x - p1.x;
		diff.y = p2.y - p1.y;
		diff.z = p2.z - p1.z;

	    float distancesq =diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
	    float e = pow(distancesq, 0.5);
	    float r;
		// float c = (vg.x * diff.x +  vg.y * diff.y + vg.z * diff.z);

		r = a + b + k * dot(diff, vg)/(pow(distancesq, 0.5));

		float R = pow (pow(diff.x, 2) + pow(diff.y, 2), 0.5);
		float f = sqrt(p1.x*p1.x + p1.y*p1.y + (p1.z-3)*(p1.z-3));
		float V = sqrt(vg.x*vg.x + vg.y*vg.y);
	    // distancesq *= distancesq;

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
	    if(V==0 && f<r)
	    {	
	    	coords.pose.position.x=p1.x-(p1.x*(r+0.3))/sqrt(p1.x*p1.x + p1.y*p1.y);
	    	coords.pose.position.y=p1.y-(p1.y*(r+0.3))/sqrt(p1.x*p1.x + p1.y*p1.y);
	    }
	    else if (e < r)
		{  
			if(vg.x == 0 && vg.y == 0 && f > r){
				coords.pose.position.x=0;
				coords.pose.position.y=0;	
			}
			else{  // ROS_INFO_STREAM("Hello");
				// coords.pose.position.x=p1.x-(diff.y*(r+0.3))/R;
		  //   	coords.pose.position.y=p1.y+(diff.x*(r+0.3))/R;

		    	 if (-diff.y*r*vg.x+diff.x*r*vg.y<0){
		    	 		coords.pose.position.x=p2.x-(vg.y*(r+0.3))/V;
		    	 	    coords.pose.position.y=p2.y+(vg.x*(r+0.3))/V;
		    	 	    ROS_INFO_STREAM(vg.y/vg.x);
		    	 }

		    	 else 
		    		{		coords.pose.position.x=p2.x+(vg.y*(r+0.3))/V;
		    		    	coords.pose.position.y=p2.y-(vg.x*(r+0.3))/V;}

		    		
		    	}// ROS_INFO_STREAM(coords.pose.position);
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
	     
	    // ROS_INFO_STREAM(r);
	    p.publish(coords);
	    ros::spinOnce();
	    loopRate.sleep();
	}

	return 0;
}



