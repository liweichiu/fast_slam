#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"


int main(int argc, char **argv)
{

	ros::init(argc, argv, "CMD_Pub");	//node name
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(100);

	while (ros::ok()){
		geometry_msgs::Twist cmd;
		cmd.linear.x = 1.0f;
		cmd.angular.z = 0.5f;
		pub.publish(cmd);
		ROS_INFO("publish cmd vel");				

		ros::spinOnce();
    	loop_rate.sleep();
	}

	return 0;
}
