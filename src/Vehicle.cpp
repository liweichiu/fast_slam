#include "MatrixCal.h"
#include "QuadrantAngle.h"
#include <stdio.h>
#include <math.h>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

double del_t = 0.01;
double X[3] = {2,0,PI/2};
double previous_pose[3] = {2,0,PI/2};
double LM_X[30] = { 0 };
double LM_Y[30] = { 0 };
double theta = 0;
double Rt_disgain = 50.0;
double Rt_degree = (double)(0.00436 / 4.0);
int LMc = sizeof(LM_X) / sizeof(double);

void Callback(const geometry_msgs::Twist& msg){
	double v = msg.linear.x;
	double w = msg.angular.z;

	///Vechile model///
	double c1 = 0.05;
	double c2 = 0.005;
	double c3 = 0.05;
	double c4 = 0.005;

	double r[4] = {0,0,0,0};
	r[0] = (double)pow((c1 * fabs(v) + c2 * fabs(w)), 2);
	r[3] = (double)pow((c3 * fabs(v) + c4 * fabs(w)), 2);
	
	double ran[2];
	mvnrnd2(v, w, r, ran);
	double v_actual = ran[0];
	double w_actual = ran[1];

	double change_pose[3];
	change_pose[0] = v_actual * del_t * cos(previous_pose[2] + w_actual * del_t);
	change_pose[1] = v_actual * del_t * sin(previous_pose[2] + w_actual * del_t);
	change_pose[2] = w_actual * del_t;

	for (int i = 0; i < 3; i++) {
		X[i] = previous_pose[i] + change_pose[i];
		previous_pose[i] = X[i];
	}
	Qangle(&X[2]);
	Qangle(&previous_pose[2]);

}



//publish in callback function by class

class SensorPublish
{
	public:
		SensorPublish(){
			cone_pub = m.advertise<std_msgs::Float64MultiArray>("coneXP", 1000);
			GT_pub = m.advertise<geometry_msgs::Twist>("GroundTruth", 1000);
		}
		void coneXP_pub(){
		///Sensor model///
		std_msgs::Float64MultiArray sensorXP;
			for (int i = 0; i < LMc; i++) {
				double q = distance2(LM_X[i], LM_Y[i], X[0], X[1]);
				double phi = (double)atan2((double)LM_Y[i] - (double)X[1], (double)LM_X[i] - (double)X[0]) - X[2];
				Qangle(&phi);
				double Q[4] = { 0 };
				Q[0] = (q / Rt_disgain) * (q / Rt_disgain);
				Q[3] = (Rt_degree * q) * (Rt_degree * q) + 0.0001f;

				if (q < 2 && fabs(rad2deg(phi)) < 50.0) {	
					double ran[2];
					mvnrnd2(q, phi, Q, ran);
					Qangle(&ran[1]);
					sensorXP.data.push_back(ran[0]);
					sensorXP.data.push_back(ran[1]);
				}
			}
		cone_pub.publish(sensorXP);
		}
		void GroundTruth_pub(){
			geometry_msgs::Twist msg;
			msg.linear.x = X[0];
			msg.linear.y = X[1];
			msg.angular.z = X[2];
			GT_pub.publish(msg);
		}
		 
	private:
		ros::NodeHandle m;
		ros::Publisher GT_pub;
		ros::Publisher cone_pub;
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Vehicle");	//node name
	ros::NodeHandle n;
	//ros::Publisher cone_pub = n.advertise<std_msgs::Float64MultiArray>("coneXP", 1000);
	//ros::Publisher GT_pub = n.advertise<geometry_msgs::Twist>("GroundTruth", 1000);
	ros::Subscriber sub = n.subscribe("cmd_vel",1000,Callback);


	for (int i = 0; i < 18; i++) {
		LM_X[i] = 2.5 * cos(theta);
		LM_Y[i] = 2.5 * sin(theta);
		theta += 20.0 / 180.0 * PI;
	}
	theta = 0;
	for (int i = 18; i < 30; i++) {
		LM_X[i] = 1.5 * cos(theta);
		LM_Y[i] = 1.5 * sin(theta);
		theta += 30.0 / 180.0 * PI;
	}
	SensorPublish pub;
	
	ros::Timer coneTimer = n.createTimer(ros::Duration(0.05),std::bind(&SensorPublish::coneXP_pub,pub));
	ros::Timer Timer = n.createTimer(ros::Duration(0.01),std::bind(&SensorPublish::GroundTruth_pub,pub));
			
	ros::spin();

	return 0;
}
//ros::Rate loop_rate(20);
