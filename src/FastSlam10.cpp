#include <stdio.h>
#include "Particle.h"
#include "VehicleModel.h"
#include <math.h>
#include "MatrixCal.h"
#include "QuadrantAngle.h"
#include "SensorModel.h"
#include "fast_slam.h"
#include "resample_weight.h"
#include <random>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>

const int landmark_number = 30;
double v_actual = 0.0f;						// velocity cmd, unit: (m/s)
double w_actual = 0.0f;						// omega cmd, unit: (rad/s)
int data_size;
double sensor_dataX[landmark_number];		// sensor data distance
double sensor_dataP[landmark_number];		// sensor data head angle
bool zt_in = false;

void CommandCallback(const geometry_msgs::Twist& msg){
	v_actual = msg.linear.x;
	w_actual = msg.angular.z;
	
}
void coneCallback(const std_msgs::Float64MultiArray& msg){
	data_size = msg.data.size()/2;
	for(int i=0;i<msg.data.size()/2;i++){
		sensor_dataX[i] = msg.data[2*i];
		sensor_dataP[i] = msg.data[2*i+1];
	}
	zt_in = true;
	
}


int main(int argc, char **argv)
{
	//////////file open////////////
	FILE* fp_XYT;
	fp_XYT = fopen64("/home/nvidia/Desktop/XYT_Data.txt", "w");
	FILE* fp_St;
	fp_St = fopen64("/home/nvidia/Desktop/St_Data.txt", "w");
	FILE* fp_Nt;
	fp_Nt = fopen64("/home/nvidia/Desktop/Nt_Data.txt", "w");
	FILE* fp_mu;
	fp_mu = fopen64("/home/nvidia/Desktop/Cone_mu_Data.txt", "w");
	FILE* fp_cor;
	fp_cor = fopen64("/home/nvidia/Desktop/Cone_cor_Data.txt", "w");
	FILE* fp_target;
	fp_target = fopen64("/home/nvidia/Desktop/Cone_tar_Data.txt", "w");

	
	/////////initial settings////////////////
	// Parameter Init
	const int M = 30;			// Particle number
	double del_t = 0.05;			// unit : (s)
	double Init_Size = 0.0f;		// the init range of vehicle X Y, unit: (m)
	double Init_way = 0;			// the init range of vehicle Theta, unit: (degree)
	double Init_position[2] = { 2,0 };			// unit: (m)
	double Init_head_angle = PI / 2;			// unit: (rad)
	//double simu_time = 10.0;						// unit: (second)
	double XYT[3];				// real vehicle state
	XYT[0] = Init_position[0];
	XYT[1] = Init_position[1];
	XYT[2] = Init_head_angle;
	std::default_random_engine normal_seed;
	std::normal_distribution<double> gaussian(0.0, 1.0);
	// Init the vehicle state of the particle
	struct node Particle[M];
	for (int j = 0; j < M; j++) {
		Particle[j].St[0] = Init_position[0] + Init_Size * gaussian(normal_seed);
		Particle[j].St[1] = Init_position[1] + Init_Size * gaussian(normal_seed);
		Particle[j].St[2] = Init_head_angle + Init_way * PI / 180 * gaussian(normal_seed) - Init_way / 2.0;
		Particle[j].Nt = 0;
	}
	// Init the vehicle state of the particle
	int sensor_check[landmark_number] = { 0 };	// sensor data check
	double zt[2];
	double ut[2];
	double wt[M];
	////////////////////////////////////////////
	

	//////////ros settings////////////////
	ros::init(argc, argv, "FastSlam10");	//node name
	
	ros::NodeHandle n;
	ros::Publisher 	pub = n.advertise<std_msgs::Float64MultiArray>("status", 1000);
 	ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 1000, CommandCallback);	//topic name and callback function
	ros::Subscriber sensor_sub = n.subscribe("coneXP", 1000, coneCallback);
	
	ros::Rate loop_rate(100);

	ROS_INFO("Initialize Finished!");

	double begin = ros::Time::now().toSec();	

 	while (ros::ok()){
		//ground truth
		double XYT_t_1[3];
		for (int i = 0; i < 3; i++) {
			XYT_t_1[i] = XYT[i];
		}
		VehicleModel(v_actual, w_actual, XYT_t_1, del_t/5, XYT);

		///if no new sensor data -> SLAM
		if(zt_in == false){
			double St_1[3];
			for (int m = 0; m < M; m++) {
				for (int i = 0; i < 3; i++) {
					St_1[i] = Particle[m].St[i];
				}
				VehicleModel(v_actual, w_actual, St_1, del_t/5, Particle[m].St);
			}
		}

		int time_stop = 1;	// to handle sensor many cone in the same time
		///if have new sensor data -> SLAM
		if(zt_in == true){
			for (int jk = 0; jk < data_size; jk++) {
				zt[0] = sensor_dataX[jk];
				zt[1] = sensor_dataP[jk];
				for (int m = 0; m < M; m++) {
					double St_1[3];
					int Nt_1 = 0;
					struct ConeSet* Cone_set = NULL;

					for (int i = 0; i < 3; i++) {
						St_1[i] = Particle[m].St[i];
					}
					Nt_1 = Particle[m].Nt;
					Cone_set = Particle[m].next;
					if (time_stop == 1) {
						ut[0] = v_actual;
						ut[1] = w_actual;
					}
					else {
						ut[0] = 0;
						ut[1] = 0;
					}
					fast_slam10(zt, ut, St_1, Nt_1, Cone_set, del_t/5, Particle[m].St, &Particle[m].Nt, &Particle[m].next, &wt[m]);
				}
				time_stop = 0;

				//////// resample
				int re_sample_index[M];
				resample_weight(wt, re_sample_index, M);
				struct node up_Particle[M];
				for (int i = 0; i < M; i++) {
					up_Particle[i] = Particle[i];
				}
				for (int i = 0; i < M; i++) {
					Particle[i] = up_Particle[re_sample_index[i]];
				}
				//////// resample
			}
			
				zt_in = false;
		}
	

		struct ConeSet* fpt_cone;
		
		std_msgs::Float64MultiArray status;
		
		status.data.push_back((double)Particle[0].St[0]);
		status.data.push_back((double)Particle[0].St[1]);
		status.data.push_back((double)Particle[0].St[2]);
		
		
		fprintf(fp_XYT, "%f\t", XYT[0]);
		fprintf(fp_XYT, "%f\t", XYT[1]);
		fprintf(fp_XYT, "%f\n", XYT[2]);

		fprintf(fp_St, "%f\t", Particle[0].St[0]);
		fprintf(fp_St, "%f\t", Particle[0].St[1]);
		fprintf(fp_St, "%f\n", Particle[0].St[2]);
		fprintf(fp_Nt, "%d\n", Particle[0].Nt);

		fpt_cone = Particle[0].next;
		for (int i = 0; i < Particle[0].Nt; i++) {
			if (fpt_cone) {
				fprintf(fp_mu, "%f\t", fpt_cone->mu[0]);
				fprintf(fp_mu, "%lf\n", fpt_cone->mu[1]);
				fprintf(fp_cor, "%f\t", fpt_cone->cor[0]);
				fprintf(fp_cor, "%f\t", fpt_cone->cor[1]);
				fprintf(fp_cor, "%f\t", fpt_cone->cor[2]);
				fprintf(fp_cor, "%f\n", fpt_cone->cor[3]);
				fprintf(fp_target, "%d\n", fpt_cone->correspond);
				
				status.data.push_back((double)fpt_cone->mu[0]);
				status.data.push_back((double)fpt_cone->mu[1]);
				if (fpt_cone->cor[0] < 1e-5 && fpt_cone->cor[3] < 1e-5){
					status.data.push_back((double)fpt_cone->cor[0]);
					status.data.push_back(0);
					status.data.push_back(0);
					status.data.push_back((double)fpt_cone->cor[3]);
				}
				else{
					status.data.push_back((double)fpt_cone->cor[0]);
					status.data.push_back((double)fpt_cone->cor[1]);
					status.data.push_back((double)fpt_cone->cor[2]);
					status.data.push_back((double)fpt_cone->cor[3]);
				}
				fpt_cone = fpt_cone->next;
				
				
			}
			else {
				printf("txt error\n");
			}

		}

		pub.publish(status);

    	ros::spinOnce();
    	loop_rate.sleep();

		//double now = ros::Time::now().toSec();
		//ROS_INFO("time : %f s",now-begin);
			
  	}

	fclose(fp_XYT);
	fclose(fp_St);
	fclose(fp_Nt);
	fclose(fp_mu);
	fclose(fp_cor);
	fclose(fp_target);

	printf("txt finish\n");

	return 0;
}





/*
		int time_stop = 1;	// to handle sensor many cone in the same time
		int nodata = 0;		// to handle no sensor data
		
		for (int jk = 0; jk < landmark_number; jk++) {

			if (jk<data_size) { //jk<data_size //sensor_check[jk] == 1
				zt[0] = sensor_dataX[jk];
				zt[1] = sensor_dataP[jk];
				for (int m = 0; m < M; m++) {
					double St_1[3];
					int Nt_1 = 0;
					struct ConeSet* Cone_set = NULL;

					for (int i = 0; i < 3; i++) {
						St_1[i] = Particle[m].St[i];
					}
					Nt_1 = Particle[m].Nt;
					Cone_set = Particle[m].next;
					if (time_stop == 1) {
						ut[0] = v_actual;
						ut[1] = w_actual;
					}
					else {
						ut[0] = 0;
						ut[1] = 0;
					}
					fast_slam10(zt, ut, St_1, Nt_1, Cone_set, del_t, Particle[m].St, &Particle[m].Nt, &Particle[m].next, &wt[m]);
				}
				time_stop = 0;

				//////// resample
				int re_sample_index[M];
				resample_weight(wt, re_sample_index, M);
				struct node up_Particle[M];
				for (int i = 0; i < M; i++) {
					up_Particle[i] = Particle[i];
				}
				for (int i = 0; i < M; i++) {
					Particle[i] = up_Particle[re_sample_index[i]];
				}
				//////// resample
			}
			else nodata++;
		}
		if (nodata == landmark_number) {
			double St_1[3];
			for (int m = 0; m < M; m++) {
				for (int i = 0; i < 3; i++) {
					St_1[i] = Particle[m].St[i];
				}
				VehicleModel(v_actual, w_actual, St_1, del_t/5, Particle[m].St);
			}
		}
*/
