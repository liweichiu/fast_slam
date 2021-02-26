#include "SensorModel.h"
#include "MatrixCal.h"
#include "QuadrantAngle.h"
#include <stdio.h>
#include <math.h>



void SensorModel(double* X, double* SensorData_finalX, double* SensorData_finalP, int* SensorCheckfinal) {
	const int ld_num = 10;
	//double LM_X[4] = { 45,25,20,50 };
	//double LM_Y[4] = { 35,50,60,25 };
	//double LM_X[15] = { 45,25,20,50,10,20,30,40,50,60,70,80,90,100,110 };
	//double LM_Y[15] = { 35,50,60,25,10,20,30,40,50,60,70,80,90,100,110 };
	double LM_X[30] = { 0 };
	double LM_Y[30] = { 0 };
	double theta = 0;
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
	double Rt_disgain = 50.0;
	double Rt_degree = (double)(0.00436 / 4.0);
	//double Rt_disgain = 2000;
	//double Rt_degree = 0;

	int LMc = sizeof(LM_X) / sizeof(double);
	double SensorDataX[sizeof(LM_X) / sizeof(double)] = { 0 };
	double SensorDataP[sizeof(LM_X) / sizeof(double)] = { 0 };
	int SensorCheck[sizeof(LM_X) / sizeof(double)] = { 0 };

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
		SensorCheck[i] = 1;
		SensorDataX[i] = ran[0];
		SensorDataP[i] = ran[1];
		}
		SensorData_finalX[i] = SensorDataX[i];
		SensorData_finalP[i] = SensorDataP[i];
		SensorCheckfinal[i] = SensorCheck[i];
	}

}
