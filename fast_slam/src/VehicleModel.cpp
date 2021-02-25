#include "VehicleModel.h"
#include <cmath>
#include <stdio.h>
#include "MatrixCal.h"
#include "QuadrantAngle.h"



void VehicleModel(double v, double w, double *previous_pose, double del_t, double *X) {
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
	}
	Qangle(&X[2]);
}

