#include "fast_slam.h"
#include <cmath>
#include "QuadrantAngle.h"
#include "VehicleModel.h"
#include "Particle.h"
#include <stdio.h>
#include "MatrixCal.h"
void fast_test2(struct ConeSet* ConeSet_par) {
	ConeSet_par = ConeSet_par->next;
	printf("2mu=%f\t", ConeSet_par->mu[0]);
	printf("2mu=%f\n", ConeSet_par->mu[1]);
}
void fast_test(double* St_par, struct ConeSet** ConeSet_par, int* Nt) {
	struct ConeSet* cur = NULL;
	struct ConeSet* st = NULL;
	for (int i = 0; i < 5; i++) {
		struct ConeSet* fir = NULL;
		fir = (struct ConeSet*)malloc(sizeof(struct ConeSet));
		if (i == 0) {
			fir->mu[0] = 9.81;
			st = fir;
			cur = fir;
		}
		else if (i == 1) {
			fir->mu[0] = 1.349;
			st->next = fir;
			st = st->next;
		}
		else {
			if (i == 2) {
				fir->mu[0] = 2.685;
				st->next = fir;
				st = st->next;
			}
			else {
				fir->mu[0] = 4.558;
				st->next = fir;
				st = st->next;
			}

		}
	}
	*ConeSet_par = cur;
	*Nt = 3;
}

void fast_slam10(double* zt, double* ut, double* St_1, int Nt_1, struct ConeSet* Cone_set, double del_t, double* St_par, int* Nt_par, struct ConeSet** ConeSet_par, double* wt) {
	//////// parameter 

	// sensor model parameter
	double Rt_disgain = 50.0;
	double Rt_degree = (0.00436 / 4.0);
	// control input noise
	double c1 = 0.05;
	double c2 = 0.005;
	double c3 = 0.05;
	double c4 = 0.005;
	// the parameter of the cone is the same or different
	double observer_distance = 0.0;
	double observer_angle = 0.0873 * 15.0 / 5.0;
	// new cone pdf
	double p0 = 0.005;

	//////// parameter 
	double v_actual = ut[0];
	double w_actual = ut[1];

	//// St = h(St_1,ut)
	double St[3] = { 0 };	// vehicle state
	VehicleModel(v_actual, w_actual, St_1, del_t, St);

	double max_wn = 0.0;
	int n_hat = -1;
	struct ConeSet* first = NULL;
	struct ConeSet* current = NULL;
	struct ConeSet* predict_Cone_set = Cone_set;
	struct ConeSet* update_Cone_set = Cone_set;
	
	////// Particle filter predict
	for (int n = 0; n < Nt_1; n++) {
		if (n != 0) {
			if (predict_Cone_set) predict_Cone_set = predict_Cone_set->next;	// pointer point to next cone data
			else printf("Cone_set error\n");
		}
		double cone_mu[2] = { 0 };												// cone X Y coordinate
		double cone_cor[4] = { 0 };												// cone covariance
		if (predict_Cone_set) {
			cone_mu[0] = predict_Cone_set->mu[0];
			cone_mu[1] = predict_Cone_set->mu[1];
			for (int i = 0; i < 4; i++) {
				cone_cor[i] = predict_Cone_set->cor[i];
			}
		}
		double q = distance2(cone_mu[0], cone_mu[1], St[0], St[1]);				// Sensor model predict distance
		double zn_hat[2];														// Sensor model predict Phi
		zn_hat[0] = q;
		zn_hat[1] = atan2(cone_mu[1] - St[1], cone_mu[0] - St[0]) - St[2];
		Qangle(&zn_hat[1]);
		double G[4];															// Sensor model Jacobian 
		G[0] = (cone_mu[0] - St[0]) / q;
		G[1] = (cone_mu[1] - St[1]) / q;
		G[2] = -(cone_mu[1] - St[1]) / (q * q);
		G[3] = (cone_mu[0] - St[0]) / (q * q);
		double Rt[4] = { 0 };													// Sensor model noise covariance
		Rt[0] = (q / Rt_disgain) * (q / Rt_disgain);
		Rt[3] = (Rt_degree * q) * (Rt_degree * q) + 0.0001;

		double Gcor[4];
		mattimes(G, 2, 2, cone_cor, 2, 2, Gcor);
		double GT[4];
		transpose(G, 2, 2, GT);
		double GcorGT[4];
		mattimes(Gcor, 2, 2, GT, 2, 2, GcorGT);
		double Q[4];															// Error propagation covariance : mean Sensor model predict cone covariance
		matplus(GcorGT, 2, 2, Rt, Q);
		// calculate resample weight  
		double err[2];
		err[0] = zt[0] - zn_hat[0];
		err[1] = zt[1] - zn_hat[1];
		Qangle(&err[1]);

		double invQ[4];
		inv2(Q, invQ);
		double errTinvQ[2];
		mattimes(err, 1, 2, invQ, 2, 2, errTinvQ);
		double errTinvQerr[1];
		mattimes(errTinvQ, 1, 2, err, 2, 1, errTinvQerr);

		double detQ = Q[0] * Q[3] - Q[1] * Q[2];
		double num_wn = sqrt(2.0 * PI * detQ);
		double den_wn = exp(-0.5 * errTinvQerr[0]);
		double wn = den_wn / num_wn;
		if (wn >= max_wn) {
			max_wn = wn;
			n_hat = n;
			//printf("n=%d\n", n_hat);
		}
	}
	int Nt = max_wn > p0 ? Nt_1 : Nt_1 + 1;
	n_hat = max_wn < p0 ? Nt_1 : n_hat;
	max_wn = max_wn > p0 ? max_wn : p0;
	int Nt_update = Nt;
	double mu_t[2];
	double cor_t[4];
	int target_i = 0;
	// case1: new cone , case2: EKF update , case3: if cone is fake ,delete
	for (int n = 0; n < Nt; n++) {
		if (n != 0) {
			if (update_Cone_set) update_Cone_set = update_Cone_set->next;
			else printf("Cone_set error\n");
		}
		if (n == Nt_1) {
			mu_t[0] = St[0] + zt[0] * cos(zt[1] + St[2]);
			mu_t[1] = St[1] + zt[0] * sin(zt[1] + St[2]);
			double q = distance2(mu_t[0], mu_t[1], St[0], St[1]);
			double G[4];
			G[0] = (mu_t[0] - St[0]) / q;
			G[1] = (mu_t[1] - St[1]) / q;
			G[2] = -(mu_t[1] - St[1]) / (q * q);
			G[3] = (mu_t[0] - St[0]) / (q * q);
			double invG[4];
			inv2(G, invG);
			double Rt[4] = { 0 };
			Rt[0] = (q / Rt_disgain) * (q / Rt_disgain);
			Rt[3] = (Rt_degree * q) * (Rt_degree * q) + 0.0001;
			double invGRt[4];
			mattimes(invG, 2, 2, Rt, 2, 2, invGRt);
			double invGT[4];
			transpose(invG, 2, 2, invGT);
			mattimes(invGRt, 2, 2, invGT, 2, 2, cor_t);
			target_i = 1;
		}
		else if (n == n_hat) {
			double mu_t_1[2] = { 0 };
			double cor_t_1[4] = { 0 };
			int target_i_t_1 = 0;

			if (update_Cone_set) {
				mu_t_1[0] = update_Cone_set->mu[0];
				mu_t_1[1] = update_Cone_set->mu[1];
				for (int i = 0; i < 4; i++) {
					cor_t_1[i] = update_Cone_set->cor[i];
				}
				target_i_t_1 = update_Cone_set->correspond;
			}
			else printf("update cone_Set error\n");
			double q = distance2(mu_t_1[0], mu_t_1[1], St[0], St[1]);
			double G[4];
			G[0] = (mu_t_1[0] - St[0]) / q;
			G[1] = (mu_t_1[1] - St[1]) / q;
			G[2] = -(mu_t_1[1] - St[1]) / (q * q);
			G[3] = (mu_t_1[0] - St[0]) / (q * q);
			double Rt[4] = { 0 };
			Rt[0] = (q / Rt_disgain) * (q / Rt_disgain);
			Rt[3] = (Rt_degree * q) * (Rt_degree * q) + 0.0001;

			double Gcor[4];
			mattimes(G, 2, 2, cor_t_1, 2, 2, Gcor);
			double GT[4];
			transpose(G, 2, 2, GT);
			double GcorGT[4];
			mattimes(Gcor, 2, 2, GT, 2, 2, GcorGT);
			double Q[4];
			matplus(GcorGT, 2, 2, Rt, Q);
			double invQ[4];
			inv2(Q, invQ);
			double corGT[4];
			mattimes(cor_t_1, 2, 2, GT, 2, 2, corGT);
			double K[4];
			mattimes(corGT, 2, 2, invQ, 2, 2, K);

			double zn_hat[2];
			zn_hat[0] = q;
			zn_hat[1] = atan2(mu_t_1[1] - St[1], mu_t_1[0] - St[0]) - St[2];
			Qangle(&zn_hat[1]);
			double err[2];
			err[0] = zt[0] - zn_hat[0];
			err[1] = zt[1] - zn_hat[1];
			Qangle(&err[1]);
			double Kerr[2];
			mattimes(K, 2, 2, err, 2, 1, Kerr);
			mu_t[0] = mu_t_1[0] + Kerr[0];
			mu_t[1] = mu_t_1[1] + Kerr[1];
			double KG[4];
			mattimes(K, 2, 2, G, 2, 2, KG);
			double I2minusKG[4];
			I2minusKG[0] = 1 - KG[0];
			I2minusKG[1] = -KG[1];
			I2minusKG[2] = -KG[2];
			I2minusKG[3] = 1 - KG[3];
			mattimes(I2minusKG, 2, 2, cor_t_1, 2, 2, cor_t);
			target_i = target_i_t_1 + 1;
		}
		else {
			double mu_t_1[2] = { 0 };
			double cor_t_1[4] = { 0 };
			int target_i_t_1 = 0;


			mu_t_1[0] = update_Cone_set->mu[0];
			mu_t_1[1] = update_Cone_set->mu[1];
			for (int i = 0; i < 4; i++) {
				cor_t_1[i] = update_Cone_set->cor[i];
			}
			target_i_t_1 = update_Cone_set->correspond;


			mu_t[0] = mu_t_1[0];
			mu_t[1] = mu_t_1[1];
			for (int i = 0; i < 4; i++) {
				cor_t[i] = cor_t_1[i];
			}
			target_i = target_i_t_1; 
			
			double cone_dis = distance2(mu_t_1[0], mu_t_1[1], St[0], St[1]);
			double cone_angle = atan2(mu_t_1[1] - St[1], mu_t_1[0] - St[0]) - St[2];
			Qangle(&cone_angle);
			if (fabs(cone_dis - zt[0]) > observer_distance || fabs(cone_angle - zt[1]) > observer_angle)
				target_i = target_i_t_1;
			else
				target_i = target_i_t_1 - 1;
		}

		struct ConeSet* cone_data;
		cone_data = (struct ConeSet*)malloc(sizeof(struct ConeSet));
		if (cone_data) {
			cone_data->mu[0] = mu_t[0];
			cone_data->mu[1] = mu_t[1];
			for (int i = 0; i < 4; i++) {
				cone_data->cor[i] = cor_t[i];
			}
			cone_data->correspond = target_i;
		}
		else printf("cone_data malloc error\r\n");

		if (first == NULL) {
			if (target_i >= 0) {
				first = cone_data;
				if (cone_data) current = cone_data;
				else printf("cone_data malloc error\n");
			}
			else Nt_update = Nt_update - 1;
		}
		else {
			if (current) {
				if (target_i >= 0) {					// if cone is not fake
					current->next = cone_data;
					current = current->next;
				}
				else Nt_update = Nt_update - 1;
			}
			else printf("current error");
		}
	}

	for (int i = 0; i < 3; i++) {
		St_par[i] = St[i];
	}
	*Nt_par = Nt_update;
	*ConeSet_par = first;
	*wt = max_wn;
}