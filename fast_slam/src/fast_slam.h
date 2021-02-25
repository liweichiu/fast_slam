#pragma once
void fast_test(double* St_par, struct ConeSet** ConeSet_par,int *Nt);
void fast_test2(struct ConeSet* ConeSet_par);
void fast_slam10(double* zt, double* ut, double* St_1, int Nt_1, struct ConeSet* Cone_set, double del_t, double* St_par, int* Nt_par, struct ConeSet** ConeSet_par, double *wt);
void fast_slam20(double* zt, double* ut, double* St_1, int Nt_1, struct ConeSet* Cone_set, double del_t, double* St_par, int* Nt_par, struct ConeSet** ConeSet_par, double* wt);