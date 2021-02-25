#pragma once
struct ConeSet {
	double mu[2];
	double cor[4];
	int correspond;
	struct ConeSet* next;
};

//typedef struct node;

struct node {
	double St[3];
	int Nt;
	struct ConeSet* next;
};