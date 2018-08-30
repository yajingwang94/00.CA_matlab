#ifndef BSPLINE_H
#define BSPLINE_H


typedef struct point
{
	double x;
	double y;
}Point;


typedef struct 
{
	double p;
	Point nowPonint;
}Posture;


typedef struct 
{
	Posture posture;
	double DoubleS;
}AddDouResult;

#define MAX_P 10
typedef float Matrix[MAX_P+1][MAX_P+1];

// static float   splinelength; //?¨´???¨²???¡è??
// int   IsComLen=1; //??¡¤??¨¨???????¨´???¨²???¡è??
// float  BSplineU[50] = {0}; //B?¨´??????
// float  BSplineP[50] = {0};


void Get_U_P(int n,float q[],float U[],float P[],float t1[2],float t2[2]);
void BSplineViaPoints(float u,int n,float U[],float P[],float path[2],float diff_path[2],float double_diff_path[2]);//u buwei 1
void BSplinePoint(float u, float U[], int n_knot, int p,float P[],int d, float s[]);
int WhichSpan(float u, float U[], int n_knot, int p);
void DersBasisFuns(float u, int i, int p, int n, float U[],Matrix Ders);
void BasisFuns(int i, float u, int p, float U[], float B[]);
void Spline_init(void);
Posture CurveFunSpline(double s);
void BSpline(double s,double pck[]);
#endif // BSPLINE_H
