   　　　　　　　　　　　　　　　　　　　　　　　　　　   　　　　　　　　　　　　　　　　　　　　　　　　   　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　　#include "mex.h"
#include "BSpline.h"
#include <math.h>


//void BSplinePoint( float u, float U[], int n_knot, int p,float P[],int d, float s[]);

void Get_U_P(int n,float q[],float U[],float P[],float t1[2],float t2[2])
{
	float _U[50]={0};
	float B[MAX_P]={0};
    int i,k;
	double dd=0;

	float a[50]={0},b[50]={0},c[50]={0},x0[50]={0},x1[50]={0},d0[50]={0},d1[50]={0};
	float m=0;
	int n1;
		
	n=n-1;
	for(i=1;i<=n;i++)
	{
		dd=dd+sqrtl(powl((q[i+n+1]-q[i+n]),2)+powl((q[i]-q[i-1]),2));
	}

	for(i=1;i<=n-1;i++)
	{
		_U[i]=_U[i-1]+sqrtl(powl((q[i+n+1]-q[i+n]),2)+powl((q[i]-q[i-1]),2))/dd;
	}

	_U[0]=0;_U[n]=1;
	U[0]=_U[0];U[1]=_U[0];U[2]=_U[0];U[n+4]=_U[n];U[n+5]=_U[n];U[n+6]=_U[n];
	for(i=0;i<=n;i++)
	{
		U[i+3]=_U[i];
	}

	t1[0]=(q[1]-q[0])/(_U[1]-_U[0]);
    t1[1]=(q[n+2]-q[n+1])/(_U[1]-_U[0]);
	t2[0]=(q[n]-q[n-1])/(_U[n]-_U[n-1]);
    t2[1]=(q[2*n+1]-q[2*n])/(_U[n]-_U[n-1]);

	P[0]=q[0];
	P[n+3]=q[n+1];
	P[1]=q[0]+U[4]/3*t1[0];
	P[n+4]=q[n+1]+U[4]/3*t1[1];
	P[n+1]=q[n]-(1-U[n+3])/3*t2[0];
	P[2*n+4]=q[2*n+1]-(1-U[n+3])/3*t2[1];
	P[n+2]=q[n];
	P[2*n+5]=q[2*n+1];


	n1=n-1;
	for(k=2;k<=n1;k++)
	{
		BasisFuns(WhichSpan(_U[k],U, n+6, 3),_U[k],3,U,B);
		a[k]=B[0];
		b[k]=B[1];
		BasisFuns(WhichSpan(_U[k-1],U, n+6, 3),_U[k-1],3,U,B);
		c[k-1]=B[2];
	}
	BasisFuns(WhichSpan(_U[1],U, n+6, 3),_U[1],3,U,B);
	b[1]=B[1];

	for(k=1;k<=n1;k++)
	{
		d0[k]=q[k];
		d1[k]=q[n+k+1];
	}

	BasisFuns(WhichSpan(_U[1],U, n+6, 3),_U[1],3,U,B);
	d0[1]=q[1]-B[0]*P[1];
	d1[1]=q[n+2]-B[0]*P[n+4];

	BasisFuns(WhichSpan(_U[n-1],U, n+6, 3),_U[n-1],3,U,B);
	d0[n1]=q[n-1]-B[2]*P[n+1];
	d1[n1]=q[2*n]-B[2]*P[2*n+4];

	for(k=2;k<=n1;k++)
	{
		m=a[k]/b[k-1];
		b[k]=b[k]-m*c[k-1];
		d0[k]=d0[k]-m*d0[k-1];
		d1[k]=d1[k]-m*d1[k-1];
	}

	x0[n1]=d0[n1]/b[n1];
	x1[n1]=d1[n1]/b[n1];
	for(k=n1-1;k>=1;k--)
	{
		x0[k]=(d0[k]-c[k]*x0[k+1])/b[k];
		x1[k]=(d1[k]-c[k]*x1[k+1])/b[k];
	}

	for(k=2;k<=n;k++)
	{
		P[k]=x0[k-1];
		P[k+n+3]=x1[k-1];
	}
}

void BSplineViaPoints(float u,int n,float U[],float P[],float path[2],float diff_path[2],float float_diff_path[2])//u buwei 1
{
    float s[2]={0,0},diff_s[2]={0,0},float_diff_s[2]={0,0};
    int i,k,j;
	Matrix Ders;

	n=n-1;
	BSplinePoint(u,U,n+6,3,P,2,s);
	path[0]=s[0];
	path[1]=s[1];

	i= WhichSpan(u,U,n+6,3);
	DersBasisFuns(u,i,3,2,U,Ders);
	for (k=0;k<2;k++) /* For each components of the B-spline*/
	{
		diff_s[k] = 0;
		float_diff_s[k]=0;
		for (j = 0; j<=3; j++)
		{
			if(k==0)
			{
				diff_s[k] = diff_s[k] + P[k*5 + i-3+j]*Ders[1][j];//B[j];
			float_diff_s[k] = float_diff_s[k] + P[k*5 + i-3+j]*Ders[2][j];//B[j];
             //   diff_s[k] = diff_s[k] + P[j]*Ders[1][j+3-i];//B[j];
			//		float_diff_s[k] = float_diff_s[k] + P[j]*Ders[2][j+3-i];//B[j];
			}
			if(k==1)
			{
			  diff_s[k] = diff_s[k] + P[k*(n+3)+i-3+j]*Ders[1][j];//B[j];
	     		float_diff_s[k] = float_diff_s[k] + P[k*(n+3)+i-3+j]*Ders[2][j];//B[j];
             //   diff_s[k] = diff_s[k] + P[n+3+j]*Ders[1][j+3-i];//B[j];
		     // float_diff_s[k] = float_diff_s[k] + P[n+3+j]*Ders[2][j+3-i];//B[j];
			}
		}
	}
	diff_path[0]=diff_s[0];
	diff_path[1]=diff_s[1];
	float_diff_path[0]=float_diff_s[0];
	float_diff_path[1]=float_diff_s[1];
}

void BasisFuns(int i, float u, int p, float U[], float B[])
	/*
	Input: i - knot span including u
	u - value of the independent variable
	p - degree of the spline
	U[] - Knot vector
	Output: B[] - value of the nonvanishing basis function at u
	*/
{
	int j,r;
	float temp, acc;
	float DR[MAX_P], DL[MAX_P];
	B[0]=1;
	for (j=1; j<=p;j++)
	{
		DL[j] = u - U[i+1-j];
		DR[j] = U[i+j] - u;
		acc = 0.0;
		for (r=0; r<= j-1;r++)
		{
			temp = B[r]/(DR[r+1] + DL[j-r]);
			B[r] = acc + DR[r+1]*temp;
			acc =DL[j-r]*temp;
		}
		B[j] = acc;
	}
}

int WhichSpan(float u, float U[], int n_knot, int p)
	/*
	Input: u - value of the independent variable
	U[] - Knot vector
	n_knot - length of U[] -1
	p - degree of the spline
	Output: mid - index of the knot span including u
	*/
{
	int high, low, mid;
	high = n_knot - p;
	low = p;
	if (u == U[high])
		mid = high;
	else
	{
		mid = (high+low)/2;
		while ((u<U[mid])||(u>=U[mid+1]))
		{
			if (u==U[mid+1])
				mid = mid+1; /* knot with multiplicity >1 */
			else
			{
				if (u > U[mid])
					low = mid;
				else
					high=mid;
				mid = (high+low)/2;
			}
		}
	}
	return mid;
}

void BSplinePoint( float u, float U[], int n_knot, int p,float P[],int d, float s[])
	/*
	Inputs:u - value of the independent variable
	U[] - Knot vector
	n_knot - length of U[] -1
	p - degree of the spline
	P[] - Control point vector
	d - dimensions of a control point (2 in 2D, 3 in 3D, etc.)
	Output:s[] - value of the B-spline at u
	*/
{
	float B[MAX_P];
	int i, k, j;
	i= WhichSpan(u, U, n_knot, p);
	BasisFuns(i, u, p, U, B);

	for (k = 0; k<d; k++) /* For each components of the B-spline*/
	{
		s[k] = 0;
		for (j = 0; j<=p; j++)
		{
			s[k] = s[k] + P[k*(n_knot-p) + i-p+j]*B[j];
		}
	}
}

void DersBasisFuns(float u, int i, int p, int n, float U[],Matrix Ders)
{
	/*
	Inputs: u - value of the independent variable
	j - index of the knot span, which includes u
	p - degree of the spline
	n - max degree of differentiation of B-spline
	basis functions
	U[] - Knot vector
	Output: Ders[][] - values of B-spline basis functions and theirs
	derivatives at u
	*/
	float DR[MAX_P], DL[MAX_P];
	Matrix Du, a;
	float acc, temp, d;
	int j, r, k, s1, s2, rk, pk, j1, j2;
	Du[0][0] = 1.0;
	for (j=1; j<=p; j++)
	{
		DL[j] = u - U[i+1-j];
		DR[j] = U[i+j]-u;
		acc = 0.0;
		for (r=0;r<j;r++)
		{
			Du[j][r] = DR[r+1] + DL[j-r];
			temp = Du[r][j-1] / Du[j][r];
			Du[r][j] = acc + DR[r+1] * temp;
			acc = DL[j-r] * temp;
		}
		Du[j][j] = acc;
	}
	for (j=0; j<=p; j++)
		Ders[0][j] = Du[j][p];
	for (r=0; r<=p; r++)
	{
		s1=0;
		s2=1;
		a[0][0] = 1.0;
		for (k=1; k<=n; k++)
		{
			d = 0.0;
			rk=r-k;
			pk=p-k;
			if (r >= k)
			{
				a[s2][0] = a[s1][0] / Du[pk+1][rk];
				d = a[s2][0] * Du[rk][pk];
			}
			if (rk >= -1)
				j1=1;
			else
				j1 = -rk;
			if (r-1 <= pk)
				j2=k-1;
			else
				j2=p-r;
			for (j=j1; j<=j2; j++)
			{
				a[s2][j] = (a[s1][j] - a[s1][j-1]) / Du[pk+1][rk+j];
				d += a[s2][j] * Du[rk+j][pk];
			}
			if (r <= pk)
			{
				a[s2][k] = -a[s1][k-1] / Du[pk+1][r];
				d += a[s2][k] * Du[r][pk];
			}
			Ders[k][r] = d;
			j = s1; s1 = s2; s2 = j;
		}
	}
	r=p;
	for (k=1; k<=n; k++)
	{
		for (j=0; j<=p; j++) Ders[k][j] *= r;
		r *= (p-k);
	}
}

// void Spline_init(void)
// {
// 	int i;
//     float q[50*2]={0};
//     
// 	float ps[10]={0,-66.7172, -400.3066, -589.3380, -522.6165,
//         0,  548.7672,  597.7696,  391.9785,  -19.5988};
// 
//  
// 	int n=5;
// 	float t1[2]={0,0},t2[2]={0,0};
// 	float s[2]={0,0};
//     float xlast=0,ylast=0;
// //	double diff_x=0;
// 	int isample;
// 
//    for(i=0;i<10;i++)
//    {	
//        q[i]=ps[i];
//    }
// 	splinelength=0;    
// 	Get_U_P(n,q,BSplineU,BSplineP,t1,t2);//
// 	for(isample=0;isample<=100;isample++)
// 	{	
//           BSplinePoint(isample*0.01,BSplineU,n-1+6,3,BSplineP,2,s);
//          if(isample>0)
//         {
//         //		splinelength=splinelength+sqrt((s[0]-xlast)*(s[0]-xlast)+(s[1]-ylast)*(s[1]-ylast));
//                 splinelength=splinelength+s[0];
//         }
//          xlast=s[0];
//          ylast=s[1];
// 	}
// 		
// 	IsComLen=0;
//  //    splinelength=4000; 
// }


// Posture CurveFunSpline(float s)
// {
// 	Posture	re;
// 	float xprim,yprim;
// 	float usample;
// //  const int n=gWFtable.wfnum[0];
// 	//double t1[2]={0,0},t2[2]={0,0};
// 	float path[2]={0,0},diff_path[2]={0,0},double_diff_path[2]={0,0};
// 
// 	if(IsComLen==1)
// 	{
// 		Spline_init();
//     }
// 
// 	if(splinelength!=0)
// 	{
// 		usample=s/splinelength;
// 	}
// 	else
// 	{
// 		usample=0;
// 	}
// 	
// 	if(usample>=1||usample<0)
// 	{
// 		usample=0;
// 	}
// 	
// 	BSplineViaPoints(usample,5,BSplineU,BSplineP,path,diff_path,double_diff_path);//???u???????????
// 	re.nowPonint.x=path[0];
// 	re.nowPonint.y=path[1];
// //	xprim=diff_path[0]/splinelength;
// //	yprim=diff_path[1]/splinelength;
// 	
// 	xprim=diff_path[0];
// 	yprim=diff_path[1];
// 	
// //	re.nowPonint.x=0;
// //	re.nowPonint.y=0;
// //	xprim=1;
// //	yprim=1;
//   re.p=atan2(yprim,xprim);
// 	
// 	return re;
// }

void BSpline(double s,double pck[])
{
	AddDouResult rekfs;
	double xprim,yprim,xpprim,ypprim;
	float usample;
	float path[2]={0,0},diff_path[2]={0,0},double_diff_path[2]={0,0};

  float  splinelength=2508.618390858;
//   float  splinelength=1988.137936626109;
   float BSplineU[11]={0, 0, 0, 0, 0.2536775, 0.4842969, 0.7088977,  1, 1, 1, 1};
   float BSplineP[14]={0, 44.47752, 288.4869, -243.5291, -852.8672, -522.6165, -522.6165, 0, 155.4366, 503.6388,  725.7311, 614.3641, -22.20579, -22.20579};
// 	if(IsComLen==1)
// 	{
// 		Spline_init();
//     }

	if(splinelength!=0)
	{
      //   splinelength=3000;
		usample=s/splinelength;
        if(usample>=1||usample<=0)
        {
            usample=0;
        }

        BSplineViaPoints(usample,5,BSplineU,BSplineP,path,diff_path,double_diff_path);
        rekfs.posture.nowPonint.x=path[0];
        rekfs.posture.nowPonint.y=path[1];
        xprim=diff_path[0]/splinelength;
        yprim=diff_path[1]/splinelength;
        xpprim=double_diff_path[0]/(splinelength*splinelength);
        ypprim=double_diff_path[1]/(splinelength*splinelength);

        rekfs.posture.p=atan2(yprim,xprim);

        rekfs.DoubleS=(xprim*ypprim-xpprim*yprim)/powl((xprim*xprim+yprim*yprim),3/2);

         pck[0]=rekfs.posture.nowPonint.x;
         pck[1]=rekfs.posture.nowPonint.y;
         pck[2]=rekfs.posture.p;
         pck[3]=rekfs.DoubleS;
//            pck[0]=diff_path[0];
//            pck[1]=diff_path[1];
//            pck[2]=double_diff_path[0];
//            pck[3]=double_diff_path[1];
	}
	else
	{
        pck[0]=1;
        pck[1]=2;
        pck[2]=3;
        pck[3]=4;
	}
}
	


/*下面这个mexFunction的目的是使MATLAB知道如何调用这个timestwo函数*/ 
void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[])      
    /* nlhs是MATLAB命令行方式下输出参数的个数； 
     *plhs[]是MATLAB命令行方式下的输出参数； 
     nrhs是MATLAB命令行方式下输入参数的个数； 
     *prhs[]是MATLAB命令行方式下的输入参数； */   
    { 

   double s,*y; 
    int mrows,ncols; 
    /* Check for proper number of arguments. */ 
    if(nrhs!=1) { 
    mexErrMsgTxt("One input required."); 
    } else if(nlhs>1) { 
    mexErrMsgTxt("Too many output arguments"); 
    } 

     /* 在MATLAB命令行方式下，本MEX文件的调用格式是y=timestwo(x) 
     输入参数（x）个数＝1，输出参数（y）个数＝1，所以在程序一 
     开始就检查nrhs是否＝1以及nlhs是否>1（因为MATLAB有一个缺省 
     输出参数ans，所以nlhs可以=0 */ 
    /* The input must be a noncomplex scalar double.*/ 

    /* 为输出创佳一个矩阵，显然这个矩阵也应该是1x1的 */ 
    plhs[0] = mxCreateDoubleMatrix(4,1, mxREAL); 
    /* 获得指向输入/输出矩阵数据的指针 */ 
    s = *(mxGetPr(prhs[0])); 
    y = mxGetPr(plhs[0]); 
    /* 调用C++函数timestwo(y,x) */ 
    BSpline(s,y);
    } 

