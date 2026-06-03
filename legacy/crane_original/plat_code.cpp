#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <string.h>

#define TRUE  1
#define FALSE 0

#ifndef D2R
#define D2R  0.01745329
#endif

#ifndef R2D
#define R2D 57.29577951
#endif

#define Sqrt(x)  sqrt((double)(x))
#define Fabs(x)  fabs((double)(x))

struct plat_params_struct
{
	double p0_x0 ;        // X coordinate of point P0 in the 0 coordinate system (Y & Z values are 0)
	double q0_x0, q0_y0 ; // X and Y coordinates of point Q0 in 0 coordinate system (Z value is 0)
    double s1_x1 ;        // X coordinate of point S1 in the 1 coordinate system (Y & Z values are 0)
	double t1_x1, t1_y1 ; // X and Y coordinates of point T1 in 1 coordinate system (Z value is 0)

	double o0s0 ;         // distance between points O0 and S0
	double p0t0 ;         // distance between points P0 and T0
	double q0r0 ;         // distance between points Q0 and R0
	double r1o1 ;         // distance between points R1 and O1
	double s1p1 ;         // distance between points S1 and P1
	double t1q1 ;         // distance between points T1 and Q1
} ;

typedef struct Polyy{
	int deg;
	double coef[37];
	double eval(double x);
	} Poly ;
double Poly::eval(double x)
{
   int i ;
   double result = coef[0] ;
   double val ;

   if (deg > 0)
      result += coef[1]*x ;

   for (i=2 ; i<=deg ; i++)
     {
      val = pow (x, (double)i) ;
      result += val * coef[i] ;
     }

   return (result) ;
}

void pmult( Poly a, Poly b, Poly *c );
void psub( Poly a, Poly b, Poly *c );
void padd( Poly a, Poly b, Poly *c );
void pscale( Poly a, double s, Poly *as );

void solve_georedux (double L_o0o1,double L_s0s1,double L_p0p1,double L_t0t1,
					 double L_q0q1,double L_r0r1,double Lsfor33[6]) ;
void solve_platform (int *pnum_solutions,
		     double T_2_1[8][4][4],
		     double p_x_1, double q_x_1, double q_y_1,
		     double s_x_2, double t_x_2, double t_y_2,
		     double L_or, double L_os, double L_ps,
		     double L_pt, double L_qt, double L_qr) ;

void matmult(double ans[4][4], double matrix1[4][4], double matrix2[4][4]);
void vecmult(double ans1[4], double matrix1[4][4], double vector1[4]);
double dotproduct(double vector1[3], double vector2[3]);
void crossproduct(double ans[3], double vector1[3], double vector2[3]);
double vecmag(double vector[3]);
int valuenear (double x, double goal, double tol) ;
void Inverse(double matdata[], int numcol, double *det,
			double invary[]);
void MatSwap(double *s1, double *s2);
void Transpose(double *a, double *b, int m, int n);


/*
Function to reduce the special 66 platform geometry to the
 33 geometry in order to calculate the 33 leg lengths to send
 to the "solve_platform" function to do a forward analysis
*/
void solve_georedux(struct plat_params_struct *param, double L_o0o1,double L_s0s1,double L_p0p1,
					double L_t0t1,double L_q0q1,double L_r0r1,
					double Lsfor33[6])
{
	double o0p0, o0s0, p0q0, p0t0, q0o0, q0r0, s0p0, t0q0, r0o0,
		   r1s1, r1o1, s1t1, s1p1, t1r1, t1q1, o1s1, p1t1, q1r1,
		   A, B, C, D, E, F, k1, k2, k3, k4, k5, k6,
		   K0, K1, K2, K3, K4, K5, K6,
		   m1, m2, m3, m4, m5, m6;

	o0p0 = param->p0_x0 ;
	o0s0 = param->o0s0  ;
	p0q0 = sqrt(pow(param->p0_x0-param->q0_x0,2) + pow(param->q0_y0,2)) ;
	p0t0 = param->p0t0 ;
	q0o0 = sqrt(pow(param->q0_x0,2) + pow(param->q0_y0,2)) ;
	q0r0 = param->q0r0 ;
	s0p0 = o0p0 - o0s0 ;
	t0q0 = p0q0 - p0t0 ;
	r0o0 = q0o0 - q0r0 ;
	r1s1 = param->s1_x1 ;
	r1o1 = param->r1o1 ;
	s1t1 = sqrt(pow(param->s1_x1-param->t1_x1,2) + pow(param->t1_y1,2)) ; ;
	s1p1 = param->s1p1 ;
	t1r1 = sqrt(pow(param->t1_x1,2) + pow(param->t1_y1,2)) ;
	t1q1 = param->t1q1 ;
	o1s1 = r1s1 - r1o1 ;
	p1t1 = s1t1 - s1p1 ;
	q1r1 = t1r1 - t1q1 ;
	
	A = o1s1/r1o1;
	B = p1t1/s1p1;
	C = q1r1/t1q1;
	D = s0p0/o0s0;
	E = t0q0/p0t0;
	F = q0r0/r0o0;

	k1 = (o1s1*r1o1) + (o1s1*o1s1);
	k2 = (p1t1*s1p1) + (p1t1*p1t1);
	k3 = (q1r1*t1q1) + (q1r1*q1r1);
	k4 = (s0p0*o0s0) + (s0p0*s0p0);
	k5 = (t0q0*p0t0) + (t0q0*t0q0);
	k6 = (r0o0*q0r0) + (q0r0*q0r0);

	m1 = r1s1/r1o1;
	m2 = s1t1/s1p1;
	m3 = t1r1/t1q1;
	m4 = o0p0/o0s0;
	m5 = p0q0/p0t0;
	m6 = q0o0/r0o0;

	K0 = (k6 - k3 + C*k5 - C*E*k2 + B*C*E*k4 - B*C*D*E*k1)/(F - A*B*C*D*E);
	K1 = (-B*C*D*E*m1)/(F - A*B*C*D*E);
	K2 = (B*C*E*m4)/(F - A*B*C*D*E);
	K3 = (-C*E*m2)/(F - A*B*C*D*E);
	K4 = (C*m5)/(F - A*B*C*D*E);
	K5 = (-m3)/(F - A*B*C*D*E);
	K6 = (m6)/(F - A*B*C*D*E);

	Lsfor33[0] = sqrt(fabs(K0 + K1*L_o0o1*L_o0o1 + K2*L_s0s1*L_s0s1 + K3*L_p0p1*L_p0p1
				     + K4*L_t0t1*L_t0t1 + K5*L_q0q1*L_q0q1 + K6*L_r0r1*L_r0r1));//or
	Lsfor33[1] = sqrt(fabs(k1 + m1*L_o0o1*L_o0o1 - A*Lsfor33[0]*Lsfor33[0]));//os
	Lsfor33[2] = sqrt(fabs(k4 + m4*L_s0s1*L_s0s1 - D*Lsfor33[1]*Lsfor33[1]));//ps
	Lsfor33[3] = sqrt(fabs(k2 + m2*L_p0p1*L_p0p1 - B*Lsfor33[2]*Lsfor33[2]));//pt
	Lsfor33[4] = sqrt(fabs(k5 + m5*L_t0t1*L_t0t1 - E*Lsfor33[3]*Lsfor33[3]));//qt
	Lsfor33[5] = sqrt(fabs(k3 + m3*L_q0q1*L_q0q1 - C*Lsfor33[4]*Lsfor33[4]));//qr

/*	cout << "\n" << Lsfor33[0] << "\n" << Lsfor33[1] << "\n" << Lsfor33[2] << "\n" << Lsfor33[3] << "\n" << Lsfor33[4] << "\n" << Lsfor33[5] << "\n";*/
}

/*
Function to perform forward analysis of 33 stewart platform
*/
void solve_platform (int *pnum_solutions,
		     double T_2_1[8][4][4],
		     double p_x_1, double q_x_1, double q_y_1,
		     double s_x_2, double t_x_2, double t_y_2,
		     double L_or, double L_os, double L_ps,
		     double L_pt, double L_qt, double L_qr)
{
   int poly_solve(double root_r[], double root_c[], int d, double coeff[]) ;
   int i ;

   double p_1[3], q_1[3], vk[3];
   p_1[0] = p_x_1;
   p_1[1] = 0.0;
   p_1[2] = 0.0;
   q_1[0] = q_x_1;
   q_1[1] = q_y_1;
   q_1[2] = 0.0;
   vk[0]  = 0.0;
   vk[1]  = 0.0;
   vk[2]  = 1.0;

   double L_op, L_pq, L_oq ;
   L_op = vecmag(p_1);//sqrt(p_1[0]*p_1[0] + p_1[1]*p_1[1] + p_1[2]*p_1[2]);//!p_1 ;
   L_oq = vecmag(q_1);//sqrt(q_1[0]*q_1[0] + q_1[1]*q_1[1] + q_1[2]*q_1[2]);//!q_1 ;
   L_pq = sqrt(fabs((p_1[0] - q_1[0])*(p_1[0] - q_1[0])
		  +(p_1[1] - q_1[1])*(p_1[1] - q_1[1])
		  +(p_1[2] - q_1[2])*(p_1[2] - q_1[2])));//!(p_1 - q_1) ;


   double s_2[3], t_2[3];
   s_2[0] = s_x_2;
   s_2[1] = 0.0;
   s_2[2] = 0.0;
   t_2[0] = t_x_2;
   t_2[1] = t_y_2;
   t_2[2] = 0.0;

   double L_rs, L_rt, L_st ;
   L_rs = vecmag(s_2);//sqrt(s_2[0]*s_2[0] + s_2[1]*s_2[1] + s_2[2]*s_2[2]);//!s_2 ;
   L_rt = vecmag(t_2);//sqrt(t_2[0]*t_2[0] + t_2[1]*t_2[1] + t_2[2]*t_2[2]);//!t_2 ;
   L_st = sqrt(fabs((s_2[0] - t_2[0])*(s_2[0] - t_2[0])
		  +(s_2[1] - t_2[1])*(s_2[1] - t_2[1])
		  +(s_2[2] - t_2[2])*(s_2[2] - t_2[2])));//!(s_2 - t_2) ;

   double c41, s41, c34, s34, c12, s12, c23 ;
   double c41_o, s41_o ;
   double c41_p, s41_p ;
   double pxq[3];//cross product of p_1 and q_1

/* four bar at point O ////////////////////////////////////////*/

   c41_o = c41 = dotproduct(p_1,q_1)/(L_op*L_oq);
   crossproduct(pxq,p_1,q_1);
   s41_o = s41 = (pxq[2]/(L_op*L_oq))*vk[2];

   c23 = (L_or*L_or + L_os*L_os - L_rs*L_rs) / (2.0*L_or*L_os) ;

   c34 = (L_os*L_os + L_op*L_op - L_ps*L_ps) / (2.0*L_os*L_op) ;
   s34 = sin(acos(c34)) ;

   c12 = (L_oq*L_oq + L_or*L_or - L_qr*L_qr) / (2.0*L_oq*L_or) ;
   s12 = sin(acos(c12)) ;

   /* First equation
      AA1 y^2 x^2 + BB1 x^2 + CC1 y^2 + DD1 x y + EE1 = 0  */
   double AA1, BB1, CC1, DD1, EE1 ;

   AA1 = s12 * (s41*c34 - c41*s34) + c12*(c41*c34+s41*s34) - c23 ;
   BB1 = s12 * (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;
   CC1 = s12 * (c41*s34 - s41*c34) + c12*(c41*c34+s41*s34) - c23 ;
   DD1 = 4.0 * s12 * s34 ;
   EE1 = -s12* (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;

/* four bar at point P ////////////////////////////////////////*/

   double v_pq[3], v_po[3];
   v_pq[0] = q_1[0] - p_1[0];
   v_pq[1] = q_1[1] - p_1[1];
   v_pq[2] = q_1[2] - p_1[2];
   v_po[0] = -p_1[0];
   v_po[1] = -p_1[1];
   v_po[2] = -p_1[2];

   c41 = dotproduct(v_pq,v_po)/(vecmag(v_po)*vecmag(v_pq));
   crossproduct(pxq,v_pq,v_po);
   s41 = (pxq[2]/(vecmag(v_po)*vecmag(v_pq)))*vk[2];

   c41_p = -c41 ;
   s41_p =  s41 ;

   c23 = (L_pt*L_pt + L_ps*L_ps - L_st*L_st) / (2.0*L_pt*L_ps) ;

   c34 = (L_pq*L_pq + L_pt*L_pt - L_qt*L_qt) / (2.0*L_pq*L_pt) ;
   s34 = sin(acos(c34)) ;

   c12 = (L_op*L_op + L_ps*L_ps - L_os*L_os) / (2.0*L_op*L_ps) ;
   s12 = sin(acos(c12)) ;

   /* Third equation
      AA3 y^2 z^2 + BB3 y^2 + CC3 z^2 + DD3 y z + EE3 = 0 */
   double AA3, BB3, CC3, DD3, EE3 ;

   AA3 = s12 * (s41*c34 - c41*s34) + c12*(c41*c34+s41*s34) - c23 ;
   BB3 = s12 * (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;
   CC3 = s12 * (c41*s34 - s41*c34) + c12*(c41*c34+s41*s34) - c23 ;
   DD3 = 4.0 * s12 * s34 ;
   EE3 = -s12* (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;

/* four bar at point Q ////////////////////////////////////////*/

   double v_qp[3], v_qo[3];
   v_qp[0] = - v_pq[0] ;
   v_qp[1] = - v_pq[1] ;
   v_qp[2] = - v_pq[2] ;
   v_qo[0] = - q_1[0] ;
   v_qo[1] = - q_1[1] ;
   v_qo[2] = - q_1[2] ;

   c41 = dotproduct(v_qo,v_qp)/(vecmag(v_qo)*vecmag(v_qp));
   crossproduct(pxq,v_qo,v_qp);
   s41 = (pxq[2]/(vecmag(v_qo)*vecmag(v_qp)))*vk[2];

   c23 = (L_qt*L_qt + L_qr*L_qr - L_rt*L_rt) / (2.0*L_qt*L_qr) ;

   c34 = (L_qr*L_qr + L_oq*L_oq - L_or*L_or) / (2.0*L_qr*L_oq) ;
   s34 = sin(acos(c34)) ;

   c12 = (L_pq*L_pq + L_qt*L_qt - L_pt*L_pt) / (2.0*L_pq*L_qt) ;
   s12 = sin(acos(c12)) ;

   /* Second equation
      AA2 z^2 x^2 + BB2 z^2 + CC2 x^2 + DD2 z x + EE2 = 0 */
   double AA2, BB2, CC2, DD2, EE2 ;

   AA2 = s12 * (s41*c34 - c41*s34) + c12*(c41*c34+s41*s34) - c23 ;
   BB2 = s12 * (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;
   CC2 = s12 * (c41*s34 - s41*c34) + c12*(c41*c34+s41*s34) - c23 ;
   DD2 = 4.0 * s12 * s34 ;
   EE2 = -s12* (c41*s34 + s41*c34) + c12*(c41*c34-s41*s34) - c23 ;

/* Form up the i/o equation.*/
   Poly a1, b1, c1, a2, b2, c2,
	temp1, temp2, a1a2, c1c2, a1c2, a2c1,
	b1b1, b2b2, b1b2, a2c2, c2c2, a1c1, a1a1, c1c1, a2a2,
	DD, p32, p33, p34, p35, p36, alpha, beta, rho1, rho2, ioeqn;

	a1.deg=2; a2.deg=2; c1.deg=2; c2.deg=2; b1.deg=1; b2.deg=1;

	a1.coef[0]=CC1;
	a1.coef[1]=0.0;
	a1.coef[2]=AA1;
	a2.coef[0]=BB2;
	a2.coef[1]=0.0;
	a2.coef[2]=AA2;
	c1.coef[0]=EE1;
	c1.coef[1]=0.0;
	c1.coef[2]=BB1;
	c2.coef[0]=EE2;
	c2.coef[1]=0.0;
	c2.coef[2]=CC2;
	b1.coef[0]=0.0;
	b1.coef[1]=0.5*DD1;
	b2.coef[0]=0.0;
	b2.coef[1]=0.5*DD2;

/*
	for(i = 0; i<3; ++i)
	{
		cout << "a1.coef[i] = " << a1.coef[i] << "\n";
		cout << "a2.coef[i] = " << a2.coef[i] << "\n";
		cout << "c1.coef[i] = " << c1.coef[i] << "\n";
		cout << "c2.coef[i] = " << c2.coef[i] << "\n";
	}
*/

	pmult( a1, a2, &a1a2 );
	pmult( c1, c2, &c1c2 );
	pmult( a2, c1, &a2c1 );
	pmult( a1, c2, &a1c2 );
	pmult( a2, c2, &a2c2 );
	pmult( c2, c2, &c2c2 );
	pmult( a2, a2, &a2a2 );
	pmult( a1, c1, &a1c1 );
	pmult( c1, c1, &c1c1 );
	pmult( a1, a1, &a1a1 );
	pmult( b1, b1, &b1b1 );
	pmult( b2, b2, &b2b2 );
	pmult( b1, b2, &b1b2 );

	pscale( a2c1, 2.0*AA3*BB3, &temp1);
	pmult(temp1, c1c2, &temp1);
	//pmult(temp1, c1c2, &p1);

/*	cout << AA3 << "\n";
	cout << BB3 << "\n";

	cout << a2c1.coef[0] << "  " << a2c1.coef[1] << "  " << a2c1.coef[2] << "  " << a2c1.coef[6] << "\n";
	cout << temp1.coef[0] << "  " << temp1.coef[1] << "  " << temp1.coef[2] << "  " << temp1.coef[6] << "\n";
	cout << c1c2.coef[0] << "  " << c1c2.coef[1] << "  " << c1c2.coef[2] << "  " << c1c2.coef[6] << "\n";
	cout << p1.coef[0] << "  " << p1.coef[1] << "  " << p1.coef[2] << "  " << p1.coef[6] << "\n";
*/
	pscale( c1c1, 4.0*AA3*BB3, &temp2);
	pmult(temp2, b2b2, &temp2);

	psub( temp1, temp2, &DD );

	pscale( a1c1, 2.0*AA3*CC3, &temp1);
	pmult(temp1, c2c2, &temp2);

	padd( DD, temp2, &DD );

	pscale( c2c2, 4.0*AA3*CC3, &temp1);
	pmult(temp1, b1b1, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a2, 2.0*AA3*EE3, &temp1);
	pmult(temp1, c1c2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1c1, 4.0*AA3*EE3, &temp1);
	pmult(temp1, b2b2, &temp2);

	padd( DD, temp2, &DD );

	pscale( a2c2, 4.0*AA3*EE3, &temp1);
	pmult(temp1, b1b1, &temp2);

	padd( DD, temp2, &DD );

	pscale( b1b1, 8.0*AA3*EE3, &temp1);
	pmult(temp1, b2b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( c1c2, 2.0*AA3*DD3, &temp1);
	pmult(temp1, b1b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a2, 2.0*BB3*CC3, &temp1);
	pmult(temp1, c1c2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1c1, 4.0*BB3*CC3, &temp1);
	pmult(temp1, b2b2, &temp2);

	padd( DD, temp2, &DD );

	pscale( a2c2, 4.0*BB3*CC3, &temp1);
	pmult(temp1, b1b1, &temp2);

	padd( DD, temp2, &DD );

	pscale( b1b1, 8.0*BB3*CC3, &temp1);
	pmult(temp1, b2b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a2, 2.0*BB3*EE3, &temp1);
	pmult(temp1, a2c1, &temp2);

	padd( DD, temp2, &DD );

	pscale( a2a2, 4.0*BB3*EE3, &temp1);
	pmult(temp1, b1b1, &temp2);

	psub( DD, temp2, &DD );

	pscale( a2c1, 2.0*BB3*DD3, &temp1);
	pmult(temp1, b1b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a1, 2.0*CC3*EE3, &temp1);
	pmult(temp1, a2c2, &temp2);

	padd( DD, temp2, &DD );

	pscale( a1a1, 4.0*CC3*EE3, &temp1);
	pmult(temp1, b2b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1c2, 2.0*CC3*DD3, &temp1);
	pmult(temp1, b1b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a2, 2.0*DD3*EE3, &temp1);
	pmult(temp1, b1b2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a2, DD3*DD3, &temp1);
	pmult(temp1, c1c2, &temp2);

	psub( DD, temp2, &DD );

	pscale( c1c1, AA3*AA3, &temp1);
	pmult(temp1, c2c2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a2a2, BB3*BB3, &temp1);
	pmult(temp1, c1c1, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a1, CC3*CC3, &temp1);
	pmult(temp1, c2c2, &temp2);

	psub( DD, temp2, &DD );

	pscale( a1a1, EE3*EE3, &temp1);
	pmult(temp1, a2a2, &temp2);

	psub( DD, temp2, &alpha );

	pscale( b1b2, 4.0*AA3*EE3, &temp1);
	pscale( c1c2, AA3*DD3, &temp2);
		padd(temp1, temp2, &beta);
	pscale( a1a2, DD3*EE3, &temp1);
		padd(beta, temp1, &beta);
	pscale( b1b2, 4.0*BB3*CC3, &temp1);
		psub(beta, temp1, &beta);
	pscale( a2c1, BB3*DD3, &temp1);
		psub(beta, temp1, &beta);
	pscale( a1c2, CC3*DD3, &temp1);
		psub(beta, temp1, &beta);

	psub(b1b1, a1c1, &rho1);
	psub(b2b2, a2c2, &rho2);

	pmult(alpha, alpha, &p32);
	pmult(beta, beta, &p33);
	pmult(rho1, rho2, &p34);
	pscale(p33, 4.0, &p35);
	pmult(p35, p34, &p36);
	psub(p32, p36, &ioeqn);
/*   for (i=0; i<17; ++i)
     {
	     cout << "ioeqn[i] = " << ioeqn.coef[i] << "\n";
     }
*/

   double unitval, tempunitval ;
   unitval = ioeqn.coef[16] ;
   tempunitval = 1.0/unitval;
   pscale(ioeqn, tempunitval, &ioeqn);

   double coef2[9] ;
   for (i=8 ; i>=0 ; --i)
     {
      coef2[i] = ioeqn.coef[2*i] ;
/*	  cout << "coef2[i] = " << coef2[i] << "\n";*/
     }

   double xsq_r[8], xsq_c[8] ;
   int OK ;
   OK = poly_solve(xsq_r, xsq_c, 8, coef2) ;
   if (OK != 1)
     {
//      cout << "\nERROR in poly_solve\n\n" ;
      exit(9) ;
     }

   int num_real = 0 ;
   double xx[8], yy[8], zz[8] ;
   for (i=0 ; i<8 ; ++i)
      if (valuenear(xsq_c[i], 0.0, 0.0001) && (xsq_r[i] >= 0.0))
	{
	 xx[num_real] = sqrt(fabs(xsq_r[i])) ;
	 num_real++ ;
	}

   *pnum_solutions = num_real ;

   /* Find corresponding values for thetay and thetaz.*/

   double y_candidate[2], z_candidate[2] ;
   double aa1, bb1, cc1, aa2, bb2, cc2 ;
   double aa3, bb3, cc3, dd3 ;
   double discr ;
   int badone[8]={0,0,0,0,0,0,0,0} ;
   double cand_value[4] ;
   double ang, cos_ang ;  // fold angles

   /* Get coordinates of points r, s, and t in the 1st coord. system
      (get the coordinates in the 1st system, then fold the triangles)
      get point s coordinates before folding */
   double sx_prefold, sy_prefold ;
   double sx, sy, sz ;
   cos_ang = (L_op*L_op + L_os*L_os - L_ps*L_ps) / (2.0*L_op*L_os) ;
   ang = acos(cos_ang) ;
   sx_prefold =   L_os*cos_ang ;
   sy_prefold = - L_os*sin(ang) ;

   /* get point r in xtra coordinate system before folding*/
   double rx_prefold, ry_prefold ;
   double rx, ry, rz ;
   cos_ang = (L_or*L_or + L_oq*L_oq - L_qr*L_qr) / (2.0*L_or*L_oq) ;
   ang = acos(cos_ang) ;
   rx_prefold =   L_or*cos_ang ;
   ry_prefold =   L_or*sin(ang) ;

   /* get point t in xtra2 coordinate system before folding*/
   double tx_prefold, ty_prefold ;
   double tx, ty, tz ;
   cos_ang = (L_pt*L_pt + L_pq*L_pq - L_qt*L_qt) / (2.0*L_pt*L_pq) ;
   ang = acos(cos_ang) ;
   tx_prefold =   L_pt*cos_ang ;
   ty_prefold = - L_pt*sin(ang) ;

   double thetax, thetay, thetaz ;
   double sin_x, cos_x, sin_y, cos_y, sin_z, cos_z ;

   double xvec[3], yvec[3], zvec[3], tempvec[3];
/*   cdc_vector xvec(3L), yvec(3L), zvec(3L), tempvec(3L) ;*/

   for (i=0 ; i< *pnum_solutions ; ++i)
     {
      aa1 = a1.eval(xx[i]) ;
/*    	  cout << "\naa1 = " << aa1;*/
      bb1 = b1.eval(xx[i]) ;
/*	  	  cout << "\nbb1 = " << bb1;*/
      cc1 = c1.eval(xx[i]) ;
/*	  	  cout << "\ncc1 = " << cc1;*/
      aa2 = a2.eval(xx[i]) ;
/*	  	  cout << "\naa2 = " << aa2;*/
      bb2 = b2.eval(xx[i]) ;
/*	  	  cout << "\nbb2 = " << bb2;*/
      cc2 = c2.eval(xx[i]) ;
/*	  	  cout << "\ncc2 = " << cc2;*/
      discr = 4.0*bb1*bb1 - 4.0*aa1*cc1 ;
      if (discr < 0)
		{
			badone[i] = TRUE ;
//			cout << "bady"<< discr << endl ;
			continue ;
		}
      y_candidate[0] = (-2.0*bb1 + sqrt(discr)) / (2.0*aa1) ;
      y_candidate[1] = (-2.0*bb1 - sqrt(discr)) / (2.0*aa1) ;

      discr = 4.0*bb2*bb2 - 4.0*aa2*cc2 ;
      if (discr < 0)
		{	
			badone[i] = TRUE ;
//			cout << "badz"<< discr <<endl ;
			continue ;
		}
      z_candidate[0] = (-2.0*bb2 + sqrt(discr)) / (2.0*aa2) ;
      z_candidate[1] = (-2.0*bb2 - sqrt(discr)) / (2.0*aa2) ;

      aa3 = 4.0*AA3*bb1*bb2 + DD3*aa1*aa2 ;
      bb3 = 2.0*AA3*bb1*cc2 - 2.0*BB3*aa2*bb1 ;
      cc3 = 2.0*AA3*bb2*cc1 - 2.0*CC3*aa1*bb2 ;
      dd3 = AA3*cc1*cc2 + EE3*aa1*aa2 - BB3*aa2*cc1 - CC3*aa1*cc2 ;

      cand_value[0] = fabs(aa3*y_candidate[0]*z_candidate[0]
		    + bb3*y_candidate[0] + cc3*z_candidate[0] + dd3) ;

      cand_value[1] = fabs(aa3*y_candidate[1]*z_candidate[0]
		    + bb3*y_candidate[1] + cc3*z_candidate[0] + dd3) ;

      cand_value[2] = fabs(aa3*y_candidate[0]*z_candidate[1]
		    + bb3*y_candidate[0] + cc3*z_candidate[1] + dd3) ;

      cand_value[3] = fabs(aa3*y_candidate[1]*z_candidate[1]
		    + bb3*y_candidate[1] + cc3*z_candidate[1] + dd3) ;
      if ((cand_value[0] < cand_value[1]) && (cand_value[0] < cand_value[2])
	       && (cand_value[0] < cand_value[3]))
		{
			yy[i] = y_candidate[0] ;
			zz[i] = z_candidate[0] ;
		}

      else if ((cand_value[1] < cand_value[0]) &&
	   (cand_value[1] < cand_value[2]) && (cand_value[1] < cand_value[3]))
		{
			yy[i] = y_candidate[1] ;
			zz[i] = z_candidate[0] ;
		}

      else if ((cand_value[2] < cand_value[0]) &&
	   (cand_value[2] < cand_value[1]) && (cand_value[2] < cand_value[3]))
		{
			yy[i] = y_candidate[0] ;
			zz[i] = z_candidate[1] ;
		}

      else if ((cand_value[3] < cand_value[0]) &&
	   (cand_value[3] < cand_value[1]) && (cand_value[3] < cand_value[2]))
		{
			yy[i] = y_candidate[1] ;
			zz[i] = z_candidate[1] ;
		}
      thetax = 2.0*atan(xx[i]) ;
      thetay = 2.0*atan(yy[i]) ;
      thetaz = 2.0*atan(zz[i]) ;

      sin_x = sin(thetax) ;   cos_x = cos(thetax) ;
      sin_y = sin(thetay) ;   cos_y = cos(thetay) ;
      sin_z = sin(thetaz) ;   cos_z = cos(thetaz) ;

      sx =  sx_prefold ;
      sy =  cos_y * sy_prefold ;
      sz = -sin_y * sy_prefold ;

      rx = c41_o * rx_prefold - s41_o * cos_x * ry_prefold ;
      ry = s41_o * rx_prefold + c41_o * cos_x * ry_prefold ;
      rz = sin_x * ry_prefold ;

      tx = c41_p * tx_prefold - s41_p * cos_z * ty_prefold + L_op ;
      ty = s41_p * tx_prefold + c41_p * cos_z * ty_prefold ;
      tz = -sin_z * ty_prefold ;

      /* Enter origin of 2nd coord system as seen in 1st*/
      T_2_1[i][0][3] = rx ;
      T_2_1[i][1][3] = ry ;
      T_2_1[i][2][3] = rz ;
      T_2_1[i][3][3] = 1.0 ;

      /* Enter x axis of 2nd coord system as seen in 1st*/
      xvec[0] = sx - rx ;
      xvec[1] = sy - ry ;
      xvec[2] = sz - rz ;
	  double tempmag;
	  tempmag = vecmag(xvec);
	  xvec[0] = xvec[0]/tempmag;
	  xvec[1] = xvec[1]/tempmag;
	  xvec[2] = xvec[2]/tempmag;
/*      xvec = ~xvec ;*/
      T_2_1[i][0][0] = xvec[0] ;
      T_2_1[i][1][0] = xvec[1] ;
      T_2_1[i][2][0] = xvec[2] ;
      T_2_1[i][3][0] = 0.0 ;

      /* Enter z axis of 2nd coord system as seen in 1st*/
      tempvec[0] = tx - rx ;
      tempvec[1] = ty - ry ;
      tempvec[2] = tz - rz ;
/*      zvec = xvec ^ tempvec;*/
	  crossproduct(zvec,xvec,tempvec) ;
	  tempmag = vecmag(zvec);
	  zvec[0] = zvec[0]/tempmag;
	  zvec[1] = zvec[1]/tempmag;
	  zvec[2] = zvec[2]/tempmag;
/*      zvec = ~zvec ;*/
      T_2_1[i][0][2] = zvec[0] ;
      T_2_1[i][1][2] = zvec[1] ;
      T_2_1[i][2][2] = zvec[2] ;
      T_2_1[i][3][2] = 0.0 ;

      /* Enter y axis of 2nd coord system as seen in 1st*/

	  crossproduct(yvec,zvec,xvec);
	  tempmag = vecmag(yvec);
	  yvec[0] = yvec[0]/tempmag;
	  yvec[1] = yvec[1]/tempmag;
	  yvec[2] = yvec[2]/tempmag;

/*      yvec = zvec ^ xvec ;*/
/*      yvec = ~yvec ;*/
      T_2_1[i][0][1] = yvec[0] ;
      T_2_1[i][1][1] = yvec[1] ;
      T_2_1[i][2][1] = yvec[2] ;
      T_2_1[i][3][1] = 0.0 ;
     }

}

/*Function to multiply two matrices and return the answer*/
void matmult(double ans[4][4],double matrix1[4][4], double matrix2[4][4])
{
   int i,j,k;

   for(i=0;i<4;i++)
   {
      for(j=0;j<4;j++)
      {
         for(k=0;k<4;k++)
	 {
	    ans[i][j]+=matrix1[i][k]*matrix2[k][j];
	 }
      }
   }
}

/*Function to multiply a matrix times a vector and return the answer*/
void vecmult(double ans1[4], double matrix1[4][4], double vector1[4])
{
   int i,j;
   for(i=0;i<4;i++)
	   ans1[i] = 0;

   for(i=0;i<4;i++)
   {
      for(j=0;j<4;j++)
      {
	 ans1[i]+=matrix1[i][j]*vector1[j];
      }
   }
}

double dotproduct(double vector1[3], double vector2[3])
{
	double ans = 0;
	int i;

	for(i=0;i<3;i++)
	{
		ans += vector1[i]*vector2[i];
	}
	return ans;
}

void crossproduct(double ans[3], double vector1[3], double vector2[3])
{
	ans[0] = vector1[1]*vector2[2]-vector1[2]*vector2[1];
	ans[1] = vector1[2]*vector2[0]-vector1[0]*vector2[2];
	ans[2] = vector1[0]*vector2[1]-vector1[1]*vector2[0];
}

double vecmag(double vector[3])
{
	double ans;
	ans = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
	return ans;
}

int valuenear (double val, double goal, double tol)
{
   if ((val > goal-tol) && (val < goal+tol))
      return 1 ;
   else
      return 0 ;
}

int _INT[100];
char _CHAR[50];


void Inverse(double matdata[], int numcol, double *det,
			double invary[])
{
   int  *pivlst, i, j, k, l, lerow, lecol, l1;
   char *pivchk;
   double piv, t, leval;

   pivlst = &(_INT[0]); // (int *) calloc(50*2,2)  ;
   pivchk = &(_CHAR[0]); // (char *) calloc(50,1)  ;
   (*det) = 1.0;
   for ( i = 0; i <= numcol-1; ++i ) {
      pivchk[i] = 0;
	 for ( j = 0; j <= numcol-1; ++j )
	 invary[i*numcol+j] = matdata[i*numcol+j];
   }
   for ( i = 0; i <= numcol-1; ++i ) {
      leval = 0.0;
      for ( j = 0; j <= numcol-1; ++j ) {
	 if ( ! (pivchk[j]) ) {
	    for ( k = 0; k <= numcol-1; ++k ) {
	       if ( ! (pivchk[k]) ) {
		  if ( fabs(invary[j*numcol+k]) > leval ) {
		     lerow = j;
		     lecol = k;
		     leval = fabs(invary[j*numcol+k]);
		  }
	       }
	    }
	 }
      }
      pivchk[lecol] = 1;
      pivlst[i*2] = lerow;
      pivlst[i*2+1] = lecol;
      if ( lerow != lecol ) {
	 (*det) = -(*det);
	 for ( l = 0; l <= numcol-1; ++l ) {
		  MatSwap(&invary[lerow*numcol+l],
			&invary[lecol*numcol+l]);
	 }
      }
      piv = invary[lecol*numcol+lecol];
      (*det) = (*det) * piv;
      if ( (*det) > 1.0e+30 ) {
	 (*det) = 1;
      }
      invary[lecol*numcol+lecol] = 1.0;
      for ( l = 0; l <= numcol-1; ++l ) {
	    invary[lecol*numcol+l] =invary[lecol*numcol+l]/piv;
      }
      for ( l1 = 0; l1 <= numcol-1; ++l1 ) {
	 if ( l1 != lecol ) {
	    t = invary[l1*numcol+lecol];
	    invary[l1*numcol+lecol] = 0;
	    for ( l = 0; l <= numcol-1; ++l ) {
			invary[l1*numcol+l]=invary[l1*numcol+l] -
							invary[lecol*numcol+l]*t;
	    }
	 }
      }
   }
   for ( i = 0; i <= numcol-1; ++i ) {
      l = numcol - i -1;
      if ( pivlst[l*2] != pivlst[l*2+1] ) {
	 lerow = pivlst[l*2];
	 lecol = pivlst[l*2+1];
	    for ( k = 0; k <= numcol-1; ++k )
		 MatSwap(&invary[k*numcol+lerow],
			&invary[k*numcol+lecol]);
      }
   }

//   free(pivlst); free(pivchk);

}//	Inverse( (double*)J, 6, &c, (double*)Jinv );

/*********************************************************/

void MatSwap(double *s1, double *s2)
{
   double temp;

   temp = (*s1);
   (*s1) = (*s2);
   (*s2) = temp;
}

/*********************************************************/

void Transpose(double *a, double *b, int m, int n)
{
   int i,j;
   for(i=0 ; i<m ; i++)
    for(j=0 ; j<n ; j++)
     *(b+(j*m)+i) = *(a+(i*n)+j);
}


/*******************************************************/
int poly_solve (double root_r[], double root_c[], int d, double xcof[])

/* This routine will evaluate the roots of a polynomial of
      degree "d" ("d" must be less than or equal to 36).
   "root_r" and "root_c" are the real and complex parts of the
      'd' solutions to the original equation.
   "xcof" is an array of coefficients, ordered from smallest
      to largest power.

   xcof[16] x^16 + xcof[15] x^15 + ... + xcof[1] x + xcof[0] = 0 */

{
   double coef[37], dis, X, Y, Z[37], X0, XX[40],YY[40],
	 U, V, dUx, dUy, den , dX, dY, dXY, XY, C, B[40] ;
   int i, k, deg, cnt ;
   int lst, lflip, ltry ;

   if (d > 36)
      return (0) ;

   for (i=0 ; i<=d ; ++i)
      coef[i] = xcof[i] ;

   deg = d ;

   while (coef[deg] == 0.0)
      deg-- ;  /*The leading coefficient was zero.*/

   if (deg <1)
      return (-1) ;  /*The polynomial must be at least of degree 1.*/

   cnt = 0 ;  /*cnt keeps track of the number of roots found */

   if (deg == 1)
      goto solve_linear ;

   if (deg == 2)
      goto solve_quad ;

   /**************************/
   /*  Set initial values.   */
   /**************************/
L30:
   lst = 0 ;   /*lst counts the number of different starting values*/
   lflip = 0 ; /*lflip determines whether the inverse polynomial is
		 being considered */

   X = 0.00608 ;
   Y = 0.2939 ;

L35:
   X0 = X ;
   X  = -5.0*Y ;
   Y  =  2.0*X0 ;

   ltry = 0 ;  /*ltry counts the # of interations for a starting value*/

   lst++ ;

L38:
   XX[0] = 1.0 ;
   YY[0] = 0.0 ;

   for (i=1 ; i<=deg ; ++i)
     /*Evaluate x^16, x^15, etc where x is complex*/
     {XX[i] = X * XX[i-1] - Y * YY[i-1] ;
      YY[i] = X * YY[i-1] + Y * XX[i-1] ;   /*line 40*/
     }
   U = coef[0] ;
   V = 0.0 ;

   for (i=1 ; i<=deg ; ++i) /*Evaluate the polynomial. */
     {U += coef[i] * XX[i] ;
      V += coef[i] * YY[i] ;
     }

   dUx = 0.0 ;
   dUy = 0.0 ;

   for (i=1 ; i<=deg ; ++i)
     {dUx += i*coef[i] * XX[i-1] ;
      dUy -= i*coef[i] * YY[i-1] ;  /*line 60*/
     }
   den = dUx*dUx + dUy*dUy ;

   dX =  (V*dUy - U*dUx)/den ;
   dY = -(U*dUy + V*dUx)/den ;

   X += dX ;   /*Next try for root. */
   Y += dY ;

   if (Fabs(X) < 40.0)
     {dXY = Sqrt(dX*dX + dY*dY) ;
      XY =  Sqrt(X*X + Y*Y) ;

      if (Fabs(dXY/XY) > 0.0000000002)   /*was 0.0000001 */
	{ltry++ ;
	 if (ltry<400)     /*was 300*/
	    goto L38 ;
	 else
	    goto flip_poly ;
	}
      else
	 goto reduce_poly ;
     }
flip_poly:
   lflip++ ;
   ltry = 0 ;

   for (k=0 ; k<=deg ; ++k)
      Z[k] = coef[deg-k] ;

   for (k=0 ; k<=deg ; ++k)
      coef[k] = Z[k] ;

   if (lflip ==1)
     {X = 0.189 ;
      Y = -0.132 ;
      goto L38 ;
     }

   if (lflip ==2)
      if (lst < 4 )
         goto L35 ;
      return (-300) ;  /*A solution was not found for 300 iterations
                         for 4 starting values. */

   /**************************/
reduce_poly:

   if (Fabs(Y) < 0.000006)  /*was 0.0000005*/
      Y = 0.0 ;
   cnt++ ;

   if (lflip ==1)
     {for (k=0 ; k<=deg ; ++k)  /*flip it back*/
	 Z[k] = coef[deg-k] ;
      for (k=0 ; k<=deg ; ++k)
         coef[k] = Z[k] ;

      den = X*X + Y*Y ; /*The root to the orig. eqn is 1/(X+iY)*/
      root_r[cnt-1] = X = X/den ;
      root_c[cnt-1] = Y = Y/den ;
     }

   else
     {root_r[cnt-1] = X ;
      root_c[cnt-1] = Y ;
     }

   if (Y==0.0)
     {/*Reduce the equation by one degree.*/

      C = X ;
      B[deg] = 0.0 ;
      for (k=deg-1 ; k>=0 ; --k)
	 B[k] = coef[k+1] + C * B[k+1] ;  /*115*/

      deg-- ;  /*Reduce the degree of the polynomial by 1*/

      for (k=0 ; k<=deg ; ++k)
         coef[k] = B[k] ;

      if (deg ==2)
	 goto solve_quad ;
      else if (deg ==1)
         goto solve_linear ;

      else
	 goto L30 ;
     }

   else
     {/*Reduce the equation by the complex conjugates.*/
      cnt++ ;
      root_r[cnt-1] =  X ;
      root_c[cnt-1] = -Y ;

      B[deg-2] = coef[deg] ;
      B[deg-3] = coef[deg-1] + 2.0* X * B[deg-2] ;

      for (k=deg-4 ; k>=0 ; --k)
	{B[k] = coef[k+2]  - (X*X+Y*Y) * B[k+2] + 2.0 * X * B[k+1] ;
        }
      deg -= 2 ;

      for (k=0 ; k<=deg ; ++k)
	 coef[k] = B[k] ;

      if (deg==2)
         goto solve_quad ;
      if (deg==1)
	 goto solve_linear ;
      else
         goto L30 ;
     }


   /**************************/
solve_quad:
   dis = coef[1]*coef[1] - 4.0*coef[2]*coef[0] ;

   X = -coef[1] / (2.0*coef[2]) ;

   if (dis>= 0.0)
     {Y = Sqrt(dis) / (2.0*coef[2]) ;
      root_r[cnt] = X+Y ;
      root_r[cnt+1] = X-Y ;
      root_c[cnt] = root_c[cnt+1] = 0.0 ;
     }

   else
     {Y = Sqrt(-dis)/ (2.0*coef[2]) ;
      root_r[cnt] =   root_r[cnt+1] = X ;
      root_c[cnt] = -(root_c[cnt+1] = Y) ;
     }
   return (1) ;

solve_linear:
   root_r[cnt] = -coef[0] / coef[1] ;
   root_c[cnt] = 0.0 ;
   return (1) ;
}

/*******************************************************/

/**********************************************************

multiplies two Polynomials:

	a[]={ a0, a1, a2, ..., a(da) }
	b[]={ b0, b1, b2, ..., b(db) }
	ab[]={ ab0, ab1, ab2, ..., ab(da+db) }

**********************************************************/

void pmult( Poly A, Poly B, Poly *AB )
{
	int i, j, da, db;
	double *a, *b, *ab;

	da=A.deg;
	db=B.deg;
	a=A.coef;
	b=B.coef;
	ab=AB->coef;
	AB->deg=da+db;

	for(i=0; i<da+db+1; i++)
		ab[i]=0.;

	for(i=0; i<=da; i++)
		for(j=0; j<=db; j++)
			ab[i+j] += a[i] * b[j];
}

/**********************************************************
subtract two Polys:
**********************************************************/

void psub( Poly A, Poly B, Poly *A_B )
{
	int i, ds, db;
	double *a=A.coef, *b=B.coef, *a_b=A_B->coef;

	if( A.deg > B.deg ) {
		db=A.deg;
		ds=B.deg;
		for(i=0; i<=db; i++) {
			if( i<=ds )
				a_b[i] = a[i] - b[i];
			else
				a_b[i] = a[i];
			}
		}
	else {
		db=B.deg;
		ds=A.deg;
		for(i=0; i<=db; i++) {
			if( i<=ds )
				a_b[i] = a[i] - b[i];
			else
				a_b[i] = -b[i];
			}
		}

	A_B->deg=db;

}

/**********************************************************
adds two Polys:

	ba= b + a;

**********************************************************/

void padd( Poly A, Poly B, Poly *A_B )
{
	int i, ds, db;
	double *a=A.coef, *b=B.coef, *a_b=A_B->coef;

	if( A.deg > B.deg ) {
		db=A.deg;
		ds=B.deg;
		for(i=0; i<=db; i++) {
			if( i<=ds )
				a_b[i] = a[i] + b[i];
			else
				a_b[i] = a[i];
			}
		}
	else {
		db=B.deg;
		ds=A.deg;
		for(i=0; i<=db; i++) {
			if( i<=ds )
				a_b[i] = a[i] + b[i];
			else
				a_b[i] = b[i];
			}
		}

	A_B->deg=db;

}

/**********************************************************
scales a Poly:
**********************************************************/

void pscale( Poly A, double s, Poly *AS )
{
	int i;
	double *a=A.coef, *as=AS->coef;

	for(i=0; i<=A.deg; i++)
		as[i] = s*a[i];

	AS->deg=A.deg;
}

