#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
    //see Figure 8 in PowerPoint file
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



int main()
{
	struct plat_params_struct plat_params ;

	void solve_georedux (struct plat_params_struct *params,
					 double L_o0o1,double L_s0s1,double L_p0p1,double L_t0t1,
					 double L_q0q1,double L_r0r1,double Lsfor33[6]) ;
	void solve_platform (int *pnum_solutions,
		     double T_2_1[8][4][4],
		     double p_x_1, double q_x_1, double q_y_1,
		     double s_x_2, double t_x_2, double t_y_2,
		     double L_or, double L_os, double L_ps,
		     double L_pt, double L_qt, double L_qr) ;
	void vecmult(double ans1[4], double matrix1[4][4], double vector1[4]);
	double dist (double pt1[4], double pt2[4]) ;


    /* input items */
		/* constant platform parameters */
		plat_params.p0_x0 = 123.0 ;     // mm
		plat_params.q0_x0 = 123.0/2.0 ;  plat_params.q0_y0 = sin(60.0*D2R)*123.0 ;
		plat_params.s1_x1 =  61.5 ;
		plat_params.t1_x1 =  61.5/2.0 ;  plat_params.t1_y1 = sin(60.0*D2R)*61.5 ;
		plat_params.o0s0 =  109.0 ;
		plat_params.p0t0 =  109.0 ;
		plat_params.q0r0 =  109.0 ;
		plat_params.r1o1 =   47.5 ;
		plat_params.s1p1 =   47.5 ;
		plat_params.t1q1 =   47.5 ;

		/* leg lengths */
		double L_oo = 72.0 ;   // distance between points O0 and O1
		double L_ss = 68.0 ;   // distance between points S0 and S1, etc.
		double L_pp = 73.0 ;
		double L_tt = 67.0 ;
		double L_qq = 71.0 ;
		double L_rr = 70.5 ;

	/* output items */
		double T_1_0[8][4][4];                        // all possible poses of top platform
												      // (transformation matrix 1 to 0
		int num_solutions ;                           // number of real solutions

	/* local variables */
		double L_or, L_os, L_ps, L_pt, L_qt, L_qr; //virtual 3-3 leg lengths
		double Lsfor33[6] ;                        //virtual 3-3 leg lengths passed out of georedux function

   solve_georedux(&plat_params, L_oo,L_ss,L_pp,L_tt,L_qq,L_rr,Lsfor33);

   // Rename the lengths of the equivalent 3-3 

	L_or = Lsfor33[0] ;
	L_os = Lsfor33[1] ;
	L_ps = Lsfor33[2] ;
	L_pt = Lsfor33[3] ;
	L_qt = Lsfor33[4] ;
	L_qr = Lsfor33[5] ;

	solve_platform (&num_solutions, T_1_0, plat_params.p0_x0, plat_params.q0_x0, plat_params.q0_y0,
					  plat_params.s1_x1, plat_params.t1_x1, plat_params.t1_y1,
				   L_or, L_os, L_ps, L_pt, L_qt, L_qr) ;

	printf("num solutions = %d\n", num_solutions) ;

	FILE *fp ;
	fp = fopen("out.txt", "w") ;

	fprintf (fp, "INPUT ITEMS:\n") ;
    fprintf (fp, "\tpx0 = %8.4lf\tqx0 = %8.4lf\tqy0 = %8.4lf\n",
		plat_params.p0_x0, plat_params.q0_x0, plat_params.q0_y0) ;
    fprintf (fp, "\tsx1 = %8.4lf\ttx1= %8.4lf\tty1 = %8.4lf\n",
		plat_params.s1_x1, plat_params.t1_x1, plat_params.t1_y1) ;
	fprintf (fp, "\tdist_o0_s0 = %8.4lf\tdist_p0_t0 = %8.4lf\tdist_q0_r0 = %8.4lf\n",
		plat_params.o0s0, plat_params.p0t0, plat_params.q0r0) ;
	fprintf (fp, "\tdist_r1_o1 = %8.4lf\tdist_s1_p1 = %8.4lf\tdist_t1_q1 = %8.4lf\n",
		plat_params.r1o1, plat_params.s1p1, plat_params.t1q1) ;
	fprintf (fp, "\tLeg O length = %8.4lf\n", L_oo) ;
	fprintf (fp, "\tLeg P length = %8.4lf\n", L_pp) ;
	fprintf (fp, "\tLeg Q length = %8.4lf\n", L_qq) ;
	fprintf (fp, "\tLeg R length = %8.4lf\n", L_rr) ;
	fprintf (fp, "\tLeg S length = %8.4lf\n", L_ss) ;
	fprintf (fp, "\tLeg T length = %8.4lf\n", L_tt) ;

	fprintf (fp, "\nOUTPUTS:\n") ;
	fprintf (fp, "\t%d solutions for transformation matrix T_1_to_0\n", num_solutions) ;
	int i, j ;
	for (i=0 ; i<num_solutions ; ++i)
	   {fprintf (fp, "\nsolution %d\n", i) ;
		for (j=0 ; j<4 ; ++j)
        fprintf (fp, "%8.4lf\t%8.4lf\t%8.4lf\t%8.4lf\n", T_1_0[i][j][0], T_1_0[i][j][1], T_1_0[i][j][2], T_1_0[i][j][3]) ;
       }

	// Check the solutions.
	double pt_r1_0[4], pt_s1_0[4], pt_t1_0[4] ;   // coordinates of points R1, S1, and T1 in coord sys 0
	double pt_o1_0[4], pt_p1_0[4], pt_q1_0[4] ;   // coordinates of points O1, P1, and Q1 in coord sys 0
	double dist_oo, dist_pp, dist_qq, dist_rr, dist_ss, dist_tt ;

	double pt_r1_1[4]  = {0.0, 0.0, 0.0, 1.0} ;
	double pt_s1_1[4]  = {plat_params.s1_x1, 0.0, 0.0, 1.0} ;
	double pt_t1_1[4]  = {plat_params.t1_x1, plat_params.t1_y1, 0.0, 1.0} ;
	double pt_o1_1[4]  = {plat_params.r1o1, 0.0, 0.0, 1.0} ;
	double pt_p1_1[4]  = {plat_params.s1_x1+cos(120.0*D2R)*plat_params.s1p1, sin(120.0*D2R)*plat_params.s1p1, 0.0, 1.0} ;
	double pt_q1_1[4]  = {plat_params.t1_x1+cos(240.0*D2R)*plat_params.t1q1, plat_params.t1_y1+sin(240.0*D2R)*plat_params.t1q1, 0.0, 1.0} ;

	double pt_o0_0[4]  = {0.0, 0.0, 0.0, 1.0} ;
	double pt_p0_0[4]  = {plat_params.p0_x0, 0.0, 0.0, 1.0} ;
	double pt_q0_0[4]  = {plat_params.q0_x0, plat_params.q0_y0, 0.0, 1.0} ;
	double pt_r0_0[4]  = {plat_params.q0_x0+cos(240.0*D2R)*plat_params.q0r0, plat_params.q0_y0+sin(240.0*D2R)*plat_params.q0r0, 0.0, 1.0} ;
	double pt_s0_0[4]  = {plat_params.o0s0, 0.0, 0.0 , 1.0} ;
	double pt_t0_0[4]  = {plat_params.p0_x0+cos(120.0*D2R)*plat_params.p0t0, sin(120.0*D2R)*plat_params.p0t0 , 0.0, 1.0} ;

	fprintf (fp, "\nCheck of solutions.\n\n") ;

	for (i=0 ; i< num_solutions ; ++i)
	   {fprintf (fp, "Solution %d\n", i) ;
        vecmult(pt_r1_0, T_1_0[i], pt_r1_1) ;
        vecmult(pt_s1_0, T_1_0[i], pt_s1_1) ;
        vecmult(pt_t1_0, T_1_0[i], pt_t1_1) ;
		vecmult(pt_o1_0, T_1_0[i], pt_o1_1) ;
		vecmult(pt_p1_0, T_1_0[i], pt_p1_1) ;
		vecmult(pt_q1_0, T_1_0[i], pt_q1_1) ;

		dist_oo = dist(pt_o0_0, pt_o1_0) ;
		dist_pp = dist(pt_p0_0, pt_p1_0) ;
		dist_qq = dist(pt_q0_0, pt_q1_0) ;
		dist_rr = dist(pt_r0_0, pt_r1_0) ;
		dist_ss = dist(pt_s0_0, pt_s1_0) ;
		dist_tt = dist(pt_t0_0, pt_t1_0) ;

        fprintf (fp, "\tdist o0_o1 = %8.4lf\n", dist_oo) ;
		fprintf (fp, "\tdist p0_p1 = %8.4lf\n", dist_pp) ;
		fprintf (fp, "\tdist q0_q1 = %8.4lf\n", dist_qq) ;
		fprintf (fp, "\tdist r0_r1 = %8.4lf\n", dist_rr) ;
		fprintf (fp, "\tdist s0_s1 = %8.4lf\n", dist_ss) ;
		fprintf (fp, "\tdist t0_t1 = %8.4lf\n", dist_tt) ;
        }
	
	fclose(fp) ;
}

double dist(double pt1[4], double pt2[4])
{
	double mydist ;
	mydist = sqrt(pow(pt1[0]-pt2[0], 2) + pow(pt1[1]-pt2[1], 2) + pow(pt1[2]-pt2[2], 2)) ;
	return(mydist) ;

}
