/* ________________________________________________________
  |                                                        |
  | Program Name:   criteria.c                             |
  |________________________________________________________|
  |                                                        |
  | description: This file contains the declarations for   |
  |    different criteria, such as Least Norm, Force,      |
  |    Torque, or strength optimization.                   |
  |                                                        |
  | Procedures:                                            |
  |    Least_Norm() : H is ZERO, and B is identity.        |
  |    Least_Flow() : H is ZERO, and B is governing matrix |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Header File to link all others */

/* ________________________________________________________
  | name:    Least_Norm()                                  |
  | description:  Form Least Norm Motion, the reference    |
  |     vector Zr is zero and therefore H is zero.  The B  |
  |     vector is also zero.                               |
  |                                                        |
  | inputs:  none                                          |
  | outputs: H is the zero vector, B is the identity matrix|
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker      3/95    Created		           |
  |________________________________________________________| */

int Least_Norm (MATRIX *B, MATRIX *H, FILE *datafp)
{
  int i, j=0;
  
  
  for (i=0; i<H->rows; i++)   H->p[i][0] = 0.0;  /* this is Zr reference */
  for (j=0; j<B->cols; j++)
    for (i=0; i<B->rows; i++)
      if (i==j)
        B->p[i][j] = 1.0;
      else
        B->p[i][j] = ZERO;

  if (DEBUG) {
    fmat_pr(datafp,"Least Norm B", B);
    fmat_pr(datafp,"Least Norm H", H);
  }
}

/* ________________________________________________________
  | name:    Least_FLOW()                                  |
  | description:  Form Least Flow Motion, the reference    |
  |     vector Zr is zero and therefore H is zero.  The B  |
  |     vector is a matrix built upon the flows of the     |
  |     hydrolic pumps (currently on AIARM is implemented) |
  |                                                        |
  | inputs:  Previous Joint Angles and Previous dq         |
  | outputs: H is the zero vector, B is the govrning matrix|
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker     12/95       Created		           |
  |________________________________________________________| */

int Least_Flow (MATRIX *B, MATRIX *H, MATRIX *Qarray, FILE* datafp)
{
  int i, j;

  for (i=0; i<H->rows; i++)   H->p[i][0] = 0.0;  /* this is Zr reference */
  for (j=0; j<B->cols; j++)
    for (i=0; i<B->rows; i++)
      /* The 1.0 for dq really means don't modifiy the equations, while */
      /* the negative distance means don't generate output to datafile  */
      if (i==j) B->p[i][i] = calc_flow(i, Qarray->p[i][0], 1.0, -1.0, 
							(MATRIX *) 0);
      else      B->p[i][j] = ZERO;


  if (DEBUG) {
    fmat_pr(datafp,"Least FLOW B", B);
    fmat_pr(datafp,"Least FLOW H", H);
  }
}

/* FLOW equations for AIRARM's Hydraulic Motors */
float calc_flow(int index, float Q, float dq, float distance, MATRIX *old)
{
  static float FLOW=ZERO, totaltime = ZERO;
  float  numerator, denominator, dFLOW, time;

  dq = fabs(dq);
  switch (index) {
	case 0 :
	    Q           = 1.5479 - Q;
	    numerator   = 0.386 * sin(Q);
	    denominator = sqrt( 1.29 - .769 * cos(Q) );
	    dFLOW       = numerator / denominator;
	      if (dq<ZERO)  dFLOW *= dq * 4.28;			/*push*/
	      else          dFLOW *= dq * 2.60;			/*pull*/
	    break;
	case 1 :
	    Q           = 1.360 - (Q - PI/2);
	    numerator   = 0.574 * sin(Q);
	    denominator = sqrt( 2.05 - 1.15 * cos(Q) );
	    dFLOW       = numerator / denominator;
	      if (dq<ZERO)  dFLOW *= dq * 4.82;			/*push*/
	      else          dFLOW *= dq * 2.68;			/*pull*/
	    break;
	case 2 :
	      if (dq>ZERO)  dFLOW = dq * .838;		/*push*/
	      else          dFLOW = dq * .429;		/*pull*/
	    break;
	case 3 :
	    Q           = 0.546 - (Q); dq = -dq;
	    numerator   = 0.190 * sin(Q);  
	    denominator = sqrt( 0.82 - 0.38 * cos(Q) );
	    dFLOW       = numerator / denominator;
	      if (dq<ZERO)  dFLOW *= dq * 2.10;
	      else          dFLOW *= dq * 1.73;
	    break;
	case 4 :
	    dFLOW = .05500 * dq;
	    break;
	case 5 :
	    dFLOW = .00688 * dq;
	    break;
  }

  /* if the B Matrix is not being Calculated */
  if (distance >= ZERO) {  
    time  = distance/(60*SPEED);  /* SPEED is (.3m/s) defined in general.h */
 /*   FLOW += (fabs(dFLOW)/time);   	 /* Uncomment for flow comparisons */
    FLOW += SQUARE(fabs(dFLOW)/time);  /* Uncomment for norm comparisons */

    if (index==0) fprintf(FLWfile, "%7.5f", totaltime);

    fprintf(FLWfile, " %11.8f",       (time)?(fabs(dFLOW)/time):ZERO);
    if (DEBUG) {
      fprintf(datafp, "dFLOW[%d]: %10.4f dF/dt: %13.8f\n", index, fabs(dFLOW), 
			fabs(dFLOW)/time);
      if (index == 5) 
        fprintf(datafp, "FLOW: %f  TIME: %f  DISTANCE: %f\nFLOWRATE: %f\n",
                         FLOW, time, distance, (time)?FLOW/time:0.0);
    }
    if (index==5){
	fprintf(FLWfile, " %11.6f\n", (time)?FLOW:ZERO); 
        fprintf(distfp, "%f \n", distance);
	FLOW = 0.0;
        totaltime += time;
    }
  }

  return (fabs(dFLOW));
}
