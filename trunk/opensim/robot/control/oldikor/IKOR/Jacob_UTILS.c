/* ________________________________________________________
  |                                                        |
  | Program Name:   Jacob_UTILS.c                          |
  |________________________________________________________|
  |                                                        |
  | description:  Driver for GET_JACOB as well as code for |
  |     the transformation matrixes and the extraction of  |
  |     Euler Angles.                                      |
  |                                                        |
  | Procedures:                                            |
  |     GET_JACOBIAN   :                                   |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Generic Constants             */

/* ________________________________________________________
  | name:    GET_JACOBIAN /GET_JACOBIAN_ALTERED            |
  | description:  if N SPACE == 6 or 3, then the jacobian  |
  |    is a (6x7), or (6x10), or a (3x7), or (3x10) matrix.|
  |                                                        |
  |    For Obstacle Avoidance, the Jacobian must be comput-|
  |    ed at the point of intersection.  Functions in      |
  |    Jacob_UTILS.c accomplish this assuming the follow-  |
  |    ing hold:					   |
  |      1. The links should be labeled as LL[0] - LL[4]   |
  |      2. Change constants such as Link Lengths ect..    |
  |         into a data file (see ROBOT_AIRARM.dat and     |
  |         the Users Guide)                               |
  |                                                        |
  | inputs:  Initial Qarray and LL[0]-LL[4]                |
  | outputs: Returns the Jacobian                          |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     10/94    Created		           |
  |________________________________________________________| */

void GET_JACOBIAN (MATRIX *Jacob, MATRIX *Qarray)
{
  int i;
  for (i=0; i<Robot->NL; i++) LL[i] = Robot->LINKS[i];

  GET_JACOB (Jacob, Qarray);
}

void GET_JACOBIAN_ALTERED (MATRIX *Jacob, MATRIX *Qarray)
{
  GET_JACOB (Jacob, Qarray);
}


/* ________________________________________________________
  | name:                  getT2                           |
  | description:   Calculate transformation matrix for a   |
  |     given three angular rotations and three transla-   |
  |     tions.  Parameters entered in the order: z axis    |
  |     rotation, y axis rotation, x axis rotation, x axis |
  |     translation, y axis translation, z axis transla-   |
  |     tion. The following equations come from the mult-  |
  |     iplication of three separate rotation matrices and |
  |     one translation matrix (all are 4x4).              |
  |                                                        |
  | inputs:  x,y,z rot and x,y,z tran                      |
  | outputs: Returns the Transformation Matrix             |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | unkown       ?/??      Created		           |
  |________________________________________________________| */

MATRIX *getT2(double ZA,double YB,double XG, double tx,double ty,double tz)
{
double sa, sb, sg, ca, cb, cg;
MATRIX *T;

T=mat_malloc(4,4);

sa = sin(ZA); sb = sin(YB); sg = sin(XG);
ca = cos(ZA); cb = cos(YB); cg = cos(XG);

T->p[0][0]=  ca*cb;  T->p[0][1]=ca*sb*sg-sa*cg;  T->p[0][2]=ca*sb*cg+sa*sg;
T->p[1][0]=  sa*cb;  T->p[1][1]=sa*sb*sg+ca*cg;  T->p[1][2]=sa*sb*cg-ca*sg;
T->p[2][0]= -sb;     T->p[2][1]=cb*sg;           T->p[2][2]=cb*cg;
T->p[3][0]=  0.0;    T->p[3][1]=0.0;             T->p[3][2]=0.0;   
T->p[0][3]= tx;     
T->p[1][3]= ty;              
T->p[2][3]= tz;
T->p[3][3]=  1;

return (T);
}  /* getT2 */
 


/* ________________________________________________________
  | name:                  ExtractRPY2                     |
  | description: Now to extract Euler Angles from T matrix |
  |     which is in effect the homogenous transform matrix |
  |     from base to end effector.  Purpose- extracts roll,|
  |     pitch, and yaw angles from orthogonal rotation     |
  |     matrix using  J.J. Craig's algorithm(2nd ed. 1989) |
  |                                                        |
  | inputs:  transformation matrix                         |
  | outputs: Returns Euler angles in x_of_link structur    |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | unkown       ?/??      Implemented		           |
  | c hacker     3/95      incorporated new x_of_link      |
  |________________________________________________________| */

void ExtractRPY2(MATRIX * T, MATRIX *x_of_link)
{
  extern FILE *datafp;
  double SMALL_RPY = 0.01745;
  double temp1, temp2, temp3,
  	 gamma = 0, beta = 0, alpha = 0;

  temp1 = sqrt(T->p[0][0]*T->p[0][0] + T->p[1][0]*T->p[1][0]);
  temp2 =   - (T->p[2][0]);

  beta = atan2(temp2, temp1);		/* beta or attitude angle     */

  if      (fabs(beta - (PI / 2.0e0)) < SMALL) {
    gamma = ZERO;	        	/* gamma or heading angle     */
    temp1 = T -> p[1][1];
    temp2 = T -> p[0][1];
    alpha = atan2(temp2, temp1);	/* alpha or bank angle        */
  }

  else if (fabs(beta + (PI / 2.0e0)) < SMALL) {
    gamma = ZERO;	        	/* gamma or heading angle     */
    temp1 = T -> p[1][1];
    temp2 = T -> p[0][1];
    alpha = -atan2(temp2, temp1);	/* alpha or bank angle        */
  }

  else  {
    temp3 = cos(beta);
    temp2 = T -> p[1][0] / temp3;
    temp1 = T -> p[0][0] / temp3;
    gamma   = atan2(temp2, temp1);	/* gamma or heading angle  */
    temp2 = T -> p[2][1] / temp3;
    temp1 = T -> p[2][2] / temp3;
    alpha  = atan2(temp2, temp1);	/* alpha or bank angle     */
  }

  if (x_of_link->cols > 3)  x_of_link -> p[Robot->NL][3] = gamma;
  if (x_of_link->cols > 4)  x_of_link -> p[Robot->NL][4] = beta;
  if (x_of_link->cols > 5)  x_of_link -> p[Robot->NL][5] = alpha;

}			     /* ExtractRPY2 */



