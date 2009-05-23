/* ________________________________________________________
  |                                                        |
  | Program Name:   constraints.c                          |
  |________________________________________________________|
  |                                                        |
  | description: Determines if constraints are necessary   |
  |    for the system and if so calculates necessary betas.|
  |                                                        |
  | Procedures:                                            |
  |    avoid_limits    : finds limit breach & calls beta   |
  |    avoid_obstacles : finds obs   breach & calls beta   |
  |    find_intersection_sphere : As named                 |
  |________________________________________________________| */

#include <IKOR/general.h>      /* Header File to link all others */

/* ________________________________________________________
  | name:    avoid_limits                                  |
  | description:  Loops through all joint limits to find   |
  |     out if a joint has reached either an upper or a    |
  |     a lower limit.  If one has been reached and        |
  |     solution vectors have yet to be found, solution    |
  |     vectors are calculated and betas produced, other-  |
  |     wise, if solutions have already been found, the    |
  |     find beta function is called.                      |
  |                                                        |
  | inputs:  See Declarations below                        |
  | outputs: Calls functions to calculate betas.           |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's    |
  |  c hacker     10/94    Ported from 2 g's to FSP        |
  |                2/95    Ported to Solutions struct      |
  |________________________________________________________| */

int avoid_limits(FSP_data, Jacob, dx, got_gs, datafp)
FILE   *datafp;         /* data file declaration            */
int    *got_gs;         /* get_gs or not, used for  TWOSTEP */
MATRIX *Jacob,		/* needed to find soln's if TWOSTEP */
       *dx;		/* needed to find soln's if TWOSTEP */
Solutions *FSP_data;    /* see structures.h                 */
{
  int    qchk, ii,
         bound = FALSE;
  double distance_2_move;

  if (DEBUG) {
    fprintf(datafp,"\n  ______________________________  \n");
    fprintf(datafp,"\n      JOINT LIMIT AVOIDANCE       \n");
    fprintf(datafp,  "  ______________________________  \n\n");
  }
  for (qchk = 0; qchk < M; qchk++)
  {
              /*  limit_upper... */
    if  (rad(Robot->Angles[qchk].Min_limit) > FSP_data->Qarray->p[qchk][0])
    {	
      bound = TRUE;
      distance_2_move = 
        rad(Robot->Angles[qchk].Min_limit) - FSP_data->Qarray->p[qchk][0];
    }			     /*  limit_upper... */

    else      /*  limit_lower... */
    if (rad(Robot->Angles[qchk].Max_limit) < FSP_data->Qarray->p[qchk][0])
    {
      bound = TRUE;
      distance_2_move = 
	rad(Robot->Angles[qchk].Max_limit) - FSP_data->Qarray->p[qchk][0];
    }			     /*  limit_lower... */

  if (bound)
    if ( (  Robot->Angles[qchk].Prism  ) ||
	 ((!Robot->Angles[qchk].Prism)&&(fabs(distance_2_move)>.00001))   )
    {
      bound = FALSE;
      if ( (STEP == TWO) && (!(*got_gs)) ) 
      {
        GET_JACOBIAN(Jacob, FSP_data->Qarray);
        Solution_generator ( FSP_data, Jacob, dx, datafp ); 
        *got_gs = TRUE;
      }

      #ifdef DEBUG_OUT
        fprintf(stderr, "   joint(1-%d) #%i limited  ", M, qchk+1);
      #endif
      if (DEBUG) {
        fprintf(datafp, "\n\njoint # %i out of range by %lf ",
  	        qchk+1, deg(distance_2_move));
      }
      find_jl_beta(FSP_data, qchk, distance_2_move, datafp);
    }				/* end if(bound)	*/
  }			/* end for(qchk...)	*/
}		/* end avoid_limits()	*/

/* ________________________________________________________
  | name:    avoid_obstacles                               |
  | description:  Loops through all obstacles and calls    |
  |     routines to determine intersection. If an obstacle |
  |     has been breached and solution vectors have yet to |
  |     be found, solution vectors are calculated and      |
  |     betas are produced, otherwise, if solutions have   |
  |     already been found, only the find beta function is |
  |     called.                                            |
  |                                                        |
  | inputs:  See Declarations below                        |
  | outputs: Calls Function to Calculate Betas             |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's    |
  |  c hacker     10/94    Ported from 2 g's to FSP        |
  |                2/95    Ported to Solutions struct      |
  |________________________________________________________|*/

int avoid_obstacles(FSP_data, Jacob, dx, x_of_link, ns, spheredata, 
		    got_gs, datafp)
int     ns;  	                /* circ info (# of circles) */
double  spheredata[4][4];  	/* circ info (rad, center)  */
FILE   *datafp;         /* data file declaration            */
int    *got_gs;         /* get_gs or not, used for  TWOSTEP */
MATRIX *Jacob,		/* needed to find soln's if TWOSTEP */
       *dx,		/* needed to find soln's if TWOSTEP */
       *x_of_link;  	/* link endpts                      */
Solutions *FSP_data;    /* see structures.h                 */
{
  int    i, kk, is, ii, elbow_check=0;
  double newl, delta,
	 pt1[3], pt2[3], Xj[3], normal[3];

  if (STEP == TWO) GET_ACTUAL_X ( FSP_data->Qarray, x_of_link );	

  if (DEBUG) {
    fprintf(datafp,"\n  ______________________________  \n");
    fprintf(datafp,"\n         OBSTACLE AVOIDANCE       \n");
    fprintf(datafp,  "  ______________________________  \n\n");
    fmat_pr(datafp," Position of Each Link ", x_of_link);
  }

  for (kk = 0; kk < Robot->NL; kk++)
  {
    for (i=0; i<3; i++)
    {
      pt1[i] = x_of_link->p[kk][i];      
      pt2[i] = x_of_link->p[kk+1][i];
    }
    if (DEBUG) {
      fprintf (datafp,"\nChecking LINK(0-4) %d\n", kk);
      fprintf (datafp," pt1 = < %f, %f, %f >\n pt2 = < %f, %f, %f >",
		pt1[0],pt1[1],pt1[2],pt2[0],pt2[1],pt2[2]);
    }

    for (is = 0; is < ns; is++)
    {
      if (find_intersection_sphere(pt1, pt2, is, ns, spheredata,
		 &elbow_check, &newl, &delta, normal, datafp))
      {
	if ( (STEP == TWO) && (!(*got_gs)) ) 
        {
          GET_JACOBIAN(Jacob, FSP_data->Qarray);
          Solution_generator( FSP_data, Jacob, dx, datafp ); 
          *got_gs = TRUE;
	}
	for (i=0; i<3; i++)
          Xj[i]= x_of_link->p[kk][i] + newl*x_of_link->p[kk+1][i];


        #ifdef DEBUG_OUT
          fprintf (stderr,"   LINK %d COLLIDED\n", kk);
        #endif
	if (DEBUG) fprintf (datafp,"\tXj  = <   %4.6f,  %4.6f,  %4.6f   >\n",
		  				Xj[0],  Xj[1],  Xj[2]);

	find_obs_beta(FSP_data, kk, &newl, delta, normal, datafp);

      }				/* end if (intersection) */
    }			/* end for(is=0 to ns)  */
  }		/* end kk loop */
}	/* end avoid_obstacles() */



/* ________________________________________________________
  | name:    find_intersection_sphere                      |
  | description:  For a given obstacle and link, determines|
  |     if that link is within the obstacles buffer zone.  |
  |     If so, returns closest point on the link to the    |
  |     center of the object, the normal vector from the   |
  |     center to that point, and the distance needed in   |
  |     to move said point to the edge of the buffer zone. |
  |     This information is critical to the formation of   |
  |     of beta values.                                    |
  |                                                        |
  | inputs:  link (pt1,pt2), sphere                        |
  | outputs: newl, delta, normal                           |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's    |
  |  c hacker      2/95    Ported to Solutions struct      |
  |________________________________________________________|*/
 
int find_intersection_sphere(pt1, pt2, ws, ns, spheredata,
	 elbow_check, newl, delta, normal, datafp)
FILE   *datafp;          /* data file declaration         */
int     ws;              /* which sphere                  */
int    *elbow_check;     /* check if elbow of link inside */
double  normal[3],       /* normal from center of sphere  */
       *newl,            /* closest pnt j to cntr of obs  */
       *delta,           /* distance pnt j is in boundary */
        pt1[3], pt2[3];  /* endpoints of link             */
int     ns;		       /* sphere info (# spheres) */
double  spheredata[4][4];      /* sphere info (rad, cntr) */
{
  int    i, j;
   
  double x_temp,  y_temp,  z_temp,
  	 x_mid,   y_mid,   z_mid,
	 xcenter, ycenter, zcenter, radius, 
         deltax,  deltay,  deltaz,
  	 b_check1, b_check2, root_chk,
  	 coeff_t_sq, coeff_t_f,
  	 t_sol_init, t_sol1, t_sol2,
  	 xsol1, xsol2,  ysol1, ysol2,  zsol1, zsol2,
  	 constant1, constant2, constant3, constantf1, constantf;

  xsol1 =ysol1 =zsol1 =x_temp =y_temp =z_temp =0.0;

  xcenter = spheredata[ws][0];
  ycenter = spheredata[ws][1];
  zcenter = spheredata[ws][2];
  radius  = spheredata[ws][3];

  /* defining constants needed in forming equations */
  deltax  = pt2[0] - pt1[0];
  deltay  = pt2[1] - pt1[1];
  deltaz  = pt2[2] - pt1[2];

  coeff_t_sq = (deltax * deltax) + (deltay * deltay) + (deltaz * deltaz);


  coeff_t_f = 2 * ( deltax * (pt1[0] - xcenter) +
		    deltay * (pt1[1] - ycenter) +
 		    deltaz * (pt1[2] - zcenter) );

  constant1  = pt1[0] - xcenter;
  constant2  = pt1[1] - ycenter;
  constant3  = pt1[2] - zcenter;
  constantf1 =  (constant1 * constant1) + 
		(constant2 * constant2) + 
		(constant3 * constant3);

  constantf = constantf1 - SQUARE(radius);


  /* now that coefficients of the quadratric are defined, we solve it */

  b_check1 = SQUARE(coeff_t_f);
  b_check2 = coeff_t_sq * constantf * 4;
  root_chk = b_check1 - b_check2;

 	 /* now to ascertain what situation we have */
  if (root_chk <  0.0) return FALSE;	     /* no intersection */
  if (root_chk == 0.0) return FALSE;	     /* graze */


	 /* root_chk > 0.0 intersection occurs */
  t_sol_init = sqrt(root_chk);
  t_sol1 = (-(coeff_t_f) + t_sol_init) / (2 * coeff_t_sq);
  t_sol2 = (-(coeff_t_f) - t_sol_init) / (2 * coeff_t_sq);

  if (DEBUG) fprintf(datafp,"\tt_sol1: %4.2f t_sol2: %4.2f \n", t_sol1, t_sol2);

  /************      now check t's for validity      ************/

	     /* Does pt of inters. lie on link? */
  if (((0.0 > t_sol1) || (t_sol1 > 1.0)) && 
      ((0.0 > t_sol2) || (t_sol2 > 1.0))) 
  {  
    if (DEBUG) fprintf(datafp,"\tIntersection not on link \n");
    return FALSE;
  }

	     /* Does pt of inters. elbow sit. */
  if (((0.0 > t_sol1) || (t_sol1 > 1.0)) || 
      ((0.0 > t_sol2) || (t_sol2 > 1.0)))
  {
    if (DEBUG) fprintf(datafp, "\tElbow Intersection!\n");
   *elbow_check = *elbow_check + 1;

    if (0.0 <= t_sol1 <= 1.0)
    {
      xsol1 = pt1[0] + (deltax * t_sol1);
      ysol1 = pt1[1] + (deltay * t_sol1);
      zsol1 = pt1[2] + (deltaz * t_sol1);
    }

    if (0.0 <= t_sol2 <= 1.0)
    {
      xsol1 = pt1[0] + (deltax * t_sol2);
      ysol1 = pt1[1] + (deltay * t_sol2);
      zsol1 = pt1[2] + (deltaz * t_sol2);
    }

    if (*elbow_check == 1)
    {
      x_temp = xsol1;	     /* end of link becomes point j */
      y_temp = ysol1;
      z_temp = zsol1;
      return FALSE;
    }
    else if (*elbow_check % 2 == 0)
    {
      x_mid = (xsol1 + x_temp) / 2;
      y_mid = (ysol1 + y_temp) / 2;
      z_mid = (zsol1 + z_temp) / 2;

      *delta = radius - sqrt(((zcenter - z_mid) * (zcenter - z_mid)) 
		           + ((ycenter - y_mid) * (ycenter - y_mid)) 
		           + ((xcenter - x_mid) * (xcenter - x_mid)));

      normal[0] = x_mid - xcenter;
      normal[1] = y_mid - ycenter;
      normal[2] = z_mid - zcenter;

      *newl = 0.0;
    }/* end if 2nd inter.    */
  }  /* end elbow sit.       */
  else			     /* there are two inter. */
  {
    xsol1 = pt1[0] + (deltax * t_sol1);
    ysol1 = pt1[1] + (deltay * t_sol1);
    zsol1 = pt1[2] + (deltaz * t_sol1);

    xsol2 = pt1[0] + (deltax * t_sol2);
    ysol2 = pt1[1] + (deltay * t_sol2);
    zsol2 = pt1[2] + (deltaz * t_sol2);

    x_mid = (xsol1 + xsol2) / 2;
    y_mid = (ysol1 + ysol2) / 2;
    z_mid = (zsol1 + zsol2) / 2;

    *delta = radius - sqrt(((zcenter - z_mid) * (zcenter - z_mid)) 
			 + ((ycenter - y_mid) * (ycenter - y_mid)) 
			 + ((xcenter - x_mid) * (xcenter - x_mid)));

    normal[0] = x_mid - xcenter;
    normal[1] = y_mid - ycenter;
    normal[2] = z_mid - zcenter;

    *newl = sqrt((z_mid - pt1[2]) * (z_mid - pt1[2]) 
	       + (y_mid - pt1[1]) * (y_mid - pt1[1]) 
	       + (x_mid - pt1[0]) * (x_mid - pt1[0]));
  } /* end 2 inter. */

  if (DEBUG) {
    fprintf (datafp, "\n\tCollision with #%d circle has occured!", ws);
    fprintf (datafp, "\n\tnormal = <%lf,%lf,%lf>", normal[0], 
  					           normal[1], 
					           normal[2]);
    fprintf (datafp, "\n\tCircle Delta = %lf\n", *delta);
  }

  return  TRUE;
}			     /* end find inter sphere */

