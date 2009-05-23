/* ________________________________________________________
  |                                                        |
  | Program Name:   IKOR_driver.c                          |
  |________________________________________________________|
  |                                                        |
  | description:  Driver for FSP, Psuedo Inverse, Manipul- |
  |     ator porting, obstacle detection, and utilities.   |
  |     IKOR_driver does not contain a main, instead it can|
  |     be called by the main in short.c or CheezyIGRIP.c  |
  |     or any other.                                      |
  |                                                        |
  | Procedures:                                            |
  |     IKOR_driver    :                                   |
  |     CALC_PSEUDO    :                                   |
  |     FSP	       :				   |
  |     Euler_to_Velocities :				   |
  |     correct_euler  :				   |
  |     OPEN_FILES     :                                   |
  |     Update_History :				   |
  |	Init_Globals   :				   |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Generic Constants               */
#include <IKOR/Globals.h>       /* Global Variables used by system */

#define DEBUG_OUT 1

/* ________________________________________________________
  | name:    IKOR_driver                                   |
  | description:  Drives entire system, although main is   |
  |     not here (see short.c or CheezyIGRIP.c for main).  |
  |     Initializes global variables, reads in manipulator |
  |     trajectory files, calls Jacobian, FSP, and con-    |
  |     straint avoidance routines, if wanted. Also allows |
  |     for Moore-Penrose Pseudo Inverse solutions, however|
  |     the Pseudo is not equipped for constraint avoidance|
  |                                                        |
  | inputs:  N, M, method.  Manipulator, trajectory        |
  | outputs: Alters two variables within FSP_data.         |
  |     the beta vector for the joint is added to betall,  |
  |     and cn, the number of beta vectors present, is     |
  |     incremented.                                       |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's    |
  |  c hacker     10/94    Ported from 2 g's to FSP        |
  |                2/95    Ported to Solutions struct      |
  |                3/95    Altered FSP, Pseudo calls       |
  |________________________________________________________| */

int IKOR_driver(IKCriterion, IKMethod, Optimization, N_use, M_use)
int *IKCriterion,    /* Psuedo-Inverse, FSPleastnorm or leastflow  */
    *IKMethod,	     /* Constraints : jointlimits, obstacles...    */
     Optimization,   /* Lagrangian, BANGBANG, SIMPLEX 		   */
     M_use,          /* Size of Joint Space (!> num angles)        */
     N_use;          /* Size of Task Space (x,y,z,yw,ptch,rll)     */
{
  int    i, j, k, 			/* Generic counters               */
	 ns,                            /* Number of Spheres              */
	 STEP, 				/* Current Step Along Trajectory  */
         Num_Of_Pts,       		/* Number of points in Trajectory */
	 quit = 0;			/* Exit_Status:  -2 File Trouble  */
					/*               -1 Partial Traj  */
  float  distance;			/* Determines distance traveled   */
  double XTraj[MAX_PTS][6],             /* End Effector (EE) Coordinates  */
	 spheredata[4][4];		/* Holds sphere info		  */
  MATRIX *Jacob,                        /* Holds Forward Kinematic Deriv. */
         *dx,                           /* Holds change in "EE"           */
         *dq, 	                        /* Holds change in ANGLES         */
	 *x_of_link,    		/* Holds end points of each link  */
	 *Pos_error;    		/* Holds the error in position    */
  Solutions *FSP_data;			/* see structure.h for explanation*/
  History   Old_DQs;                    /* Contains Array of History      */

 /*--------------------------------------------------------------*
   * GLOBAL and LOCAL INITIALIZATIONS  (Order of decls important) *
   *--------------------------------------------------------------*/
  N=N_use;
  M=M_use;
  Old_DQs.whereami = -1;
  for (i=0; i<HIST_SIZE; i++)
     Old_DQs.dq[i].DQ = mat_malloc( M , 1 );
  Pos_error = mat_malloc(N, 1); /* memory for error  in position  */
  dx        = mat_malloc(N, 1); /* memory for change in position  */
  Jacob     = mat_malloc(N, M); /* memory for Forw.Kin. derivatve */
                                /* memory for x,y,z of each link  */
  x_of_link=mat_malloc(Robot->NL+1,N);
  FSP_data =Solutions_init(M,N);/* memory for solutions,betas, etc*/
                                /* initailize trajectory, data... */
  if (!(Num_Of_Pts=OPEN_FILES(XTraj, PoorMansFile, FSP_data->Qarray)))
	 return (-2);
  ns=spheres(spheredata,datafp);/* initialize obstacles in system */ 
  dq = mat_malloc(M,1);
  Update_History(&Old_DQs, dq);
  mat_free(dq);
  /* correct size of the Weight MATRIX, see structures.h,        */
  Robot->Weights->rows = Robot->Weights->cols = M;

 
  /*--------------------------------------------------------------*
   * Uncomment, to trace bus errors or segmentation faults        *
   *--------------------------------------------------------------*/
  /*datafp = stderr;            /* When in trouble, uncomment     */


  /*-------------------------------------------------------------*
   *       HEADER INFORMATION for Standard Output                *
   *-------------------------------------------------------------*/

  #ifdef DEBUG_OUT
    fprintf (stderr, "\n\n\n"
                   " *********************************************\n"
                   "   INVERSE KINEMATIC AND REDUNDANCY RESOLVER  \n"
                   "   For HERMIES CESARm, %d D.O.F, %d D.O.R.    \n"
                   " *********************************************", M,M-N);
    fprintf (stderr, "\n\n\n"
		   "     < NUMBER OF POINTS IN TRAJECTORY >  %d  \n"
		   "   	     << Time Step >>                     \n"
		   "          ---------------------              \n"
		   "\n<< -1>>", Num_Of_Pts);
  #endif
  
  /*----------------------------------------------------------------*
   *                          MAIN-LOOP                             *  
   *                                                                *
   *    Cycles through all points on trajectory until done doing    *
   *    Least Norm  and possibly Joint and Obstacle Avoidance until *
   *    an unavoidable problem occurs.                              *
   *----------------------------------------------------------------*/

  for (STEP = 0; STEP < Num_Of_Pts ; STEP++)
  { /*main_loop*/

    /*-------------------------------------------------------*
     * Print Angles for simulation programs (IGRIP, Cheeze)  *
     *    and print step number to the data and stderr files *
     *-------------------------------------------------------*/

    for (i=0;i< Robot->NA;i++)
      if (i<M) fprintf (CheezyFile, "%f  ", FSP_data->Qarray->p[i][0]);
    fprintf (CheezyFile, "  \n");
    fflush  (CheezyFile);

    if (DEBUG) 
      fprintf (datafp, "\n*************STEP <%d> ****************\n", STEP+1);
    #ifdef DEBUG_OUT
      fprintf (stderr, "\r<<%3d>>", STEP+1);
    #endif

    /*-------------------------------------------------------*
     * Based on Angles, Calculates Position of End Effector  *
     *-------------------------------------------------------*/
    #ifndef FSP_DEBUG
      GET_JACOBIAN(Jacob, FSP_data->Qarray);
      GET_ACTUAL_X(FSP_data->Qarray, x_of_link);

      if (DEBUG) {
	fmat_pr (datafp, "Jacobian for Current Configuration", Jacob);
        fprintf (datafp, "\nCurrent Position\n"    );
        for (i = 0; i <  N; i++)
          fprintf(datafp,"%s : %9f\n", dxLabel[i], x_of_link->p[Robot->NL][i]);
        fprintf (datafp, "Next Point in Trajectory\n");
      }

      /*----------------------GET DX'S-------------------------*
       *  Based on actual position, Calculates Change in X nec *
       *  This feedback loop is necessary because this system  *
       *  is highly nonlinear.                                 *
       *-------------------------------------------------------*/
      for (i = 0; i <  N; i++)
      {
        if (DEBUG) fprintf(datafp,"%s : %9f\n", dxLabel[i], XTraj[STEP][i]);	
        dx -> p[i][0] = XTraj[STEP][i] - x_of_link->p[Robot->NL][i];
        Pos_error->p[i][0]=XTraj[STEP?STEP-1:0][i] - x_of_link->p[Robot->NL][i];
        if ( fabs( dx->p[i][0] ) < SMALL ) dx->p[i][0]=ZERO;
        if (( i < 3) && ( fabs(dx->p[i][0]) > BIG) ) IKerror(26,OK,dxLabel[i]);
      }			     

      if (DEBUG) fmat_prf(datafp,"Pos Error (Cur Pos - Prev Target)",Pos_error);

      /*--------------------------------------------------------*
       * CONVERT INCREMENTAL EULER ANGLES INTO INCREMENTAL      *
       * ANGLES ABOUT EACH AXIS OF BASE FRAME                   *
       *--------------------------------------------------------*/

      if (Robot->Orient)
      {
	/* fix the 180 == -180 problem for yaw and roll, see correct_euler() */
	dx->p[3][0] =
	correct_euler( dx->p[3][0],x_of_link->p[Robot->NL][3],XTraj[STEP+1][3]);
	dx->p[5][0] =
	correct_euler( dx->p[5][0],x_of_link->p[Robot->NL][5],XTraj[STEP+1][5]);

 	Euler_to_Velocities (x_of_link, dx);
	if (Robot->Orient == ZEROD_OMEGAS) 
	  dx->p[3][0]=dx->p[4][0]=dx->p[5][0]=ZERO;
      }
    #else
      GetData(Jacob, dx);
      STEP = Num_Of_Pts;
    #endif
    
    /*-------------------------------------------------------*
     * CALCULATE dq'S USING ALGORITHM Selected by IKMethod   *
     *-------------------------------------------------------*/

    FSP_data->cn = 0;      /* reintialize num of constraints for each STEP */
    if (IKCriterion[0] != 1) 
      dq = FSP (FSP_data, &Old_DQs, Jacob, dx, IKCriterion, IKMethod, 
	 		Optimization, ns, spheredata, x_of_link, datafp);
    else
      dq = CALC_PSEUDO (Jacob, dx);
    if (dq== (MATRIX *) -1) break;
 
    /*-----------------------------------------------------------*
     *  Determine Distance Traveled				 *
     *-----------------------------------------------------------*/

    /*-----------------------------------------------------------*
     *   Now add the necessary changes in angles to the current  *
     *   Angles and keep values within 4 PI                      *
     *-----------------------------------------------------------*/
    for (i = 0; i < M; i++)
    {
      if ((IKCriterion[0]!=1)&&(fabs(FSP_data->Xelim->p[i][0]) > SMALL))
        dq->p[i][0] = FSP_data->Xelim->p[i][0];

      FSP_data->Qarray->p[i][0] += dq-> p[i][0];

      if (!Robot->Angles[i].Prism)
      {
        if (FSP_data->Qarray->p[i][0] >  2*PI) 
	    FSP_data->Qarray->p[i][0] -= 2*PI;
        if (FSP_data->Qarray->p[i][0] < -2*PI) 
	    FSP_data->Qarray->p[i][0] += 2*PI;
      }		/* end if ! prism */
    }	/* end for i=1 to M */

    mat_mul(Jacob,dq,dx);
    distance = ZERO;
    for (i = 0; i<3; i++) distance += dx->p[i][0] * dx->p[i][0];
      distance = sqrt (distance);
    for (i = 0; i < M; i++) {
      fprintf(dqfile, "%10.7f", dq->p[i][0]);
      /* specific calculations for hydrolic action of AIRARM joints */
      /*  calc_flow(i, FSP_data->Qarray->p[i][0], dq->p[i][0], 
	    distance, (MATRIX *) 0); 
      */
    }
    fprintf(dqfile,"\n");

    Update_History(&Old_DQs, dq);

    if (DEBUG) {
      fmat_prf(datafp, "dq (radians)", dq);
      fmat_prf(datafp, "resulting dx", dx); 
      fmat_prf(datafp, "Qarray (radians)", FSP_data->Qarray);
    }

    #ifdef DEBUG_OUT
    /*  if ((STEP%20)==1) /* uncomment for shorter output */
      fprint_norm( NormFile, dq, datafp );
    /*  fprint_norm( ErrorFile, Pos_error, datafp ); */
    #endif
    mat_free(dq);           /* free up dq for next step or end  */

    if (quit < 0) break;    /* If unavoidable trouble quit      */
  }  /* END MAIN FOR LOOP*/



  /*-------------------------------------------------------------*
   *                         FINAL SUMMARY	                 *
   *-------------------------------------------------------------*/
  fprintf (datafp, "\n\n ----FINAL SUMMARY------\n"
                       " --- Exit Status: %d ---\n"
                       " Errors in final point:  \n", quit);

  fprintf (datafp, "\nStarting Coordinates:\n");
  for (i = 0; i < N; i++)
    fprintf (datafp, "Element [%d] >%f\n", i, XTraj[0][i]);

  fprintf (datafp, "\nRequested Ending Coordinates\n");
  for (i = 0; i < N; i++)
    fprintf (datafp, "Element [%d] >%f\n", i, XTraj[Num_Of_Pts-1][i]);

  fmat_pr (datafp, "Actual Ending Coordinates:", x_of_link);

  #ifdef DEBUG_OUT
    fprintf (stderr,"\n DATA FILE %s HAS BEEN CREATED AND CONTAINS\n"
                      " THE ANGLES FOR THE MANIPULATOR FOR EACH TIME  \n"
                      " STEP.  EXAMINE, OR RUN IT THROUGH A SIMULATOR.\n", 
  			PoorMansFile);
  #endif

  /*------------------------------------------------------------------*
   * Clean up Global variables and Close Files        	 	      *
   *------------------------------------------------------------------*/
  fclose(ErrorFile);
  fclose(NormFile);
  fclose(CheezyFile);
  fclose(datafp);

  Solutions_free(FSP_data);
  mat_free(Pos_error);
  mat_free(x_of_link);
  mat_free(Jacob);
  mat_free(dx);
  for (i=0; i<HIST_SIZE; i++)
    mat_free(Old_DQs.dq[i].DQ);
  free (LL);

  return (quit);
}			     /* END IKOR_driver */


/* ________________________________________________________
  | name:    Euler_to_Velocities                           |
  | description:  Converts Euler Angle increments produced |
  |       by the forward kinematics into angular velocites |
  |       about each of the major axes (X, Y, and Z).  The |
  |       conversion matrix can be found in Robotics: Con- |
  |       trol, Sensing, Vision, and Intelligence p235.    |
  |                                                        |
  | inputs:  Euler Angles held in x_of_link, Requested     |
  |       change in Euler Angles held in dx.               |
  | outputs: Returns requested change in velocites held in |
  |       the dx structure.                                |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     5/95     implemented                     |
  |________________________________________________________| */

void Euler_to_Velocities (MATRIX *x_of_link, MATRIX *dx)
{
  float  dGAMMA, gamma,                 /* Euler Angles with deltas       */
	 dBETA,  beta,                  /* Euler Angles with deltas       */
	 dALPHA, alpha;                 /* Euler Angles with deltas       */
	 

  gamma    = x_of_link->p[Robot->NL][3];      
  dGAMMA   = dx->p[3][0];

  beta     = x_of_link->p[Robot->NL][4];
  dBETA    = dx->p[4][0];

  alpha    = x_of_link->p[Robot->NL][5];
  dALPHA   = dx->p[5][0];

  /* wx */  dx->p[3][0]= dALPHA*cos(gamma)*cos(beta) - dBETA*sin(gamma); 
  /* wy */  dx->p[4][0]= dALPHA*sin(gamma)*cos(beta) + dBETA*cos(gamma);
  /* wz */  dx->p[5][0]=-dALPHA*sin(beta)            + dGAMMA;

  if (DEBUG) {
    fprintf(datafp,"\nEuler Velocity Transforms and Corrections\n");
    fprintf(datafp,"\n%s%12.8f%s%12.8f\n%s%12.8f%s%12.8f\n%s%12.8f%s%12.8f\n",
                   "YAW  :  gamma    : ", gamma  , "   dGAMMA   : ", dGAMMA,
		   "PITCH:  beta     : ", beta   , "   dBETA    : ", dBETA,
		   "ROLL :  alpha    : ", alpha  , "   dALPHA   : ", dALPHA );
    fprintf(datafp,"dx(wx) : %12.8f\ndx(wy) : %12.8f\ndx(wz) : %12.8f\n",
		   (dx->p[3][0]), (dx->p[4][0]), (dx->p[5][0]) );
  }
}

/* ________________________________________________________
  | name:    correct_euler                                 |
  | description:  Because yaw and pitch are equivalent     |
  |       when the angle = 180 and -180 degrees, a dx      |
  |       that is calculated at this edge can be erroneous.|
  |       That is, -180 - 180 = -360 when the only requir- |
  |       required change is really 0.  This code corrects |
  |       for this orientation situation.                  |
  |                                                        |
  | inputs:  yaw or roll, current position, and destina-   |
  |       tion position.                                   |
  | outputs: Returns corrected change in velocites.        |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     5/95     implemented                     |
  |________________________________________________________| */

float correct_euler ( float angle, float curr, float dest )
{

  /* check if faulty condition exists */
  if ( fabs( angle ) > PI )
  {
    if (DEBUG) fprintf(datafp,"\nFaulty orientation velocity, old: %f", angle); 

    angle = PI - fabs(dest) + PI - fabs(curr); 
    /* correct for sign */
    if (( curr < 0 ) && ( dest > 0 ))
       angle = -angle;

    if (DEBUG) fprintf(datafp, "... new: %f\n", angle);
  }
  /* return corrected or non-corrected angle */
  return (angle);
}


/* ________________________________________________________
  | name:    CALC_PSEUDO                                   |
  | description:  Calculate Change in Angles using Pseudo  |
  |     Inverse. The procedure prepares the Jacobian and   |
  |     calls the routine mat_pseudoinv() found in the     |
  |     file matrix.c                                      |
  |                                                        |
  | inputs:  Jacobian and change in x                      |
  | outputs: Returns dq necessary to complete motion.      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch    4/94     Algorithm Creation for 2 g's    |
  |  c hacker     3/95     rearranged calling structure    |
  |               3/95     took struct from "struct MATRIX"|
  |________________________________________________________| */

MATRIX *CALC_PSEUDO (MATRIX *Jacob, MATRIX *dx)
{
  int    i, j;           /* counter variables                */
  MATRIX *Jacobinv,	 /* Holds Jacobian Psuedo-inverse    */
         *dq;            /* Holds delta angle changes needed */

  Jacobinv = mat_malloc(M,M);

  mat_cp(Jacob, Jacobinv);

  for (i=0;i<M;i++)      /* Must be padded with zeros        */
    for (j=N; j< M; j++)
      Jacobinv->p[j][i] = 0.0;

  mat_pseudoinv(Jacobinv); /* JacobTemp holds jacob's inverse */

  Jacobinv->cols = N;
  dq = mat_mul2(Jacobinv, dx);
  Jacobinv->cols = M;

  if (DEBUG) {
     fmat_pr(datafp,"*** Jacobian ***",  Jacob);
     fmat_prf(datafp,"requested dx",     dx);
  }

  mat_free(Jacobinv);
  return (dq);	
}


/* ________________________________________________________
  | name:    FSP                                           |
  | description:  This procedure calculates the changes in |
  |     angles via the new algorithm (FSP). It also calls  |
  |     routines to check for obstacles, limits, ect...    |
  |     Finally, a procedure to find a particular para-    |
  |     metric solution based on constraints, criteria.    |
  |     The reason for several find_t functions is due to  |
  |     speed considerations, so that, the find_t that gets|
  |     called should be the fastest.                      |
  |                                                        |
  | inputs:  Jacobian, change in x, IKmethod, sphere data, |
  |     and position.                                      |
  | outputs: Returns dq necessary to complete motion.      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch    4/94     Algorithm Creation for 2 g's    |
  |  c hacker     3/95     rearranged calling structure    |
  |               3/95     took struct from "struct MATRIX"|
  |               3/95     converted to ONE STEP Method    |
  |________________________________________________________| */

MATRIX *FSP(FSP_data, Old_DQs, Jacob, dx, IKCriterion, IKMethod, Optimization,
		ns, spheredata, x_of_link, datafp)
FILE   *datafp;	           /* data file declaration          */
int     *IKCriterion,	   /* Criterion		             */
	*IKMethod,	   /* 0-JL,2-OBS ... contraints      */
	 Optimization,	   /* LAGRANGE,BANGBANG,SIMPLEX	     */
	 ns;               /* sphere_data (# of spheres)     */
double  spheredata[4][4];  /* sphere_data (rad,x,y,z)        */
MATRIX *dx, 	           /* change in EE position          */
       *Jacob,             /* derivative of Forward Kinematic*/
       *x_of_link;         /* position of all links          */
Solutions *FSP_data;       /* see structures.h               */
History *Old_DQs;
{
  int     i, j, k,     /* counter variables                   */
	  got_gs=TRUE, /* For TWOSTEP, flag if 2nd g's made   */
	  Terminate;   /* Termination Status of FSP           */
  MATRIX *dq,          /* final solution for joint space move */
         *B,
         *H;

  Terminate = Solution_generator(FSP_data, Jacob, dx, datafp); 
  if (Terminate == NOT_COMPLETE) IKerror(10, FATAL, "");
  B = mat_malloc (  M,    M );
  H = mat_malloc (  SPAN, 1 );

  switch (IKCriterion[0]) {
     case 0: Least_Norm (B, H, datafp); break;
     case 1:  /* This is Least Norm and Should Not Be Called Here */
     case 2:  /* MinDx   ();  (implemented in IKORv2.6, not here) */
     case 3:  /* MinDist ();  (implemented in IKORv2.6, not here) */
     case 4: Least_Flow (B, H, FSP_data->Qarray, datafp);  break;
     default: fprintf(stderr, "Requested criterion does not exist! \n");
              break;
  }

  if (Optimization == BANGBANG)
  {
    if (IKCriterion[0] == 4)
    {
      dq = findt_without_Betas_BANGBANG(FSP_data, B, H,
		 Old_DQs->dq[Old_DQs->whereami].DQ, datafp);
    }
    else IKerror(31, FATAL, "BANGBANG only for Flow Currently\n");
  }
  else if (Optimization == LAGRANGIAN)
  {
    if (STEP == ONE)
    {
      /*************************************************************
       * CONSTRAINT TESTS BRANCH                                   *
       *************************************************************/
      if (IKMethod[OBSTACLES])
        avoid_obstacles (FSP_data, Jacob, dx, x_of_link,
			 ns, spheredata, &got_gs, datafp);
      if (IKMethod[JN_LIMITS])
        avoid_limits    (FSP_data, Jacob, dx, &got_gs, datafp);
    }

    if (Robot->PLAT.Holonomic)
      if (FSP_data->cn)
        dq = findt_with_Betas_Holonomic(FSP_data, B, H, datafp); 
      else
        dq = findt_without_Betas_Holonomic(FSP_data, B, H, datafp); 
    else /* Non-holonomic Platform Code */
       if (FSP_data->cn)
	 dq = findt_with_Betas_Nonholonomic(FSP_data, B, H, datafp);
       else
	 dq = findt_without_Betas_Nonholonomic(FSP_data, B, H, datafp); 


    if (STEP == TWO)
    {
      /*************************************************************
       *   Now add the necessary changes in angles to the current  *
       *   Angles and keep values withing 4 PI                     *
       *************************************************************/
      got_gs = FALSE;

      for (i = 0; i < M; i++)
      {
        if (fabs(FSP_data->Xelim->p[i][0]) > SMALL)
           dq->p[i][0] = FSP_data->Xelim->p[i][0];

        FSP_data->Qarray->p[i][0] += dq-> p[i][0];

        if (!Robot->Angles[i].Prism)
        {
          if (FSP_data->Qarray->p[i][0] >  2*PI) 
  	      FSP_data->Qarray->p[i][0] -= 2*PI;
          if (FSP_data->Qarray->p[i][0] < -2*PI) 
	      FSP_data->Qarray->p[i][0] += 2*PI;
        }	/* end if ! prism */
	dq -> p[i][0] = ZERO;	/* Reset dq's in case of no constraints */
      }	  /* end for i=1 to M */


      /*************************************************************
       * CONSTRAINT TESTS BRANCH                                   *
       *************************************************************/
      if (IKMethod[OBSTACLES])
        avoid_obstacles (FSP_data, Jacob, dx, x_of_link, 
			 ns, spheredata, &got_gs, datafp);
      if (IKMethod[JN_LIMITS])
        avoid_limits    (FSP_data, Jacob, dx, &got_gs, datafp);

      if (got_gs) 
      {
        Update_History(Old_DQs, dq);
        mat_free(dq);   /* dq has been allocated already by findt */
        if (Robot->PLAT.Holonomic)
	   dq = findt_with_Betas_Holonomic(FSP_data, B, H, datafp);
	else
	   dq = findt_with_Betas_Nonholonomic(FSP_data, B, H, datafp);
      }
    }
  } 					/* end else LAGRANGIAN */
  else if (Optimization == SIMPLEX)
  {
    if (IKCriterion[0] == 4)
    {
      dq = findt_without_Betas_SIMPLX(FSP_data, B, H, datafp);
    }
    else IKerror(31, FATAL, "SIMPLX only for Flow Currently\n");
  }					/* end SIMPLEX */


  mat_free (B);
  mat_free (H);

  return(dq);
}/*END ROUTINE*/



/* ________________________________________________________
  | name:    OPEN_FILES                                    |
  | description:  This procedure opens most of the system  |
  |     files (spheredata is in spheres() in useful_UTILS.c|
  |     Also the initial condition to the differential     |
  |     equation (notably Qarray, initial Angles) is def-  |
  |     ined, and a trajectory file is read into XTraj.    |
  |                                                        |
  | inputs:  XTraj, angle output file, Qarray              |
  | outputs: XTraj contains trajectory, Qarray has initial |
  |     angle values.                                      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch    2/92     Who knows who started this one  |
  |  c hacker    10/94     restructured, removed globals   |
  |________________________________________________________| */
int OPEN_FILES(XTraj, PoorMansFile, Qarray)
char   *PoorMansFile;
double  XTraj[MAX_PTS][6];
MATRIX *Qarray;
{			
  FILE  *XYZfile, *EAfile;
  int    i, Num_Of_Pts,
	 flag = -1,
	 fatal=  1;
  char   tempname[20];
  float  temp_Q;

  /*********************FILE INITIALIZATION*************************/
  #ifdef DEBUG_OUT
    fprintf (stderr, "\nANGLES PRINTED TO %s.\n", PoorMansFile);
  #endif

  if (Robot->Orient)
    if ((EAfile     = fopen("EulerAngles",  "r")) == NULL) 
      { IKerror (11, OK, "EulerAngles");   flag = 0; }
  if ((XYZfile    = fopen("TrajectoryPts","r")) == NULL)
     { IKerror (11, OK, "TrajectoryPts"); flag = 0; }
  if ((CheezyFile = fopen(PoorMansFile,   "w")) == NULL) 
     { IKerror (11, OK,  PoorMansFile);   flag = 0; }
  if ((NormFile   = fopen("NORMFILE",     "w")) == NULL) 
     { IKerror (11, OK, "NORMFILE");      flag = 0; }
  if ((ErrorFile  = fopen("ERRORFILE",    "w")) == NULL) 
     { IKerror (11, OK, "ERRORFILE");     flag = 0; }
  if ((FLWfile    = fopen("FLOW",         "w")) == NULL) 
     { IKerror (11, OK, "FLOW");          flag = 0; }
  if ((dqfile     = fopen("Delta-qs",     "w")) == NULL) 
     { IKerror (11, OK, "Delta-qs");      flag = 0; }
  if ((distfp     = fopen("Distance",     "w")) == NULL) 
     { IKerror (11, OK, "Distance");      flag = 0; }

  if (!flag) IKerror (12, FATAL, "OPEN_FILES");

  /*------------------------------------------------------------------*/
  /* READ TRAJECTORY INTO ARRAY XTraj, MAX 3000 POINTS (makepts.h)    */
  /*------------------------------------------------------------------*/
  fscanf(XYZfile, "%d", &Num_Of_Pts);
  if (Num_Of_Pts > MAX_PTS)
  { IKerror (15, OK, "MAX_PTS in general.h"); Num_Of_Pts = MAX_PTS; }

  /*-----------------------------------------------------------*/
  /* Print first line of Q angles at t=0                       */
  /* terminated with a '\n'.			               */
  /*-----------------------------------------------------------*/
  for (i = 0; i < Robot->NA; i++)
  {
    fscanf ( XYZfile, "%f", &temp_Q);
    if (i < M) Qarray->p[i][0] = rad(temp_Q);
  }
  fmat_pr (datafp, "Initial Q Array", Qarray);
  for (i = 0; i < Num_Of_Pts; i++)
  {
    fscanf(XYZfile, "%lf %lf %lf", 
		     &XTraj[i][0], &XTraj[i][1], &XTraj[i][2]);
    if (N>3)
    fscanf(EAfile,  "%lf %lf %lf", 
		     &XTraj[i][3], &XTraj[i][4], &XTraj[i][5]);
    /*
      if (DEBUG)
        if (N>3)
	  fprintf(datafp,"X,Y,Z: %6.4f %6.4f %6.4f EULER: %6.4f %6.4f %6.4f\n",
		XTraj[i][0],XTraj[i][1],XTraj[i][2],
		XTraj[i][3],XTraj[i][4],XTraj[i][5]);
        else
  	  fprintf(datafp,"X,Y,Z: %6.4f %6.4f %6.4f\n",
		XTraj[i][0],XTraj[i][1],XTraj[i][2]);
    */
  }
  fclose(XYZfile);
  if (N>3) fclose(EAfile);

  /*--------------------------------------------------------*/
  /* Print Header Line For IGRIP Simulation Angles File     */
  /* Which is in turn read in through Dave Reister's file   */
  /* which puts the points to the IGRIP model every so many */
  /* seconds to view the motion.                            */
  /*--------------------------------------------------------*/

  fprintf (CheezyFile, "M: %d N: %d\n", M, N);

  return (Num_Of_Pts);
}			     /* END OPEN_FILES */


/* ________________________________________________________
  | name:    init_Globals                                  |
  | description:  This, I'm not too proud of.  In order to |
  |     CheezyIGRIP work with different manipulators I had |
  |     to incorporate this, but unfortuneately CheezyIGRIP|
  |     is indeed Cheezy in many ways, one being its thirst|
  |     for global variables.  A,B,C,D,E,R,X and others    |
  |     are needed to be global for the forward kinematic  |
  |     which CheezyIGRIP uses.  Someone should eventually |
  |     remove these globals into a structure before it    |
  |     gets even more out of hand, but alas... tis not I. |
  | inputs:  none                                          |
  | outputs: As named, also initializes manipulator        |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     3/95     created sadly                   |
  |________________________________________________________| */

int Init_Globals()
{
  int i,j;
  float HandWidth   =0.05,
	HandLength  =0.05,
	FrontLength =ZERO,
	SideLength  =ZERO;


  if ((datafp   = fopen("data", "w")) == NULL) IKerror (11, FATAL, "data"); 

  /*------------------------------------------------------------------*/
  /* READ Users Manipulator's Parameters into System (Manipulators.c) */
  /*------------------------------------------------------------------*/
  init_ARM();       /* Initialize User_Selected Arm     */

  /*allocate memory for array of links*/
  all_links = (MATRIX **) malloc ( sizeof(MATRIX *) * Robot->NL );
  link_location = (MATRIX **) malloc ( sizeof(MATRIX *) * Robot->NL );

  /* E1-E4 are vectors to locate the 4 points used to draw the end effector */

  E1 = mat_malloc(4,1);  
  E2 = mat_malloc(4,1);  
  E3 = mat_malloc(4,1);
  E4 = mat_malloc(4,1);

  /* EW1-EW4 are locations of the 4 points of the EE wrt world coordinates */
  EW1 = mat_malloc(4,1); 
  EW2 = mat_malloc(4,1);   
  EW3 = mat_malloc(4,1);
  EW4 = mat_malloc(4,1);
 
  /* P1-P4 are vectors to locate the 4 points used to draw the platform */
  P1 = mat_malloc(4,1);  
  P2 = mat_malloc(4,1);   
  P3 = mat_malloc(4,1);
  P4 = mat_malloc(4,1);

  PW1 = mat_malloc(4,1);  
  PW2 = mat_malloc(4,1);   
  PW3 = mat_malloc(4,1);
  PW4 = mat_malloc(4,1);

  /* Also see Jacob_ROBOT.c for the use of A... */

  for ( i = 0; i < Robot->NL; i++) {
    /* location of end of links wrt world coords */
    all_links[i] = mat_malloc(4,1);
    /* location of ends of links */
    link_location[i] = mat_malloc(4,1);
  }

  R = mat_malloc(4,1);     /* vector to locate base of manipulator */
  X = mat_malloc(4,1);     /* vector to locate manip on platform */
  RW = mat_malloc(4,1);    /* location of manip base wrt world coords   */
  XW = mat_malloc(4,1);    /* location of manip on plat wrt world coords*/

    /* initialize all location vectors and transformation matrices */
  for (i=0;i<=3;i++)  
  { 
    for (j=0; j<Robot->NL; j++) {
      link_location[j]->p[i][0]=0;
    }
    R->p[i][0]=0;  X->p[i][0]=0;
    P1->p[i][0]=0;P2->p[i][0]=0;P3->p[i][0]=0;P4->p[i][0]=0;
    E1->p[i][0]=0;E2->p[i][0]=0;E3->p[i][0]=0;E4->p[i][0]=0;
  }

  /* drawing vectors for the manipulator */
  E1->p[0][0]=Robot->LINKS[Robot->NL-1]+HandLength; E1->p[2][0]=-HandWidth;
 		        		  E1->p[3][0]=1.0;
  E2->p[0][0]=Robot->LINKS[Robot->NL-1]           ; E2->p[2][0]=-HandWidth; 
					  E2->p[3][0]=1.0;
  E3->p[0][0]=Robot->LINKS[Robot->NL-1]           ; E3->p[2][0]= HandWidth; 
					  E3->p[3][0]=1.0;
  E4->p[0][0]=Robot->LINKS[Robot->NL-1]+HandLength; E4->p[2][0]= HandWidth; 
					  E4->p[3][0]=1.0;

  /* drawing vectors for the platform */
  if (Robot->PLAT.Exist) {
    FrontLength = Robot->PLAT.Length;
    SideLength  = Robot->PLAT.Width;
  }
  P1->p[0][0]=-FrontLength/2; P1->p[1][0]=-SideLength/2; P1->p[3][0]=1.0;
  P2->p[0][0]=-FrontLength/2; P2->p[1][0]= SideLength/2; P2->p[3][0]=1.0; 
  P3->p[0][0]= FrontLength/2; P3->p[1][0]= SideLength/2; P3->p[3][0]=1.0;
  P4->p[0][0]= FrontLength/2; P4->p[1][0]=-SideLength/2; P4->p[3][0]=1.0;

  /* manip base offset is drawn along a local y axis */
  R->p[2][0]=0; R->p[3][0]=1.0;  
  X->p[2][0]=0; X->p[3][0]=1.0;
}


/* ________________________________________________________
  | name:    Update_History                                |
  | description:  Updates the history of joint angle steps.|
  | inputs:  none                                          |
  | outputs:                                               |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  K Morgansen  7/95       created                       |
  |________________________________________________________| */

void Update_History(History *Old_DQs, MATRIX *dq)
{
int prev;

Old_DQs->whereami++;
if (Old_DQs->whereami == HIST_SIZE) Old_DQs->whereami = 0;
prev = Old_DQs->whereami-1;
if (prev == -1) prev = HIST_SIZE - 1;

mat_cp(dq, Old_DQs->dq[Old_DQs->whereami].DQ);
Old_DQs->dq[Old_DQs->whereami].time=Old_DQs->dq[prev].time+0.033332;
	 /*Old_DQs->dq[Old_DQs->whereami].time=clock()/1.0e6;      */
         /* this is a hack to make sure the results are consistent */

}
