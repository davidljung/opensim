/* ________________________________________________________
  |                                                        |
  | Program Name:   general.h                              |
  |________________________________________________________|
  |                                                        |
  | description:  Contains defines that are used by many   |
  |     of the modules in the IKOR system.  Every file that|
  |     gets compiled seperately must contain this file    |
  |     for adequate linkage to external variables.        |
  |________________________________________________________| */


#ifndef _GENERAL_H
#define _GENERAL_H

/*#define DEBUG_OUT	 ** uncomment this line when DEBUGGING    **/
/*#define FSP_DEBUG	 ** Makes FSP run on Jacob,dx in FSP_data **/
/** The MACRO DEBUG was removed, use the commandline to see output **/
#define DEBUG 1

#include <math.h>
#include <stdio.h>
#include <IKOR/structures.h>

/*****************************************************************
 *                                                               *
 *                      Global External References               *
 *                                                               *
 *****************************************************************/
extern FILE *datafp, *FLWfile, *dqfile, *distfp;

extern char *PoorMansFile,	/** Globals.h **/
             Method[5][30],	/** Globals.h **/
	     dxLabel[6][30],	/** Globals.h **/
             ARM_file[15];	/** Globals.h **/
				/** Globals.h **/
extern MATRIX   **all_links, *RW, *XW,
		*PW1, *PW2, *PW3, *PW4,
		*EW1, *EW2, *EW3, *EW4;

extern int N, M,                  /** Globals.h  **/
           STEP,           /** Globals.h  **/
           CHANGE_SPHERES;        /** Globals.h  **/
extern Manipulator_struct *Robot; /** Globals.h  **/
extern double *LL;		  /** Globals.h  **/
/*****************************************************************
 *                                                               *
 *                      Macro Definitions                        *
 *                                                               *
 *****************************************************************/

#define SPAN		(FSP_data->g->rows)
#define SPAN2		(FSP_data->g->rows - NumSpg)
#define SQUARE(a)       ((a)*(a));
#define rad(x)          ((x)*PI/180) /*Convert to Radians*/
#define deg(x)          ((x)*180/PI) /*Convert to Degrees*/
#define SWAP(a,b)       {double temp; temp=a;a=b;b=temp;}
#define MAX(a,b)        ((a) > (b) ? (a) : (b))
#define SIGN(a,b)       ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define PYTHAG(a,b)     ((at=fabs(a)) > (bt=fabs(b)) ? \
(ct=bt/at,at*sqrt(1.0+ct*ct)) : (bt ? (ct=at/bt,bt*sqrt(1.0+ct*ct)): 0.0))


#define SPEED            .3		/* Flow Defines */
#define MAX_PTS      5000 		/* Sets Maximum Points in Trajectory  */
#define BIG       5.0e-02		/* user-defined value for IKOR	      */
#define SMALL     5.0e-05		/* user-defined value for FSP & IKOR  */
#define	K2_BND	  1.0e+01		/* user-defined value for FSP	      */
#define ZERO            0.0		/* General Defines */
#define PI              3.14159265
#define	TRUE	        1
#define	FALSE           0
#define LAGRANGIAN	1		/* Optimization Defines */
#define BANGBANG	2
#define SIMPLEX		3
#define	ZEROD_OMEGAS    2		/* Constant Orientation Define */
#define FATAL		0		/* IKerror Defines */
#define OK		1
#define ONE             1		/* STEP Defines */
#define TWO             2
#define JN_LIMITS       0		/* Constraint Defines */
#define OBSTACLES       1
#define ACCELRATN       2
#define EE_IMPACT       3
#define COMPLETE	2		/* Solution Generator Return Defines */
#define NOT_COMPLETE   -1
#define RESTRICTED     -2

/*******************************************************************
 *                                                                 *
 *                      FUNCTION PROTOTYPES                        *
 *  function prototypes for every function should be in headers.h) *
 *                                                                 *
 *******************************************************************/
#include <IKOR/headers.h>
#include <UTILS/nrutil.h>
#include <UTILS/matrix.h>

#endif      /* end !_GENERAL_H */
