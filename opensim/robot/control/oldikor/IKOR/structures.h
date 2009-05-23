/* ________________________________________________________
  |                                                        |
  | Program Name:   structures.h                           |
  |________________________________________________________|
  |                                                        |
  | description: Contains data structures for system as    |
  |    well as a brief description of each. For more info  |
  |    try the user's manual.                              |
  |                                                        |
  | Structures: (order of appearance)                      |
  |    MATRIX                                              |
  |    ANGLE                                               |
  |    Platform                                            |
  |    Manipulator_struct                                  |
  |    Solutions                                           |
  |________________________________________________________| */
#include <time.h>
#include <stddef.h>

/* MATRIX_struct is a structure for the matrixes used in the system */
typedef struct 
{
  float **p;	/* Pointer to array of pointers to matrix rows. */
  int   rows;   /* Row dimension of the matrix.                 */
  int   cols;   /* Column dimension of the matrix.              */
} MATRIX;


/* Angles is an array containing containing several instances*/
/* of the following structure: limits, home, weight, etc...  */
typedef struct {
  int   Prism;       /* ==Y then translates ==N then rotates */
  float Max_limit,   /* Max Limit for specific joint         */
        Min_limit,   /* Min Limit for specific joint         */
        Max_accel,   /* Max Limit for acceleration           */
        Min_accel,   /* Min Limit for acceleration           */
	Home;        /* Home position of joint               */
} ANGLE;


/* PLAT is an instance of the following structure which con- */
/* tains information about a specific platform.              */
typedef struct {
   int   Active,     /* Determines if Platform is active     */
         Exist,      /* Determines if Platform exists        */
         Holonomic;  /* ==Y Holonomic, ==N Automobile-like   */
   float Length,     /* Length of Platform                   */
	 Width,      /* Width  of Platform                   */
	 Thick,      /* Thickness of Platform                */
	 Z_OFF,      /* offset of arm from platform Z        */
	 L_OFF,      /* offset of arm from platform center   */
	 ANG_OFF;    /* angle offset of first joint of Manip */
   MATRIX *Man_base; /* location of the base of manipulator  */
   MATRIX *Corner[4]; /* locations of the corners of platform */
} Platform;

/* Gripper is an example of this structure */
typedef struct {
    MATRIX *Corner[4];   /* Points on end effector */
    float   Orient[3];   /* Orientation of end effector */
    float   Prev_pos[3]; /* Position of end effector at previous time step */
} EndEffector;

/* Robot is an instance of the following structure which con-*/
/* tains information of the Manipulator.	             */
typedef struct {
   int   NA,           /* Maximum number of Angles             */
         NL,           /* Number of Links in Manipulator       */
         NX,           /* set the workspace pos    dim (2->2d) */
         NO,           /* set the workspace orient dim (1->2d) */
         Orient;       /* Use orientation control              */
   double    *alpha,   /* D-H parameters */
             *theta,
             *dvals,
	     *avals;
   MATRIX    *trans;    /* holds '1' or '0' if joint is active  */
		        /* inside a transformations             */
   ANGLE     *Angles;   /* see Angles_struct above.             */
   Platform   PLAT;     /* see Platform_struct above.           */
   MATRIX    *Weights;  /* How active the joint is 0<x<= 1      */
   MATRIX   **x_of_link;/* Holds locations of the links         */
   double    *LINKS;    /* length of the links of the robot     */
   EndEffector Gripper; /* location of points on end effector   */
} Manipulator_struct; 

/* FSP_data is an instance of the following structure type.  */
/* It contains info vital to FSP and IKOR's integration.     */
typedef struct {
    int   M, N,         /* (unused) may be used to replace globals */
	  Mred, 	/* number of columns in reduced Jacobian   */
	  Nred, 	/* number of rows    in reduced Jacobian   */
	  cn,	 	/* number of constraints on system (betas) */
	  Null_Space;	/* flag for detection of Null_Space Motion */
    MATRIX *g,          /* array of solution vectors               */
	   *Qarray,     /* current angles positions ( in rad )     */
           *Impact,     /* location of end effector at impact      */
	   *betall,     /* Contains All Betas (Beta all)           */
	   *Xelim;	/* Contains strict values for reduced dq's */
} Solutions;

/* Old_dq[] is an example of the following structure type. */
/* The size of this array is in general.h                  */
typedef struct {
    double   time;       /* time at which calc finished */
    MATRIX  *DQ;         /* vector of joint steps */
    } History_Element;


#define HIST_SIZE       5       /* must be at least 5 */
typedef struct {
    int whereami;                  /* points to most recent event in history */
    History_Element dq[HIST_SIZE]; /* array of joint movements               */
    } History;


