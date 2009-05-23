/* ________________________________________________________
  |                                                        |
  | Program Name:   Globals.h                              |
  |________________________________________________________|
  |                                                        |
  | description:  Contains Global variables. These should  |
  |     be eliminated when ever the chance arrives. Good   |
  |     luck, because most of these variables are used by  |
  |     CheezyIGRIP.c                                      |
  |________________________________________________________| */


/* Globals N, and M can be removed easily since FSP_data contains these */
/*     variables as well and contain correct values.  However, I have   */
/*     not had the time to convert all N,M to FSP_data->N ect...        */
int    N=-1,		   /*  Holds size of Task Space          */
       M=-1,		   /*  Holds size of Joint Space         */
       STEP=0,             /* Tells system to use twostep meth.  */
  /*       DEBUG=0, DJ*/		   /* Tells system to print out info     */
       CHANGE_SPHERES=0;   /* Tells system to prompt for spheres */
FILE  *datafp,             /* datafile: Hardly used globally     */
      *NormFile,           /* Size of Normalized dq's            */
      *ErrorFile,          /* Size of Normalized errors in posi  */
      *CheezyFile,         /* angle output file: Hardly global   */
      *NHDfile,            /* Katie For testing non-holo code    */
      *dqfile,             /* File that stores change in angles  */
      *distfp,             /* File that stores distance EE moved */
      *FLWfile;            /* Hydraulic Flow Rate Approximations */
char  *PoorMansFile="CheezyAngles",   /*Name for Angles Output   */
       Method[5][30]={  "FSP & Joint Limits", "PSEUDO INVERSE", 
		         "FSP & Obstacles"   , "FSP & JL & OA ", "FSP only"  },
       dxLabel[6][30]={ "X    ", "Y    ", "Z    ", 
			 "YAW  ", "PITCH", "ROLL " };
FILE* gcheck;

/* These globals are used in Init_Globals, Jacob_ROBOT.c, and CheezyIGRIP */
/*    They are fairly well imbedded into CheezyIGRIP, however not so much */
/*    in Jacob_ROBOT.c							  */

MATRIX  **all_links, /* location of array of all links */
        *RW,   /* location of manip base wrt world coords   */
        *XW,   /* location of manip on plat wrt world coords*/

        **link_location, /* replaces A, B, C... Vector to locate end of link n */
	*R,*X, 

	*PW1, *PW2, *PW3, *PW4, *EW1, *EW2, *EW3, *EW4,
	*P1,*P2,*P3,*P4,*E1,*E2,*E3,*E4;

char ARM_file[15] = "ROBOT.dat";              /* File to read manipulator  */
Manipulator_struct *Robot; /* Information on manipulator, see structures.h */
double   *LL;         /* get_jacobain uses these values for obstacle avoid */
                      /* these are values from the robot                   */

float EEzaxis = 0;   /* these are used to correct the default orientation of */
float EEyaxis = 0;   /* the EE. Used in Jacob_ROBOT.c & Init_ARM if the      */
float EExaxis = 0;   /* end-effector description is being overriden.         */
