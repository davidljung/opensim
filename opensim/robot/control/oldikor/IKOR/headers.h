/* ________________________________________________________
  |                                                        |
  | Program Name:   headers.h                              |
  |________________________________________________________|
  |                                                        |
  | description: Should contain headers for all functions  |
  |    in the system referenced by the system file name    |
  | 07/11/01 (A Cordero) added Manipulators.c   (!?)       |
  |________________________________________________________| */

/******************* IKOR_driver.c ***************************/

int IKOR_driver (int *IKCriterion, int *IKMethod, int Optimization, 
	 int N_use, int M_use);
MATRIX *CALC_PSEUDO (MATRIX *Jacob, MATRIX *dx);
MATRIX *FSP (Solutions *FSP_data, History *Old_DQs, MATRIX *Jacob, MATRIX *dx, 
         int *IKCriterion, int *IKMethod, int Optimization, int ns,
	 double spheredata[4][4], MATRIX *x_of_link, FILE *datafp);
void  Euler_to_Velocities (MATRIX *x_of_link, MATRIX *dx);
float correct_euler ( float angle, float curr, float dest );
int   OPEN_FILES  (double XTraj[MAX_PTS][6], char PoorMansFile[40],
	 MATRIX *Qarray);
int   Init_Globals();
void  Update_History(History *Old_DQs, MATRIX *dq);


/********************* Jacob_UTILS.c ************************/
void   GET_JACOBIAN_ALTERED(MATRIX* Jacob, MATRIX *Qarray);
void   GET_JACOBIAN        (MATRIX* Jacob, MATRIX *Qarray);
void   ExtractRPY2 (MATRIX *T, MATRIX *x_of_link);
MATRIX *getT2      (double ZA,double YB,double XG, 
		    double tx,double ty,double tz);


/********************* Jacob_ROBOT.c ************************/
extern void   GET_JACOB     (MATRIX *Jacob,  MATRIX *Qarray);
extern void   GET_ACTUAL_X  (MATRIX *Qarray, MATRIX *x_of_link);


/********************** FSP.c ******************************/
int    Solution_generator (Solutions *FSP_data, MATRIX *Jacob, 
		   MATRIX *dx, FILE *datafp); 
void   RestofSoln (int Mred, int Nred, int NextToFind, MATRIX *block,
		   MATRIX *g, MATRIX *bred, MATRIX *Ared, int *Tackon,
		   int *FirstOK, FILE *check);
int    BLOCK_COL_FIND_X(int *Tackon, float *g, float *block,
		   MATRIX *b, MATRIX *A, 
 		   int Mred, int Nred, FILE *check);
int    ReduceA    (Solutions *FSP_data, MATRIX *Aorig, MATRIX *Ared,
		   MATRIX *borig, MATRIX *bred,  
		   int *ColElim, int *RowElim, 
		   int *NumSpg, MATRIX *Specialg);
int    CheckB     (MATRIX *b, int m);
int    CheckRange (MATRIX * b, MATRIX * Aorig, MATRIX * g,
	           int *RowElim, int Mred);
void   Rebuild_gs (Solutions *FSP_data, int *ColElim,
                    MATRIX *Specialg, int NumSpg);


/******************* useful_UTILS .c ***********************/
void   IKerror     (int  vector, int fatal, char *mesg);
Solutions  *Solutions_init(int M, int N);
void   Solutions_free(Solutions *Temp );
int    GetData    (MATRIX *A,  MATRIX *b);
int    spheres     (double spheredata[4][4], FILE *datafp);
void   fmat_pr     (FILE *checkfile, char *string, MATRIX *a);
void   fmat_prf    (FILE *checkfile, char *string, MATRIX *a);
void   fprint_norm (FILE *checkfile, MATRIX *dq, FILE *datafp);

/*********************** constraints.c *********************/
int    avoid_limits    (Solutions *FSP_data, MATRIX *Jacob, MATRIX *dx,
 		    int *got_gs, FILE *datafp);
int    avoid_obstacles (Solutions *FSP_data, MATRIX *Jacob, MATRIX *dx,
		    MATRIX *x_of_link, int ns, double spheredata[4][4],
		    int *got_gs, FILE *datafp);
int    find_intersection_sphere(double pt1[3], double pt2[3], int ws,
		    int ns, double spheredata[4][4], int *elbow_check, 
		    double *newl, double *delta, double normal[3],
		    FILE *datafp);


/*********************** criteria.c ************************/
int    Least_Norm (MATRIX *B, MATRIX *H, FILE *datafp);
int    Least_Flow (MATRIX *B, MATRIX *H, MATRIX *Qarray, FILE *datafp);
float  calc_flow  (int index, float Q, float dq, float distance, MATRIX *old);

/*********************** analytical.c *********************/
MATRIX *Build_Grammian2 (Solutions *FSP_data, MATRIX *alphas, FILE *datafp);
void    rank_lost_betas (Solutions *FSP_data, MATRIX *Jacob, MATRIX *dx,
		    	 int *RowElim, FILE *datafp);
void    find_jl_beta    (Solutions *FSP_data, int chk, 
			 double limit, FILE *datafp);
void    find_obs_beta   (Solutions *FSP_data, int chk, double *newl,
			 double delta, double normal[3], FILE *datafp);
MATRIX *findt_with_Betas_Holonomic       (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, FILE *datafp);
MATRIX *findt_without_Betas_Holonomic    (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, FILE *datafp);
MATRIX *findt_with_Betas_Nonholonomic    (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, FILE *datafp);
MATRIX *findt_without_Betas_Nonholonomic (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, FILE *datafp);
MATRIX *findt_without_Betas_BANGBANG     (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, MATRIX *old, FILE *datafp);
MATRIX *findt_without_Betas_SIMPLX       (Solutions *FSP_data, MATRIX *B, 
					  MATRIX *H, FILE *datafp);


/*********************** Manipulators.c ************************/

void init_ARM();
