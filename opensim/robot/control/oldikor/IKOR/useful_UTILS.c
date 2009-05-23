/* ________________________________________________________
  |                                                        |
  | Program Name:   useful_UTILS.c                         |
  |________________________________________________________|
  |                                                        |
  | description: This file contains stuff that I'm not sure|
  |    where to put it in the system.  Some of them like   |
  |    Solutions_init really don't belong here, others     |
  |    might. I recommend that if you too have procedures  |
  |    to add and not quite sure where they should go,     |
  |    leave them here_(8^>).                              |
  |                                                        |
  | Procedures:                                            |
  |    IKerror          :                                  |
  |    Solutions_init   :                                  |
  |    Solutions_free   :                                  |
  |    GetData          :                                  |
  |    spheres          :                                  |
  |    fmat_pr/fmat_prf :                                  |
  |    fprint_norm      :                                  |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Header File to link all others */

/* ________________________________________________________
  | name:    IKerror                                       |
  | description: Prints out error message and exits to     |
  |          to the system, if desired.                    |
  |                                                        |
  | inputs:   error_vector and fatal flag.                 |
  | outputs:  error message displayed to screen.           |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker     3/95      Created		           |
  |________________________________________________________| */

void IKerror(int error_vector, int fatal, char *mesg)
{
  fprintf (stderr, "\nERROR %d: ", error_vector);
  switch  (error_vector) {
      case 10: fprintf (stderr, "FSP didn't complete!\n");             break;
      case 11: fprintf (stderr, "Unable to Open '%s'\n", mesg);        break;
      case 12: fprintf (stderr, "General File Error in '%s'", mesg);   break;
      case 15: fprintf (stderr, "Too Many Points in Trajectory. '%s'\n",mesg);
						  		       break;
      case 16: fprintf (stderr, "Request Joint Space %d is > %d in %s.\n",
			                         M, Robot->NA,mesg);   break;
      case 17: fprintf (stderr, "Requested Task Space %d is > 6 in %s.\n",
							      mesg);   break;
      case 18: fprintf (stderr, "Memory allocation error in 's'\n",mesg);
								       break;
      case 19: fprintf (stderr, "Manipulator File Integrity Lost \n Bad data",
			        " file: ROBOT.dat in '%s'\n", mesg);   break;
      case 21: fprintf (stderr, "SORRY!...more constraints %s %s\n",
				"than D.O.R. in procedure: ", mesg);   break;
      case 23: fprintf (stderr, "Defaulting to using one-shot in %s.\n",
							      mesg);   break;
      case 24: fprintf (stderr, "Platform does not exist, see %s.\n", mesg);
                                                                       break;
      case 25: fprintf (stderr, "System did not complete.\n");         break;
      case 26: fprintf (stderr, "Requested dx[%s] too large.\n",mesg); break;
      case 29: fprintf (stderr, "%s%s%s\n", "System Error: Did not Execute One",
                                " Step or Two Step in ",mesg);         break;
      case 30: fprintf (stderr, "Division by zero in Analytical.c\n"); break;
      case 31: fprintf (stderr, "Option UnImplemented: %s\n",mesg);    break;
      default: fprintf (stderr, "No such Error. error_vector doesn't exist.\n");
  }

  if (fatal == FATAL)
     { fprintf(stderr, "FATAL Error.  aborting... \n"); abort(); }
  else fprintf(stderr, "Error not Fatal.  Continuing execution...\n");
}


/* ________________________________________________________
  | name:    Solutions_init                                |
  | description: The Solutions structure is explained in   |
  |     structures.h, this procedure is used to declare    |
  |     and allocate memory for the structure.             |
  |                                                        |
  | inputs:  M (joint space) and N (Task space)            |
  | outputs: Allocates memory for the three Matrixes as    |
  |     well as the structure itself. Also initializes     |
  |     M, N, Mred, Nred (Mred and Nred are the current    |
  |     delienators of the space, they are changed in      |
  |     the file FSP.c in the function ReduceA.            |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker     3/95      Created		           |
  |________________________________________________________| */

Solutions *Solutions_init(int M, int N)
{
  int i, j;
  Solutions *Temp;

  Temp = (Solutions *) malloc( sizeof( Solutions ) );
  Temp->g	= mat_malloc( M, M);         /* Soln   */
  Temp->Qarray  = mat_malloc( M, 1);         /* Angles */
  Temp->Xelim   = mat_malloc( M, 1);         /* Angles */
  Temp->betall  = mat_malloc( M - N + 15, M); /* Betas  */

  for (i=0; i< Temp->betall->rows; i++)
	for (j=0; j<Temp->betall->cols; j++)
		Temp->betall->p[i][j]=0.0;

  if(!Temp) IKerror(18, FATAL, "Solutions_init");

  Temp->M = Temp->Mred = M;    /* init  space size */
  Temp->N = Temp->Nred = N;
  Temp->cn = 0;                /* init # constrnts */

  return (Temp);
}

/* ________________________________________________________
  | name:    Solutions_free                                |
  | description: The Solutions structure is explained in   |
  |     structures.h, this procedure is used to free       |
  |     any allocated memory for the structure.            |
  |                                                        |
  | inputs:  M (joint space) and N (Task space)            |
  | outputs: Deallocates memory for the three Matrixes as  |
  |     well as the structure itself.                      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker     3/95      Created		           |
  |________________________________________________________| */

void Solutions_free( Solutions *Temp )
{
  mat_free(Temp->betall);
  mat_free(Temp->Qarray);
  mat_free(Temp->Xelim);
  mat_free(Temp->g);
  free ( (char *) Temp);
}


/* ________________________________________________________
  | name:    GetData                                       |
  | description:    read in A and b vector to be used for  |
  |     testing the code with a single jacobian and dx. The|
  |     macro DEBUG_FSP must be defined in the file:       |
  |                    general.h.                          |
  |                                                        |
  | inputs:  A file called : FSP_data                      |
  | outputs: Aorig, borig                                  |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  K Morgansn  6/94      Created		           |
  |  c hacker    3/95      Removed struct from header      |
  |________________________________________________________| */

GetData	( MATRIX *A,  MATRIX *b)
{
  FILE *fpin;
  int i, j;

  fpin = fopen("FSP_data", "r");

  for (i=0; i<N; i++)
    for (j=0; j<M; j++)
      fscanf(fpin, "%f", &A->p[i][j]);

  for (i=0; i<N; i++)
    fscanf(fpin, "%f", &b->p[i][0]);

  fclose(fpin);
}


/* ________________________________________________________
  | name:    spheres                                       |
  | description: spheres reads a datafile, or user's input |
  |     for data concerning spherical objects.  However, I |
  |     rarely use this in an windows interface becuase it |
  |     is easier to just change the sphere_data file it-  |
  |     self, but to each his own.                         |
  | inputs:  spheredata a 2D array                         |
  | outputs: spheredata contains spherical obstacles.      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | F Tulloch    2/93      Created		           |
  | c hacker    10/94      removed spheredata as global    |
  |________________________________________________________| */

int spheres(spheredata, datafp)
FILE  *datafp;
double spheredata[4][4];
{
  int   ns, sphere_no, i;
  FILE *sphere_file;
  char  question='n';

  if (CHANGE_SPHERES)
  {
    fprintf(stdout, "\nDo you want to change the spheres? ");
    scanf(" %c", &question);
  }

  if ((sphere_file = fopen("sphere_data", "r+")) == NULL)
  { IKerror(11, OK, "sphere_data"); return 0; }
  
  if ((question != 'Y') && (question != 'y'))
  {
    fscanf(sphere_file, "%d", &ns);
    for (sphere_no = 0; sphere_no < ns; sphere_no++)
      for (i = 0; i < 4; i++)
	fscanf(sphere_file, "%lf", &spheredata[sphere_no][i]);
  }
  else
  {
    fprintf(stdout, "How many spheres? (4 max): ");

    /*IF FILE EXISTS IT MUST  define how        */
    /*  many spheres to read even if 0 spheres  */
    /* (don't just have an empty file.)         */
    scanf("%d", &ns);
    if (ns > 4)  ns = 4; /*LIMITATION*/

    for (i = 0; i < ns; i++)
    {
      fprintf(stdout, "xcenter of sphere[%d]: ", i);
      fscanf(stdin, "%f", &spheredata[i][0]);
      fprintf(stdout, "ycenter of sphere[%d]: ", i);
      fscanf(stdin, "%f", &spheredata[i][1]);
      fprintf(stdout, "zcenter of sphere[%d]: ", i);
      fscanf(stdin, "%f", &spheredata[i][2]);
      fprintf(stdout, " radius of sphere[%d]: ", i);
      fscanf(stdin, "%f", &spheredata[i][3]);
    }

    fprintf(sphere_file, "%i\n", ns);
    for (sphere_no = 0; sphere_no < ns; sphere_no++)
    {
      for (i = 0; i < 4; i++)
	fprintf(sphere_file, "%f ", spheredata[sphere_no][i]);
      fprintf(sphere_file, "\n");
    }
  }			     /******* end else ********/

  fprintf(datafp,"\n Spheres read into the system...");
  for (i=0; i< ns; i++)        /* confirm proper initialization  */
    fprintf(datafp,"\n%i: <%f, %f, %f> %f",i,
	spheredata[i][0], spheredata[i][1], spheredata[i][2], spheredata[i][3]);

  fclose(sphere_file);
  return (ns);
}			     /* end spheres... */


/* ________________________________________________________
  | name:    fmat_pr/ fmat_prf                             |
  | description: Disqusted with the inability to print the |
  |     MATRIX to a file which is currently the way IKOR   |
  |     communicates with the outside world, I have alter- |
  |     ed the mat_pr function in matrix.c. The function   |
  |     that ends with an 'f' means more decimal places    |
  |     will be printed out.                               |
  | inputs:  Datafile, description string, and a MATRIX    |
  | outputs: The checkfile contains the input string and   |
  |     MATRIX.                                            |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker    10/94      Created		           |
  |              3/95      took struct from "struct MATRIX"|
  |________________________________________________________| */

void fmat_pr (FILE *checkfile, char *string, MATRIX *a)
{ 
  int i, j;

  fprintf(checkfile, "\n   %s ROWS: %d, COLS: %d\n", 
	string, a->rows, a->cols);
  for (i = 0; i < a->rows; i++)
  {
    for (j = 0; j < a->cols; j++)
      fprintf(checkfile, "%8.4f ", a->p[i][j]);
    fprintf(checkfile, "\n");
  }
  fprintf(checkfile, "\n");
}

void fmat_prf(FILE *checkfile, char *string, MATRIX *a)
{
  int i, j;

  fprintf(checkfile, "\n   %s Rows: %d, Cols: %d\n",
	 string, a->rows, a->cols);
  for (i = 0; i < a->rows; i++)
  {
    for (j = 0; j < a->cols; j++)
    {
      fprintf(checkfile,"   %16.12f", a->p[i][j]);
      }
    fprintf(checkfile, "\n");
  }
  fprintf(checkfile, "\n");
}


/* ________________________________________________________
  | name:       fprint_norm                                |
  | description: Prints out the Norm of the change in      |
  |     angles.  This has been used mostly to compare the  |
  |     psuedo inverse against the FSP.                    |
  | inputs:  Datafile, change in angles                    |
  | outputs: The datafile contains the least norm.         |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | D Carlson    2/92      Created		           |
  | c hacker    10/94      Removed Squarroot function      |
  |              3/95      took struct from "struct MATRIX"|
  |________________________________________________________| */
void fprint_norm(outfile,dq,datafp)
MATRIX *dq;
FILE   *outfile, *datafp;
{
  int i;
  double temp, result;

/*
fprintf(stderr, "\ntemp: %8.4f  ", temp);
fprintf(stderr, "temp^2: %8.4f   ", temp);  
fprintf(stderr, "result: %8.4f", result);  
*/

  for (i = 0, result = 0.0; i < dq->rows; i++)
  {
    temp    = (dq->p[i][0]);
    temp   *= temp;
    result += sqrt((temp * temp));
  }

  if (DEBUG) fprintf(datafp, "\n NORM value: %lf", result);

  fprintf (outfile, " %14.10f\n", result);
}


/* ________________________________________________________
  | name:       fprint_hist                                |
  | description: Prints out the the contents of the history|
  | inputs:  history                                       |
  | outputs:                                               |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | K Morgansen   8/95      Created		           |
  |________________________________________________________| */

void fprint_hist(outfile, Old_DQs)
FILE *outfile;
History *Old_DQs;
{
  int i, j;

  fprintf(outfile, "index pointer at:  %d\n", Old_DQs->whereami);
  fprintf(outfile, "times:\n");
  for (i=0; i<HIST_SIZE; i++)
    fprintf(outfile, "  %5.6lf   ", Old_DQs->dq[i].time);
  fprintf(outfile, "\n\n");
  for (i=0; i<HIST_SIZE; i++)
    {
    for (j=0; j<Old_DQs->dq[i].DQ->rows; j++)
      fprintf(outfile, "%8.6lf  ", Old_DQs->dq[i].DQ->p[j][0]);
    fprintf(outfile, "\n");
    }
  fprintf(outfile, "\t==========================================\n\n");

}


/* ________________________________________________________
  | name:       fdq_pr                                     |
  | description: Prints out the the contents of dq         |
  | inputs:  dq                                            |
  | outputs:                                               |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | K Morgansen   8/95      Created		           |
  |________________________________________________________| */

void fdq_pr (DQFile, Old_DQs)
FILE *DQFile;
History Old_DQs;
{
  int now,
      i;

  now = Old_DQs.whereami-1;
  if (now == -1) now = HIST_SIZE - 1;
  fprintf(DQFile, "%8.6lf  ", Old_DQs.dq[now].time);
  for (i=0; i<M; i++)
    fprintf(DQFile, "%8.6lf  ", Old_DQs.dq[now].DQ->p[i][0]);
  fprintf(DQFile, "\n");
}

