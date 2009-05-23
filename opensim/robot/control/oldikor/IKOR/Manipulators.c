/* ________________________________________________________
  |                                                        |
  | Program Name:   Manipulators.c                         |
  |________________________________________________________|
  |                                                        |
  | description: Integrates different manipulator's into   |
  |    the system by reading ARM_file, "ROBOT.dat".  This  |
  |    allows a data file, such as ROBOT_AIRARM.dat, to be |
  |    copied into ROBOT.dat and the system will now emul- |
  |    the new manipulator. See User's Guide or sample a   |
  |    sample data file for creation of a new manipulator. |
  |                                                        |
  | Procedures:                                            |
  |    go_next         :                                   |
  |    rm_blanks       :                                   |
  |    init_arm        :                                   |
  |________________________________________________________| */


#include <IKOR/general.h>

/* ________________________________________________________
  | name:            go_next                               |
  | description:  Finds an Exclamation Mark in requested   |
  |     file and then continues to read the rest of the    |
  |     line containing that delimiter.  Prepares file     |
  |     read pointer to get input on the line immediately  |
  |     following the excamation mark.                     |
  |           33 is the exclamation '!' character          |
  |           10 is the newline    '\n' character          |
  |                                                        |
  | inputs:  File to find the delimiter                    |
  | outputs: File read pointer points to beginning of next |
  |     line.                                              |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker      3/95     Created		           |
  |________________________________________________________| */
int go_next(FILE *ARM)
{
  char CHAR = 'x';

  while ((!feof(ARM)) && (CHAR != 33)) CHAR = fgetc(ARM);
  if (!feof(ARM))
    while ((!feof(ARM)) && (CHAR != 10)) CHAR = fgetc(ARM);

  if (feof(ARM)) IKerror (19, FATAL, "go_next");
}

/* ________________________________________________________
  | name:            rm_blanks                             |
  | description:  Reads past all blanks and tabls in a     |
  |     line and returns the first character found.        |
  |                                                        |
  | inputs:  File to find the character                    |
  | outputs: Returns first non blank or non tab character  |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker      3/95     Created		           |
  |________________________________________________________| */

int rm_blanks(FILE *ARM)
{
  char CHAR=' ';

  while ((!feof(ARM)) && ((CHAR == 32) || (CHAR == 9))) CHAR = fgetc(ARM);
  if (feof(ARM)) IKerror (19, FATAL, "rm_blanks"); 

  return CHAR;
}

/* ________________________________________________________
  | name:            init_ARM                              |
  | description:  Reads file determined by global character|
  |      constant defined in Globals.h, and then builds a  |
  |      manipulator based on that data.                   |
  |                                                        |
  | inputs:  char constant ARM_file                        |
  | outputs: Global variables PLAT, Angles, LINKS, and LL  |
  |      are built.                                        |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  | c hacker      3/95     Created		           |
  |________________________________________________________| */

void init_ARM()
{
  int   i,j;       /* Loop counter variable                    */
  FILE *ARM;     /* File pointer that will point to ARM_file */
  double temp_f; /* Temporary float variable for reading     */
  int    temp_i; /* Temporary int   variable for reading     */
  extern float EEzaxis;  /* these are used to correct the default               */
  extern float EEyaxis; /* orientation of the EE. Used in Jacob_ROBOT.c IF the */
  extern float EExaxis;   /* end-effector description is being overriden.        */
  

  if (( ARM = fopen(ARM_file, "r") )==NULL) IKerror (11, FATAL, ARM_file);

  if (!(Robot = (Manipulator_struct *) malloc ( sizeof(Manipulator_struct))))
        IKerror (18, FATAL, "init_ARM");

  go_next(ARM);         /* find next exclamation mark       */
  fscanf(ARM,"%d",&temp_i);
  Robot->NL = temp_i;   /* Read in Number of Links          */
                        /* Allocate memory for LINKS and LL */
  if (!(Robot->LINKS = (double *) malloc( Robot->NL * sizeof(double) )))
        IKerror (18, FATAL, "init_ARM");
  if (!(LL           = (double *) malloc( Robot->NL * sizeof(double) )))
        IKerror (18, FATAL, "init_ARM");

  go_next(ARM);         /* find next exclamation mark       */
                        /* Assign Link Lengths to LINKS     */
  for (i=0; i<Robot->NL; i++)
  {
    fscanf(ARM,"%lf", &temp_f);
    Robot->LINKS[i] = temp_f;
  }

  go_next(ARM);         /* find next exclamation mark       */
  fscanf(ARM,"%d",&temp_i);
  Robot->NA = temp_i;   /* Read in Number of Angles         */
                        /* Allocate memory for Angles struct*/
  if (!(Robot->Angles = (ANGLE *) malloc (Robot->NA * sizeof(ANGLE))))
        IKerror (18, FATAL, "init_ARM");
  Robot->Weights= mat_malloc (Robot->NA, Robot->NA);
  for(i=0; i<Robot->NA; i++)
    for(j=0; j<Robot->NA; j++)
      Robot->Weights->p[i][j] = 0.0;
  go_next(ARM);         /* find next exclamation mark       */
                        /* Assign Angle info to Angles      */
  for (i=0; i<Robot->NA; i++) {
    fscanf(ARM,"%f", &(Robot->Angles[i].Max_limit));
    fscanf(ARM,"%f", &(Robot->Angles[i].Min_limit));
    Robot->Angles[i].Prism = rm_blanks(ARM);
    if ((Robot->Angles[i].Prism == 'N') || (Robot->Angles[i].Prism == 'n'))
         Robot->Angles[i].Prism = FALSE;
    else Robot->Angles[i].Prism = TRUE;
    fscanf(ARM,"%f", &(Robot->Weights->p[i][i]));
    fscanf(ARM,"%f", &(Robot->Angles[i].Home));
  }

  go_next(ARM);         /* find next exclamation mark       */
                        /* Determine if Robot has platform  */
  temp_i = rm_blanks(ARM);


  /*if the next parameter is "e", read optional EE control*/
  if ((temp_i == 'e') || (temp_i == 'E'))
    {
      go_next(ARM);
      fscanf(ARM,"%f", &EEzaxis);
      fscanf(ARM,"%f", &EEyaxis);
      fscanf(ARM,"%f", &EExaxis);
      EEzaxis =  (EEzaxis*PI / 180.0);     /* convert to radians */
      EEyaxis = (EEyaxis*PI/ 180.0);       /* convert to radians */
      EExaxis =   (EExaxis*PI  / 180.0);   /* convert to radians */
      go_next(ARM);
      temp_i = rm_blanks(ARM);
    }

  if ((temp_i == 'y') || (temp_i == 'Y'))
    Robot->PLAT.Exist = TRUE;
  else Robot->PLAT.Exist = FALSE;

  go_next(ARM);         /* find next exclamation mark       */
                        /* If Platform exists, Assing info  */
  if (Robot->PLAT.Exist) {
     fscanf( ARM,"%f", &( Robot->PLAT.Length  ) );
     fscanf( ARM,"%f", &( Robot->PLAT.Width   ) );
     fscanf( ARM,"%f", &( Robot->PLAT.Thick   ) );
     go_next(ARM);
     fscanf( ARM,"%f", &( Robot->PLAT.Z_OFF   ) );
     fscanf( ARM,"%f", &( Robot->PLAT.L_OFF   ) );
     fscanf( ARM,"%f", &( Robot->PLAT.ANG_OFF ) );
  }
  Robot->PLAT.Holonomic = 1;   /* Currently all platforms assumed holonomic */

  /*********************************************************
   *                                                       *
   *  Print out all data attained from Manipulator file    *
   *                                                       *
   *********************************************************/

  #ifndef FSP_DEBUG
  if (DEBUG) {
    fprintf(stderr, "\n NL: %d", Robot->NL);
    for (i=0; i<Robot->NL; i++) 
      fprintf(stderr, "\n LINK[%d]: %f", i, Robot->LINKS[i]);
    fprintf(stderr, "\n NA: %d", Robot->NA);
    for (i=0; i<Robot->NA; i++) 
      fprintf(stderr, "\n%3d %s %s %9.2f %s %9.2f %s %s %s %6.4f %s %7.2f",
	i, ":",
	"Max:"   ,Robot->Angles[i].Max_limit,
	"Min:"   ,Robot->Angles[i].Min_limit,
	"Prism:" ,(Robot->Angles[i].Prism) ? "YES" : "NO ",
	"Weight:",Robot->Weights->p[i][i]   ,
	"Home:"  ,Robot->Angles[i].Home       );

    fprintf(stderr, "\n PLAT: %s", Robot->PLAT.Exist ? "Yes" : "No");
    fprintf(stderr, "\n   Length   : %f", Robot->PLAT.Length); 
    fprintf(stderr, "\n   Width    : %f", Robot->PLAT.Width);
    fprintf(stderr, "\n   Thickness: %f", Robot->PLAT.Thick);
    fprintf(stderr, "\n   Z_OFF    : %f", Robot->PLAT.Z_OFF);
    fprintf(stderr, "\n   L_OFF    : %f", Robot->PLAT.L_OFF);
    fprintf(stderr, "\n   ANG_OFF  : %f\n\n", Robot->PLAT.ANG_OFF);
  }
  #endif

  fclose(ARM);
}





