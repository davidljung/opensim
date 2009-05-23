/* ________________________________________________________
  |                                                        |
  | Program Name:   short.c                                |
  |________________________________________________________|
  |                                                        |
  | description: Aquires Betas for constraints and det-    |
  |     ermine the Analytical solution for each time step. |
  |                                                        |
  | Procedures:                                            |
  |    main            :                                   |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Header File to link all others */

/* ________________________________________________________
  | name:    main                                          |
  | description:  launches system with as little as poss-  |
  |     possible.  If it is possible to launch with less,  |
  |     make it so.  This is a short driver that enables   |
  |     a command-line approach to the IKOR-FSP system.    |
  |     see help file for more information.                |
  |                                                        |
  | inputs:  The file info which contains help.            |
  | outputs: Calls IKOR_driver with proper parameters.     |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     10/94    Created		           |
  |________________________________________________________| */

int main (int argc, char **argv)
{
  int   i, j,			 /* counter variable                       */
        IKCriterion[2]={-1, -1}, /* 0-FSP/LN, 1-Pseudo, 2-mindx, 3-mindist */
				 /* 4-FLOW				   */
	IKMethod[5]={0,0,0,0,0}, /* 0-JL, 1-OBS, 2-BA, 3-EEdir		   */
 	MethodChosen=FALSE,
	Optimization=LAGRANGIAN, /* BANGBANG, SIMPLEX, LAGRANGIAN	   */
        IKstatus,   		 /* return status from IKOR_driver         */ 
        Platform = FALSE,
        Orient   = FALSE;
  char  *temp, *plat_type, *ornt_type, temp2[2]={ '\0','\0' };

  for (i=1; i<argc; i++)
  {
    switch (argv[i][0]) {
         case 'm':
         case 'M': temp = argv[i] +1; M        = atoi(temp); break;
         case 'n':
         case 'N': temp = argv[i] +1; N        = atoi(temp); break;
         case 't':
         case 'T': temp = argv[i] +1; temp2[0]=temp[0];
		   IKCriterion[0] = atoi(temp2); 
		   if (strlen(argv[i]) > 2)
  		   { temp = argv[i] +2; IKCriterion[1] = atoi(temp); }
		   break;
         case 's':
         case 'S': temp = argv[i] +1; STEP     = atoi(temp); break;
         case 'f':
         case 'F': MethodChosen = TRUE;
		   for (j=1; j<strlen(argv[i]); j++)
                     switch (argv[i][j]){
			case 'l':
			case 'L': IKMethod[JN_LIMITS] = TRUE; break;
			case 'o':
			case 'O': IKMethod[OBSTACLES] = TRUE; break;
			case 'b':
			case 'B': IKMethod[ACCELRATN] = TRUE; break;
                        case 'i':
                        case 'I': IKMethod[EE_IMPACT] = TRUE; break;
			case 'e':
			case 'E': temp = argv[i]+j+1;
                                  IKMethod[4] = atoi(temp); break;
                        }; break;
         case 'o':
         case 'O': ornt_type = argv[i] +1; Orient   = TRUE; break;
         case 'p':
         case 'P': plat_type = argv[i] +1; Platform = TRUE;  break;
         case 'd':
         case 'D': DEBUG           = TRUE; break;
         case 'c':
         case 'C': CHANGE_SPHERES  = TRUE; break;

	 default : fprintf(stderr,"Unknown argument! \n"); help();
                   break;
    }
  }

  Init_Globals(); /* Mostly for CheezyIGRIP & Manipulator Initialization */
  if (Orient) if (ornt_type[0]=='1') Orient = ZEROD_OMEGAS;
  if (Platform)
    if ((plat_type[0] == 'N')||(plat_type[0] == 'n')) 
	Robot->PLAT.Holonomic=FALSE; 
  if (M > Robot->NA) { IKerror (16, OK, "short.c"); M == -1; }
  if (N > 6)         { IKerror (17, OK, "short.c"); N == -1; }
  if ((STEP != ONE) && (STEP != TWO))
    { IKerror(23, OK, "short.c"); STEP = ONE; }
  Robot->Orient      = Orient;
  Robot->PLAT.Active = Platform;
  if ((Platform) && (!Robot->PLAT.Exist)) 
  { IKerror(24, OK, "ROBOT.dat"); Robot->PLAT.Active = FALSE; }
  fprintf(stderr, "NL: %d, NA: %d, Platform?: %s, Orient: %s.\n",
		   Robot->NL, Robot->NA, Robot->PLAT.Active ? "Yes" : "No", 
                         Robot->Orient      ? "Yes" : "No");

  /* set default joint space.*/
  if ((M==-1) && (Robot->PLAT.Active)) M = Robot->NA;
  else if (M==-1)                      M = Robot->NA - 3;  /*lst 3 are pltfrm*/
  /* set default task space. */
  if     ((N==-1) && (Orient)) N = 6;  
  else if (N==-1)              N = 3;

  if (IKCriterion[0]==4)
     switch (IKCriterion[1]) {
	case 1: Optimization = LAGRANGIAN; break;
	case 2: Optimization = BANGBANG;   break;
	case 3: Optimization = SIMPLEX;    break;
	default: fprintf(stderr,"\nParameter for Least Flow Optimization %s",
		"does not exist.  Choices are (1,2, or 3)\n");
		IKerror(31,FATAL,"No such Optimization");
     }
  if (IKCriterion[0]==-1) IKCriterion[0] = 0;      /* default: FSP   */

  fprintf(stderr, "IKOR Parameters: N: %d, M: %d, Plat: %s, Orient: %s.\n",
		   N, M, Robot->PLAT.Active ? "Yes" : "No", 
                         Robot->Orient      ? "Yes" : "No");

  /* begin driver */
  IKstatus = IKOR_driver(IKCriterion, IKMethod, Optimization, N, M);

  switch (IKstatus) {
    case -2 :   fprintf(stderr,"\nSome kind of file error, sorry!\n");
		break;
    case -1 :   fprintf(stderr,"\nFile completion. Partial Trajectory Only\n");
		break;
    default :   fprintf(stderr,"\nFile completion. Full Trajectory\n");
		break;

   return (IKstatus);
  } /* end switch */
} /* end short */



int help( void )
{
  FILE *temp;      /* used to point to help file             */
  char  one;       /* used to read help file                 */


  if ((temp = fopen("../COMPILE/IKOR/inform", "r")) == NULL) 
     fprintf(stderr, "INFO FILE GONE: COMPILE/IKOR/inform. Use at own RISK!\n");
  else 
     while (!feof(temp)) fputc(fgetc(temp), stderr);
  if (!temp) fclose (temp);

  exit(-1);
}
