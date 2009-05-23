/* ________________________________________________________
  |                                                        |
  | Program Name:   FSPv2.0.c                              |
  |________________________________________________________|
  |                                                        |
  | description:  Given the Jacobian and dx the procedure  |
  |     finds a a set of g vectors which span the solution |
  |     space of the equation to the equation J*q=dx, for  |
  |     all values of the vector q.                        |
  |                                                        |
  |                                                        |
  | Procedures:                                            |
  |     Solution_generator  :                              |
  |     RestofSoln          :                              |
  |     Dependency          :                              |
  |     BLOCK_COL_FIND_X    :                              |
  |     ReduceA             :                              |
  |     CheckB              :                              |
  |     CheckRange          :                              |
  |     Rebuild_gs          :                              |
  |________________________________________________________| */

#include <IKOR/general.h>

int  SystemComplete;	/* marks when all necessary vecs have been found*/
int  DEBUG_FSP = TRUE;
#define DEBUG 1
/* ________________________________________________________
  | name:    Solution_generator                            |
  | description:  Main FSP procedure which sets up all     |
  |    initialization of variables, and starts production  |
  |    of g vectors.  After calling reduce it checks for   |
  |    SpecialCase2 then finds the first valid blocking    |
  |    pattern.  After which it calls RestofSoln and       |
  |    depending on the outcome it decides which method to |
  |    determine the solution vector.                      |
  |                                                        |
  |                                                        |
  | inputs:  Aorig, borig, M, N                            |
  | outputs: Ared, bred, Mred, Nred, Xelim, ColElim,       |
  |          RowElim, g                                    |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  g fries     3/95      Created		           |
  |________________________________________________________| */

extern FILE* gcheck;     

int Solution_generator(FSP_data, Aorig, borig, check)
FILE      *check;
MATRIX    *Aorig,
	  *borig;
Solutions *FSP_data;
{
MATRIX	*Ared,		/* reduced A					*/
	*Asub,		/* submatrix from square submatrix is found	*/
	*Asqr,		/* square submatrix				*/
	*bred,		/* reduced b					*/
	*block,		/* locations of blocked columns for each soln	*/
	*n,		/* null space vectors				*/
	*Specialg;      /* null vectors created by SpecialCase1         */
float	K2,		/* matrix condition number:  sv_max/sv_min	*/
	N_BND;		/* min acceptable value for nonzero nspace comp */
int	i, j, k, I,	/* loop counters				*/
	bcheck,		/* check for completely zero bred		*/
       *ColElim,	/* marks columns eliminated from original A	*/
       *RowElim,	/* which rows are to be eliminated from the A	*/
	move,		/* which of four possible blocks is being moved	*/
	NextToFind,	/* vector number being searched for		*/
       *Tackon,		/* columns are indexed for effic, soln finding	*/
       *FirstOK,        /* if OK to substitute in for firstsoln column  */
	pos,		/* used with the Ordering[] variable		*/
	nullity,	/* dimension of null space			*/
	binrange,	/* marks if b is in the range of the A		*/
	Exit_Status,    /* tells calling routine if OK			*/
	NumSpg,         /* number of g vectors created by SpecialCase1  */
	Stop;           /* loop flag to end                             */

  /* allocate memory space for variables */
  Tackon  = ( int *) malloc ( (M - N) * sizeof( int ) );
  FirstOK = ( int *) malloc (    N    * sizeof( int ) );
  ColElim = ( int *) malloc (    M    * sizeof( int ) );
  RowElim = ( int *) malloc (    N    * sizeof( int ) );
  Ared	  = mat_malloc(N, M);
  bred	  = mat_malloc(N, 1);
  Specialg= mat_malloc((M-1),M);

  gcheck=check;
  /* initialize SystemComplete and FirstOK */
  SystemComplete = FALSE;
  for (i=0; i< N; i++)
    FirstOK[i] = FALSE;

  if ( (bcheck = (CheckB(borig, FSP_data->N))) == ZERO )
  {
    for (i=0; i< FSP_data->N; i++) 
      borig->p[i][0] = (i<3) ? 1.0:0.0;
    FSP_data->Null_Space = TRUE;  
  }
  else
    FSP_data->Null_Space = FALSE;

  /*check = stderr;		/* uncomment when in trouble	*/
  /* Eliminate all the nonredundancies from the A and b */
  ReduceA(FSP_data, Aorig, Ared, borig, bred,ColElim,RowElim,
							&NumSpg, Specialg);
  /* Setup and initialization of g vectors */
  mat_free(FSP_data->g);
  FSP_data->g = mat_malloc((FSP_data->Mred-FSP_data->Nred+1+NumSpg), M);
  for (i=0; i<(SPAN); i++)
    for (j=0; j<M; j++) FSP_data->g->p[i][j] = 0.0e0;


if (DEBUG) {
  fprintf(check,"\n  ______________________________  \n");
  fprintf(check,"\n                FSP               \n");
  fprintf(check,  "  ______________________________  \n\n");
  fmat_pr(check, " Jacobian ", Aorig);
  fmat_pr(check, " requested dx ", borig); 
  fprintf(check, " M   : %d, N   : %d\n Mred: %d, Nred: %d",
		   M, N, FSP_data->Mred, FSP_data->Nred);
  fprintf (check, "\nRowElim: ");
  for (i=0; i< N; i++) fprintf (check,"%d ", RowElim[i]);
  fprintf (check, "\nColElim: ");
  for (i=0; i< M; i++) fprintf (check,"%d ", ColElim[i]);
  fmat_pr(check, " *** Xelim *** ", FSP_data->Xelim);
  if(FSP_data->Null_Space) fprintf(check,"\nGenerating NullSpace Solutions.\n");
}
if (DEBUG_FSP) {
  fmat_pr(check, " *** A Reduced *** ", Ared);
  fmat_pr(check, " *** b Reduced *** ", bred);

}             /* if DEBUGGING, include code  (see file info) */

bcheck=CheckB(bred, FSP_data->Nred);
if (bcheck == 0)
    {
    n = mat_malloc(FSP_data->Mred, FSP_data->Nred);
    if (DEBUG_FSP) fprintf(check, "GIVEN MATRIX IS COMPLETELY RESTRICTED\n\n");
    Asub = mat_malloc(FSP_data->Nred, FSP_data->Mred);
    for (i=0; i < FSP_data->Nred; i++)
	for (j=0; j < FSP_data->Mred; j++)
	    Asub->p[i][j] = Ared->p[i][j];
    mat_null(Asub, &nullity, n, &K2);

    /* set the solution vectors for a completely restricted system  */
    for (i=0; i < FSP_data->Mred; i++)
	for (j=0; j< SPAN2 - 1; j++)
	    FSP_data->g->p[j][i] = n->p[i][j];
    for (i=0; i < FSP_data->Mred; i++)
	FSP_data->g->p[FSP_data->Mred-FSP_data->Nred][i] = 0.0;
    SystemComplete = TRUE;

    Exit_Status = RESTRICTED;
    if (DEBUG_FSP) fmat_pr(check, "solutions", FSP_data->g);

    mat_free(Asub);
    mat_free(n);
    }
else
    {
    n = mat_malloc(FSP_data->Nred, FSP_data->Nred);
    if (FSP_data->Nred != N)
	{
	    if (DEBUG_FSP) {
              fprintf(check, "GIVEN MATRIX IS RESTRICTED FOR %d ",
		 N-FSP_data->Nred);
	      fprintf(check, "OUT OF %d VECTOR COMPONENTS\n\n", N);
	    }
        }

    block= mat_malloc(SPAN2,FSP_data->Nred);
    Asqr = mat_malloc(FSP_data->Nred,FSP_data->Nred);
    NextToFind = 0;

    /* initialize the blocking positions for the first solution vector */
    if (DEBUG_FSP) 
	for (i=0; i<(FSP_data->Nred); i++) 
	     for (j=0; j<(FSP_data->Mred-FSP_data->Nred+1); j++)
		block->p[j][i] = 0;

    for (i=0; i<(FSP_data->Nred); i++)
	block->p[0][i] = i;
    k=0;
    for (i=0; i<FSP_data->Mred; i++)
      if (block->p[0][k] == i)
         {
         for (j=0; j<FSP_data->Nred; j++)
            Asqr->p[j][k] = Ared->p[j][i];
	 k++;
         }
    mat_null(Asqr, &nullity, n, &K2);

    if (DEBUG_FSP) {
	fmat_pr (check, "block", block);
	fmat_pr (check, "Asqr", Asqr);
	fmat_pr (check, "n", n);
	fprintf(check,"nullity = %d, K2 = %f\n", nullity, K2);
    }     /* if DEBUGGING, include code  (see file info) */

    /* loop until an acceptable well-conditioned first solution found */
    while ((nullity != 0) && (block->p[0][0]<N))
	{
fprintf(check,"while:\n");
	pos = FSP_data->Nred-1;
	while (block->p[0][pos] == SPAN2 - 1 +pos)
	    pos--;
	block->p[0][pos]++;
	move=pos+1;
	while (move<FSP_data->Nred)
	    {
	    block->p[0][move]=block->p[0][(move-1)]+1;
	    move++;
	    }
	k=0;
	for (i=0; i<FSP_data->Mred; i++)
	    if (block->p[0][k] == i)
		{
		for (j=0; j<FSP_data->Nred; j++)
		    Asqr->p[j][k] = Ared->p[j][i];
		k++;
		}
	if (block->p[0][0] <= FSP_data->Nred)
	    mat_null(Asqr, &nullity, n, &K2);
	if (DEBUG_FSP) {
	    fmat_pr(check, "block", block);
            fmat_pr(check, "Asqr", Asqr);
            fprintf(check, "nullity = %d, K2 = %f\n", nullity, K2);
	  }
	}

    k=0; I=0;
    for (i=0; i<FSP_data->Mred; i++)
	if ((k < FSP_data->Nred) && (block->p[0][k] == i))
	   k++;
	else
	    {
	    Tackon[I]=i;
	    I++;
	    }
fprintf(check,"calling BLOCK_COL_FIND_X\n");
    BLOCK_COL_FIND_X(Tackon,FSP_data->g->p[NextToFind],block->p[NextToFind],
		bred,Ared,FSP_data->Mred,FSP_data->Nred,check);

    if (DEBUG_FSP) {
	fprintf(check, "first solution \n");
	fmat_pr(check, "block", block);
        fmat_pr(check, "Asqr", Asqr);
        for (i=0; i<FSP_data->Mred-FSP_data->Nred;i++)
  	  fprintf(check, "here %d", Tackon[i]);
    }
    fprintf(check,"Nred=%d Mred=%d\n",FSP_data->Nred, FSP_data->Mred);
    if (FSP_data->Nred==FSP_data->Mred)
	SystemComplete=TRUE;
    else
        RestofSoln(FSP_data->Mred, FSP_data->Nred, NextToFind, block,
		   FSP_data->g, bred, Ared, Tackon, FirstOK, check);

    if (DEBUG_FSP) {
      fmat_pr(check, "block", block);
      fmat_pr(check, "solutions", FSP_data->g);
      }
   mat_free(block);
   mat_free(Asqr);
   mat_free(n);
   }



        /* check if the eliminated b elements in bred can be	*/
        /* produced from the g vectors and the reduced A matrix	*/
        /* ie check that b is in the range of A			*/
   if (!SystemComplete)
   {
	IKerror(25,OK,"");
        binrange = 0;
   }
   else
      binrange = CheckRange(borig, Aorig, FSP_data->g, RowElim,
			    FSP_data->Mred);

        /* check that the original b was not all zeros (special case) */
   bcheck = CheckB(borig, M);

        /* check whether the original A had dependent rows  */
   if ((binrange) && (bcheck != 0) && (Exit_Status != RESTRICTED))
   {
     Rebuild_gs (FSP_data, ColElim, Specialg, NumSpg);
     Exit_Status = COMPLETE;
     for (i=0; (( i < N ) && (FSP_data->cn == 0)); i++)
     {
       if (RowElim[i] != 2)
         i++;
       else {
	 /* Code should be inserted here or in ReduceA to determine     */
	 /* whether or not the b's are dependent as well, if they are   */
 	 /* nothing needs to happen, if they aren't... an error mess-   */
	 /* should be generated and the system should halt.	        */
	 /* Currently this will occur since the A matrix in the findt's */
	 /* will be the zero matrix and cause a zero division error.    */	
       }
     }
   }
   else if (Exit_Status != RESTRICTED)
   {
     Exit_Status = NOT_COMPLETE;
     if (bcheck != 0)
       fprintf(check, "\n\nB NOT IN RANGE OF A\n");
     else
       fprintf(check, "\n\n%s\n%s", "SPECIAL CASE\nB IS ZERO VECTOR",
       	 "CONSTRAINED OPTIMIZATION MUST BE USED\n");
   }

/* free up all variable memory space */
mat_free(Ared);
mat_free(bred);
mat_free(Specialg);
free(Tackon);
free(FirstOK);
free(ColElim);
free(RowElim);

return (Exit_Status);
}



/* ________________________________________________________
  | name:    RestofSoln                                    |
  | description:  Finds the remaining solution vectors     |
  |               after the first has been selected.  It is|
  |               recursive and thus calls itself when a   |
  |               valid solution is found, if this solution|
  |               leads to a dead end, it will pop back to |
  |               last solution and build from there.  This|
  |               insures that all possible combinations of|
  |               blocking patterns are available, given   |
  |               first one, in pattern which follows the  |
  |		  algoritm presented in the article.       |
  |                                                        |
  |                                                        |
  | inputs:  Mred, Nred, NextToFind, block, g, bred, Ared, |
  |          Tackon, FirstOK                               |
  | outputs: block, g                                      |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  g fries     2/95      Created		           |
  |  g fries     3/95      Addition of SpecialCase2 case   |
  |________________________________________________________| */

void RestofSoln (int Mred, int Nred, int NextToFind, MATRIX *block,
		 MATRIX *g, MATRIX *bred, MATRIX *Ared, int *Tackon,
		 int *FirstOK, FILE *check)
{
int     i, j, k, /*Loop counters                                      */
        X, Y,    /*Position markers, keeping track of the current     */
		 /*column to be subbed in for and which one to sub in */
        nullity, /*Nullity sent to mat_null for dependent rows        */
        Newtack[M-N], /*Temp array for tackon, sent to the next level of   */
		 /*recursion                                          */
       *Changed; /*Temp array for FirstOK, sent to the next level of  */
		 /*recursion                                          */
MATRIX	*n,	 /*null space vectors				      */
	*Asub;	 /*submatrix from square submatrix is found	      */
float K2;

Changed = (int *) malloc (  Nred * (sizeof( int )) );
 fprintf(check, "entering RestOfSoln\n");
/*Increment NextToFind so looking for next solution */
NextToFind++;
X = NextToFind -1;

/*Change FirstOK if a previously 0 value of g for a column in first  */
/*blocking pattern has become nonzero                                */
for (i=0; i<Nred; i++)
  if ((!FirstOK[i]) && 
  (fabs(g->p[NextToFind-1][(int) block->p[NextToFind-1][i]]) > SMALL))
  {
     FirstOK[i] = TRUE;
  }

for (j=0; j<Nred; j++)
{
  if (DEBUG_FSP) {
     if (j==0) fprintf(check,"\nFIRST OK:  ");
     fprintf(check," %d ", FirstOK[j]);
     if (j==Nred-1) fprintf(check,"\n\n");
  }
  Changed[j] = FirstOK[j];
}

while ((X<(Mred-Nred))&&(!SystemComplete))
    {
    if (DEBUG_FSP) 
      fprintf(check,"\n Next Solution to find is: %d \n", NextToFind);
    

    /*Set up next asub comparing the next remaining column to be subbed */
    /*in with the blocking pattern for that level of recursion          */
    Asub = mat_malloc(Nred,Nred+1);
    for (i=0; i<Nred; i++)
	for (j=0; j<Nred; j++)
	    Asub->p[j][i] = Ared->p[j][(int)block->p[(NextToFind-1)][i]];
    for (j=0; j<Nred; j++)
      Asub->p[j][Nred] = Ared->p[j][Tackon[X]];
    if (DEBUG_FSP) fmat_pr(check, "Asub", Asub);

    n = mat_malloc(Nred+1,1);
    mat_null(Asub, &nullity, n, &K2);
    mat_free(Asub);
    if (DEBUG_FSP) fmat_pr(check, "n", n);
    Y=0;

    /*Loop which compares column to be subbed in with current blocking */
    /*pattern for dependency, if dependent it completes swap and valid */
    while ((Y<(Nred))&&(!SystemComplete))
	{
        if (DEBUG_FSP) {
  	  fprintf(check,"\n Compare column %d with %d \n", Tackon[X], 
					(int)block->p[(NextToFind-1)][Y]);
	  fprintf(check, "\n it is %f \n",fabs(n->p[Y][0]));
	}
	if ((fabs(n->p[Y][0]) > SMALL) && (FirstOK[Y]))
	    {
	    if (DEBUG_FSP) 
	      fprintf(check, "*************** %d **************** \n", 
								NextToFind);
	    fmat_pr(check,"\n** block",block);
	    fprintf(check,"** Tackon=");
	    for(j=0; j<(M-N); j++)
	      fprintf(check,"%d ",Tackon[j]);
	    fprintf(check,"\nX=%d Y=%d\n",X,Y);

	    for (i=0; i<Nred;i++)
		block->p[NextToFind][i]=block->p[(NextToFind-1)][i];
	    Newtack[0]=(int)block->p[NextToFind][Y];
	    block->p[NextToFind][Y] =Tackon[X];
	    for (i=0; i<X; i++)
		Newtack[i+1]=Tackon[i];
	    for (j=X+1; j<(Mred-Nred); j++) {
		Newtack[j]=Tackon[j];	
	    }

	    fprintf(check,"** Newtack=");
	    for(j=0; j<(Mred-Nred); j++)
	      fprintf(check,"%d ",Newtack[j]);
	    fmat_pr(check,"\n** block",block);
	    fmat_pr(check,"** g",g);

	    BLOCK_COL_FIND_X(Newtack,g->p[NextToFind], block->p[NextToFind], 
						bred,Ared,Mred,Nred,check);

	    fmat_pr(check,"** block",block);
	    fmat_pr(check,"** g",g);

	    if (DEBUG_FSP) 
	      fprintf(check, "\n it will be %f \n",fabs(g->p[(NextToFind)] 
				[(int)block->p[NextToFind][Y]]));

	    /*Check to ensure the newly created g vector is independent */
	    /*of previously created ones in branch                      */
	    if (fabs(g->p[(NextToFind)][(int)block->p[NextToFind][Y]]) >SMALL)
		if (NextToFind <(Mred-Nred))
		    {
		    if (DEBUG_FSP) {
			fmat_pr(check, "block", block);
			fmat_pr(check, "solutions", g);
    			for (i=0; i<(Mred-Nred); i++)
			    fprintf(check, "Ord %d",Newtack[i]);
    		    }
		    /*if more g vectors are needed it goes to next level*/
		    RestofSoln(Mred, Nred, NextToFind, block, g, bred, Ared, 
			       Newtack, Changed, check);
		    }
		else
		    {
		    SystemComplete = TRUE;
		    }
	    if (!SystemComplete)
		block->p[NextToFind][Y]=Newtack[0];
	    if (DEBUG_FSP) fprintf(check, "\n **************************** \n");
	    }
	Y++;
	}
   mat_free(n);
    X++;
    }

  free (Changed);
}



/* ________________________________________________________
  | name:    Dependency                                    |
  | description:  Returns whether or not the given two     |
  |               columns are dependent upon each other or |
  |               not.  Essential for detection of an      |
  |               occurance of SpecialCase1.               |
  |                                                        |
  |                                                        |
  | inputs:  Aorig, SLRow, Nred, first, second             |
  | outputs: whether columns SLRow[first] and SLRow[second]|
  |          are dependent                                 |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  g fries     3/95      Created		           |
  |________________________________________________________| */

int Dependency (MATRIX *Atemp, int *SLRow, int Nred, int first, int second)
{
int i,STOP;
float devidor;
i=0;
devidor=0;
STOP=FALSE;
while (i<Nred && !STOP)
    if ((fabs(Atemp->p[SLRow[i]][first])<SMALL) && 
				(fabs(Atemp->p[SLRow[i]][second])<SMALL))
	i++;
    else if ((fabs(Atemp->p[SLRow[i]][first])>SMALL) &&
				(fabs(Atemp->p[SLRow[i]][second])>SMALL))
	if (devidor==0)
	    {
	    devidor= (Atemp->p[SLRow[i]][first]/Atemp->p[SLRow[i]][second]);
	    i++;
	    }
	else if (fabs(Atemp->p[SLRow[i]][first]-
			(Atemp->p[SLRow[i]][second]*devidor))<SMALL)
	    i++;
	else
	    STOP=TRUE;
    else
	STOP=TRUE;
if (STOP)
    return(FALSE);
else
    return(TRUE);
}




/* ________________________________________________________
  | name:    BLOCK_COL_FIND_X                              |
  | description:  This procedure calculates a g vector for |
  |               a given square blocking pattern.         |
  |                                                        |
  |                                                        |
  | inputs:  Tackon, g, block, bred, Ared, Mred, Nred      |
  | outputs: new g vector                                  |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  k morganson 8/94      Created		           |
  |  g fries     2/95      Adaption for new blocking       |
  |			   strategy (keeping blocked col)  |
  |________________________________________________________| */

BLOCK_COL_FIND_X	(int *Tackon, float *g, float *block,
			 MATRIX *b, MATRIX *A, 
 			 int Mred, int Nred, FILE *check)
{
int r, I, i, j,k;
float  *blocktemp, temp;
MATRIX *Atemp, *gtemp, *btemp;

Atemp = mat_malloc(Nred,Nred);
btemp = mat_malloc(Nred,1);
blocktemp = (float *) malloc( Nred * sizeof(float) );

for (i=0; i<Nred; i++)
   btemp->p[i][0]=b->p[i][0];
/* the columns blocked need to be listed from smallest to largest */
for (i=0; i<(Nred); i++)
   blocktemp[i]=(int)block[i];  


for (i=(Nred-1); i>0; i--)
   for (j=i-1; j>=0; j--)
      if (blocktemp[i]<blocktemp[j])
         {
         temp=blocktemp[i];
         blocktemp[i]=blocktemp[j];
         blocktemp[j]=temp;
         }


fprintf(gcheck,"Mred=%d Nred=%d\n",Mred,Nred);
   k=0; I=0;
   for (i=0; i<Mred; i++) {
      if (blocktemp[k] == i)
         {
         for (j=0; j<Nred; j++)
            Atemp->p[j][k] = A->p[j][i];
	 k++;
         }
      else
	I++;
   }
if (DEBUG_FSP) fmat_pr(check, "asqr", Atemp);


mat_LU_inv(Atemp);

gtemp = mat_mul2(Atemp, btemp);

if (DEBUG_FSP) {
  fprintf(check, "\n gvector");
  for (i=0; i<Nred; i++)
    fprintf(check, "%f    ", gtemp->p[i][0]);
}

/*Now add a zero to where column was blocked */
j=0;
I=0;
for (i=0; i<Nred; i++)
   g[(int)blocktemp[i]]=gtemp->p[i][0];

for (j=0; j<(Mred-Nred); j++)  
      g[Tackon[j]]=0.0e0;

 fprintf(check,"\n** BLOCK_COL_FIND_X grow="); 
 for (j=0; j< N; j++)
   fprintf(check, "%f    ",g[j]);

mat_free(Atemp);
mat_free(gtemp);
mat_free(btemp);
free (blocktemp);
}



/* ________________________________________________________
  | name:    ReduceA                                       |
  | description:  Restricted work space motions can be     |
  |               identified by rows of the A which only   |
  |               have one nonzero element.  Since the     |
  |               corresponding column must be present in  |
  |               all final joint space solutions, the     |
  |               appropriate joint space motion will be   |
  |               calculated before any redundancy         |
  |               resolution is performed, and the         |
  |               appropriate motions and joints will be   |
  |               removed from the work space and A        |
  |               respectively.  Also such cases as        |
  |               dependent rows, and SpecialCase1 must be |
  |               identified and dealt with.               |
  |                                                        |
  |                                                        |
  | inputs:  Tackon, g, block, bred, Ared, Mred, Nred      |
  | outputs: new g vector                                  |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  k morganson 8/94      Created		           |
  |  g fries     2/95      Adaption for new blocking       |
  |			   strategy and handling of dep row|
  |  g fries     3/95      Recognition and handling of     |
  |                        SpecialCase1 and cont reduction |
  |  c hacker    9/95      the new b was wrong, corrected  |
  |  c hacer    10/95      dependent rows bug fixed	   |
  |________________________________________________________| */

ReduceA(Solutions *FSP_data, MATRIX *Aorig, MATRIX *Ared, MATRIX *borig,
	MATRIX *bred, int* ColElim,int* RowElim,
        int *NumSpg, MATRIX *Specialg)

{
int i,j,k,m,r,p,   /*loop counters					*/
    StillChecking, /*flag to mark when all nonredundancies are gone	*/
    Stop,Stop2,    /*Used as loop flag in detection for SpecialCase1&2  */
    ClassANum,     /*Number of ClassificationA columns, for SpecialCase2*/
    reference,     /*Reference position used for SpecialCase2 row       */
    LastNred,      /*Check if all rows have been looked at              */
    nullity,       /*Used in mat_null when determining depend rows      */
    nul_count,     /*Loop counter for dependent rows                    */ 
    Restriction,   /*num of joints which contrib to a work space d.o.f.	*/
    DoneRed,       /*Keeps track if reduction is done, loop has executed*/
		   /*without any reduction                              */
    *SLRow,        /*Stands for Still Left Row, stores remaining non    */
		   /*eliminated rows in sequence                        */
    *SLCol,        /*Stands for Still Left Column, stores remaining non */
		   /*eliminated columns in sequence                     */
    *ClassA,       /*Array used to store ClassificationA columns        */
    count,count2,  /*Position markers when copying over SLCol when the  */
    position,      /*  SpecialCase1 has been found                      */
    nonzerocol,    /*Number of nonzero col when check for SpecialCase1  */
    *Nonzero,      /*Stores values of nonzero col when check for SC1    */
    NonZeroCol,	   /*Column which has nonzero element for give row	*/
    Flag,	   /*Used in SC2 for an undefined beta value            */
    Goin;          /*"Go in" for entance into ClassA for SC2            */
float btemp[N],    /*Holds the b vector as it is modified by backsub	*/
      K2,	   /*Matrix condition number:  sv_max/sv_min		*/
      *Constant,   /*Constant values used when comparing ClassificationA*/
                   /*columns                                            */
      beta;        /*Value used for determining SpecialCase2            */
MATRIX	*n,	   /*null space vectors				        */
	*Atrans,   /*Transpose matrix used when looking for depend rows */
	*AElim;    /*Matrix used to find null vectors when dealing with */
		   /* SpecialCase1                                      */

     /* allocation of memory */
SLRow   = (int   *) malloc ( (N) * (sizeof( int )) );
SLCol   = (int   *) malloc ( (M) * (sizeof( int )) );
Nonzero = (int   *) malloc ( (M) * (sizeof( int )) );
ClassA  = (int   *) malloc ( (M) * (sizeof( int )) );
Constant= (float *) malloc ( (N) * (sizeof( float )) );

printf("here0 N=%d M=%d\n",N,M);

     /* initialize variables */
FSP_data->Nred = N;    /* number of rows in the reduced A	*/
FSP_data->Mred = M;    /* number of columns in the reduced A	*/
*NumSpg=0;
for (i=0; i<N; i++)
   {
   SLRow[i]=i;
   RowElim[i]=0;
   btemp[i]=borig->p[i][0];
   }
for (i=0; i<M; i++)
   {
   SLCol[i]=i;
   FSP_data->Xelim->p[i][0]=0.0;
   ColElim[i]=0;
   }
DoneRed=FALSE;


     /* start reduction  */
while (!DoneRed)
    {
    DoneRed=TRUE;
    StillChecking = 1; 
     /* Check each row for the number of nonzero elements.  If only one	*/
     /* element is nonzero, solve for the joint space motion, and	*/
     /* modify the b vector so that the appropriate row and column	*/
     /* can be eliminated from the A.  After a row is eliminated	*/
     /* the remaining A must be rechecked for any new restrictions	*/
    while (StillChecking)
	{
	LastNred = FSP_data->Nred;
	for (i=0; i<FSP_data->Nred; i++)
	    {
	    j=-1;
	    Restriction=0;

	    /* check for nonzero row elements */
	    while ((j<(FSP_data->Mred-1)) && (Restriction < 2)) 
		{
		j++;
		if (fabs(Aorig->p[SLRow[i]][SLCol[j]]) > SMALL)
		    {
		    Restriction++;
		    NonZeroCol = j;
		    }
		}

	    /* if a row only has one nonzero element, eliminate it from */
	    /* the A                                             */
	    if ((Restriction == 1) && (RowElim[SLRow[i]] != 1))
		{
/* HERE CJH */	FSP_data->Xelim->p[SLCol[NonZeroCol]][0]=(FSP_data->Null_Space)?
		      0.0:btemp[SLRow[i]]/Aorig->p[SLRow[i]][SLCol[NonZeroCol]];
		for (k=0; k<FSP_data->Nred; k++)
		    btemp[SLRow[k]]= btemp[SLRow[k]]-Aorig->p[SLRow[k]][SLCol
			[NonZeroCol]]*FSP_data->Xelim->p[SLCol[NonZeroCol]][0];
		ColElim[SLCol[NonZeroCol]]=1;
		RowElim[SLRow[i]]=1;
		for (j=NonZeroCol; j<(FSP_data->Mred-1); j++)
		    SLCol[j]=SLCol[j+1];
		for (j=i; j<(FSP_data->Nred-1); j++)
		    SLRow[j]=SLRow[j+1];
		FSP_data->Nred--;
		FSP_data->Mred--;
		}
	    else            /* row of all zeros, also eliminated */
	    if ((Restriction == 0) && (RowElim[SLRow[i]] != 1))
		{
		RowElim[SLRow[i]]=1;
		for (j=i; j<(FSP_data->Nred-1); j++)
		    SLRow[j]=SLRow[j+1];
		FSP_data->Nred--;
		}
	    }
	if (FSP_data->Nred == LastNred)
	StillChecking = 0;
	}


      /* check for columns of zeros */
    for (i=0; i<FSP_data->Mred; i++)
	{
	j=0;
	while ((j<FSP_data->Nred) && (fabs(Aorig->p[SLRow[j]][SLCol[i]]) <
									SMALL))
	    j++;
	if (j==FSP_data->Nred)
	    { 
	    ColElim[SLCol[i]]=1;
	    for (j=i; j<(FSP_data->Mred-1); j++)
		SLCol[j]=SLCol[j+1];
	    FSP_data->Mred--;
	    i--;
	    }
	}

     /* check for dependent rows */
if (FSP_data->Mred !=0)
    {
    n      = mat_malloc(FSP_data->Mred,FSP_data->Mred-1);
    Atrans = mat_malloc(FSP_data->Mred,FSP_data->Mred);
    for (i=0; i<FSP_data->Nred; i++)
	for (j=0; j<FSP_data->Mred; j++)
	    Atrans->p[j][i] = Aorig->p[SLRow[i]][SLCol[j]];
    for (i=FSP_data->Nred; i<FSP_data->Mred; i++)
	for (j=0; j<FSP_data->Mred; j++) Atrans->p[j][i] = 0;

    mat_null(Atrans, &nullity, n, &K2);
    nullity=nullity+FSP_data->Nred-FSP_data->Mred;


    /* This is where the b's should be checked and determined if they */
    /* are dependent as well.  If they are not, an error message      */
    /* should be displayed and the system should stop.		      */
    /* This is probably the best place to do this, instead of at the  */
    /* bottom of solution_generator(), since it has the information.  */
    nul_count=0;
    while (nul_count < nullity)
	{
	i=FSP_data->Mred-FSP_data->Nred +nul_count;
	j=0;
	while ((j!=(n->rows))&&(fabs(n->p[j][i]) < SMALL))
	    j++;
	if ((RowElim[SLRow[j]] != 1)&&(j!=(n->rows)))  
	    {
	    RowElim[SLRow[j]] = 2;
	    for (k=j; k<(FSP_data->Nred-1); k++)
		SLRow[k]=SLRow[k+1];
	    FSP_data->Nred--;
	    }
    	nul_count++;
    	}
    mat_free(Atrans);
    mat_free(n);
    }


     /* check for SpecialCase1 */
    for (i=0;(i<FSP_data->Nred);i++)
	if (fabs(btemp[SLRow[i]])<SMALL)
	    {
	    nonzerocol=0;
	    j=0;
	    Stop =FALSE;
	    while (j<(FSP_data->Mred-1) && !Stop)
		if (fabs(Aorig->p[SLRow[i]][SLCol[j]])>SMALL)
		    {
		    k=j+1;
		    while (!Stop && (k<FSP_data->Mred))
			if (fabs(Aorig->p[SLRow[i]][SLCol[k]])>SMALL)
			    if (Dependency(Aorig, SLRow, FSP_data->Nred,
					                SLCol[j], SLCol[k]))
				k++;
			    else
				Stop=TRUE;
			else
			    k++;
		    Nonzero[nonzerocol]=SLCol[j];
		    nonzerocol++;
		    j++;
		    }	    
		else
		    j++;
	    if (fabs(Aorig->p[SLRow[i]][SLCol[(FSP_data->Mred-1)]])>SMALL)
		{
		Nonzero[nonzerocol]=SLCol[(FSP_data->Mred-1)];
		nonzerocol++;
		}
	    if (nonzerocol==1)
		Stop=TRUE;
	    if (!Stop) /* Discovered a SpecialCase1 */
		{
		DoneRed=FALSE;
		for (r=0;r<nonzerocol;r++)
		   ColElim[Nonzero[r]]=2;
		count=0;
		count2=0;
		for (position=0; position<FSP_data->Mred; position++)
		    if (SLCol[position]!=Nonzero[count2])
			{
			SLCol[count]=SLCol[position];
			count++;
			}
		    else
			count2++;
		/* Construct new g matrix */
		AElim = mat_malloc(1, nonzerocol);
		n=mat_malloc(nonzerocol, nonzerocol-1);
		for (r=0; r<nonzerocol;r++)
		    AElim->p[0][r]=Aorig->p[SLRow[i]][Nonzero[r]];
		mat_null(AElim, &nullity, n, &K2);

		FSP_data->Mred=FSP_data->Mred-nonzerocol;
		RowElim[SLRow[i]]=1;
		for (r=i; r<(FSP_data->Nred-1); r++)
		    SLRow[r]=SLRow[r+1];
		FSP_data->Nred--;
		for (m=0; m<(nonzerocol-1);m++)
		    {
		    for (r=0; r<M; r++)
			Specialg ->p[*NumSpg][r]=0;
		    for (r=0; r<nonzerocol;r++)
			Specialg ->p[*NumSpg][Nonzero[r]]=n->p[r][m];
		    *NumSpg=*NumSpg+1;
		    }
		mat_free(AElim);
		mat_free(n);
		}
	    }

    /*Check for SpecialCase2*/
    i=0;
    Stop2=FALSE;
    while ((i<(FSP_data->Nred-1)) && (!Stop2))
	{
	j=i+1;
	while ((j<FSP_data->Nred) && (!Stop2))
	    {
	    Stop=FALSE;
	    Flag=FALSE;
	    if (fabs(btemp[SLRow[i]])>SMALL)
		beta= btemp[SLRow[j]]/ btemp[SLRow[i]];
	    else
		Flag=TRUE;
	    ClassANum=0;
	    k=0;
	    reference=i;
	    while ((!Stop) && (k<FSP_data->Mred))
		{
		Goin=FALSE;
		if (fabs(Aorig->p[SLRow[i]][SLCol[k]])<SMALL)
		    if (!Flag)
			Goin=TRUE;
		    else;
		else
		    {
		    if (fabs((Aorig->p[SLRow[j]][SLCol[k]]/
			    Aorig->p[SLRow[i]][SLCol[k]])-beta)>SMALL)
			Goin=TRUE;
		    if (Flag)
			Goin=TRUE;
		    }
		if (Goin)
		    {
		    ClassA[ClassANum]=SLCol[k];
		    if (ClassANum==0)
			{
			if (fabs(Aorig->p[SLRow[reference]][ClassA[0]])<SMALL)
			    {
			    reference=j;
			    if (fabs(Aorig->p[SLRow[reference]][ClassA[0]])
									<SMALL)
				Stop=TRUE;
			    }
			if (!Stop)
			    for (r=0;r<FSP_data->Nred;r++)
				Constant[r]=Aorig->p[SLRow[r]][ClassA[0]]/
					Aorig->p[SLRow[reference]][ClassA[0]];
			}
		    else if (fabs(Aorig->p[SLRow[reference]][ClassA[ClassANum]])
									<SMALL)
			Stop=TRUE;
		    else
			{
			r=0;
			while ((r<FSP_data->Nred) && (!Stop))
			    {
			    if (fabs((Aorig->p[SLRow[r]][ClassA[ClassANum]]/
				Aorig->p[SLRow[reference]][ClassA[ClassANum]])
							   -Constant[r])>SMALL)
				Stop=TRUE;
			    r++;
			    }
			}
		    ClassANum++;
		    }
		k++;
		}
	    if (!Stop)
		Stop2=TRUE;
	    j++;
	    }
	i++;
	}
    if (Stop2)
	{
	DoneRed=FALSE;
	/* Construct new g matrix */
	AElim = mat_malloc(1, ClassANum);
	n= mat_malloc(ClassANum, ClassANum-1);
	for (p=0; p<ClassANum;p++)
	    AElim->p[0][p]=Aorig->p[SLRow[reference]][ClassA[p]];
	mat_null(AElim, &nullity, n, &K2);
	for (p=0; p<(ClassANum-1);p++)
	    {
	    for (r=0; r<FSP_data->M; r++)
		Specialg ->p[*NumSpg][r]=0;
	    for (r=0; r<ClassANum;r++)
		Specialg ->p[*NumSpg][ClassA[r]]=n->p[r][p];
	    *NumSpg=*NumSpg+1;
	    }
	FSP_data->Nred--;
	RowElim[SLRow[reference]]=1;
	for (r=0;r<ClassANum;r++)
	    ColElim[ClassA[r]]=3;
	for (r=reference;r<FSP_data->Nred;r++)
	    SLRow[r]=SLRow[(r+1)];
	count=0;
	count2=0;
	for (position=0; position<FSP_data->Mred; position++)
	    if (SLCol[position]!=ClassA[count2])
		{
		SLCol[count]=SLCol[position];
		count++;
		}
	    else
		count2++;
	FSP_data->Mred=FSP_data->Mred-ClassANum;
	mat_free(AElim);
	mat_free(n);
	}
    }


     /* store the reduced A in the variable Ared, and the	*/
     /* modified b in bred					*/
j=-1;
for (i=0; i<N; i++)
  if (RowElim[i] == 0)
     {
     j++;
     bred->p[j][0]=btemp[i];
     m=-1;
     for (k=0; k<M; k++)
	if (ColElim[k] == 0)
	   {
	   m++;
	   Ared->p[j][m] = Aorig->p[i][k];
	   }
     }
free (SLRow);
free (SLCol);
free (Nonzero);
free (ClassA);  
free (Constant);

}



/* ________________________________________________________
  | name:    CheckB                                        |
  | description:  Checks that after reducing the A, not all|
  |               requested workspace motions are zero     |
  |               (trivial motion).                        |
  |                                                        |
  |                                                        |
  | inputs:  b, corresponding N                            |
  | outputs: If it is all zeros                            |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  k morganson 8/94      Created		           |
  |________________________________________________________| */

int CheckB	(MATRIX *b, int m)
{
int i;

i=0;
      /* stop checking b when a nonzero element is found */
while ((i<m) && (fabs(b->p[i][0]) < SMALL))
   i++;

if (i==m)
   return(0);
else
   return(m);
}



/* ________________________________________________________
  | name:    CheckRange                                    |
  | description:  Checks that the original b is in the     |
  |               range of A.                              |
  |                                                        |
  |                                                        |
  | inputs:  borig, Aorig, g, RowElim, Mred                |
  | outputs: If it is in range                             |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  k morganson 8/94      Created		           |
  |________________________________________________________| */

int CheckRange (MATRIX * b, MATRIX * Aorig, MATRIX * g,
	        int   *RowElim,    int    Mred )
{
  int i, j, k;
  float CheckValue, CheckTemp, CheckTemp2;
/*
  for (i = 0; i < (Aorig -> rows); i++)
  {
    if (RowElim[i] == 2)
    {
      for (j = 0; j < (g -> rows); j++)
      {
        CheckValue = 0.0;
	for (k = 0; k< (Aorig -> cols); k++)
	  CheckValue += Aorig -> p[i][k] * g -> p[j][k];
        CheckTemp  = fabs (b->p[i][0] - CheckValue);
        CheckTemp2 = .02;   
	if (CheckTemp > CheckTemp2)
	  return (0);
      }
    }
  }
*/  return (1);
}




/* ________________________________________________________
  | name:    Rebuild_gs                                    |
  | description:  Given the created g vectors for the      |
  |               reduced Jacobian, this procedure adds    |
  |               zeros in eliminated columns and tacks on |
  |               the g vectors created by SpecialCase1    |
  |                                                        |
  |                                                        |
  | inputs:  g, ColElim, Mred, Nred, M, NumSpg, Specialg   |
  | outputs: g                                             |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker    3/95      Created		           |
  |  g fries     3/95      Additional ability handling     |
  |                        SpecialCase1 (ie Specialg)      |
  |________________________________________________________| */


void Rebuild_gs (Solutions *FSP_data, int *ColElim, MATRIX *Specialg,int NumSpg)
{
  int    i,j,k;
  MATRIX *gtemp;

  gtemp = mat_cp2(FSP_data->g);

  k = 0;
  for (j = 0; j < FSP_data->M; j++)
  {
    if (ColElim[j] != 0)
    {
       for (i=0;i<FSP_data->g->rows;i++)
	 FSP_data->g->p[i][j] = ZERO; 
    }
    else
    {
       for (i=0;i<FSP_data->g->rows;i++)
	 FSP_data->g->p[i][j] = gtemp->p[i][k];
       k++;
    }
  }

  /*Addition of null vectors created by SpecialCase1*/
  for (i=(FSP_data->Mred-FSP_data->Nred+1);i<
			(FSP_data->Mred-FSP_data->Nred+1+NumSpg);i++)
    for(j=0;j<FSP_data->M; j++)
	if (ColElim[j] !=0)
	    FSP_data->g->p[i][j]=
			Specialg->p[(i-FSP_data->Mred+FSP_data->Nred-1)][j];
	    else
		FSP_data->g->p[i][j]=FSP_data->g->p[0][j];

  mat_free(gtemp);
}


