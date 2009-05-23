/* ________________________________________________________
  |                                                        |
  | Program Name:   Analytical.c                           |
  |________________________________________________________|
  |                                                        |
  | description: Aquires Betas for constraints and det-    |
  |     ermine the Analytical solution for each time step. |
  |                                                        |
  | Procedures:                                            |
  |    Build_Grammian2 :                                   |
  |    find_jl_beta    :                                   |
  |    find_obs_beta   :                                   |
  |    findt           :  A whole slew of these            |
  |________________________________________________________| */


#include <IKOR/general.h>       /* Header File to link all others */
#include <stdio.h>
#include <math.h>

void simp1(a,mm,ll,nll,iabf,kp,bmax)
float **a,*bmax;
int *kp,iabf,ll[],mm,nll;
{
  int k;
  float test;

  *kp=ll[1];
  *bmax=a[mm+1][*kp+1];
  for (k=2;k<=nll;k++) {
    if (iabf == 0)
      test=a[mm+1][ll[k]+1]-(*bmax);
    else
      test=fabs(a[mm+1][ll[k]+1])-fabs(*bmax);
    if (test > 0.0) {
      *bmax=a[mm+1][ll[k]+1];
      *kp=ll[k];
    }
  }
}


#define EPS 1.0e-6
void simp2(a,n,l2,nl2,ip,kp,q1)
float **a,*q1;
int *ip,kp,l2[],n,nl2;
{
  int k,ii,i;
  float qp,q0,q;

  *ip=0;
  for (i=1;i<=nl2;i++) {
    if (a[l2[i]+1][kp+1] < -EPS) {
      *q1 = -a[l2[i]+1][1]/a[l2[i]+1][kp+1];
      *ip=l2[i];
      for (i=i+1;i<=nl2;i++) {
        ii=l2[i];
        if (a[ii+1][kp+1] < -EPS) {
          q = -a[ii+1][1]/a[ii+1][kp+1];
          if (q < *q1) {
            *ip=ii;
            *q1=q;
          } else if (q == *q1) {
            for (k=1;k<=n;k++) {
              qp = -a[*ip+1][k+1]/a[*ip+1][kp+1];
              q0 = -a[ii+1][k+1]/a[ii+1][kp+1];
              if (q0 != qp) break;
            }
            if (q0 < qp) *ip=ii;
          }
        }
      }
    }
  }
}


void simp3(a,i1,k1,ip,kp)
float **a;
int i1,ip,k1,kp;
{
  int kk,ii;
  float piv;

  piv=1.0/a[ip+1][kp+1];
  for (ii=1;ii<=i1+1;ii++)
    if (ii-1 != ip) {
      a[ii][kp+1] *= piv;
      for (kk=1;kk<=k1+1;kk++)
        if (kk-1 != kp)
          a[ii][kk] -= a[ip+1][kk]*a[ii][kp+1];
    }
  for (kk=1;kk<=k1+1;kk++)
    if (kk-1 != kp) a[ip+1][kk] *= -piv;
  a[ip+1][kp+1]=piv;
}


#define FREEALL free_ivector(l3,1,m);free_ivector(l2,1,m);\
  		free_ivector(l1,1,n+1);

void simplx(a,m,n,m1,m2,m3,icase,izrov,iposv)
float **a;
int *icase,iposv[],izrov[],m,m1,m2,m3,n;
{
  void simp1(),simp2(),simp3();
  int i,ip,ir,is,k,kh,kp,m12,nl1,nl2;
  int *l1,*l2,*l3;
  float q1,bmax;

  if (m != (m1+m2+m3)) nrerror("Bad input constraint counts in simplx");
  l1=ivector(1,n+1);
  l2=ivector(1,m);
  l3=ivector(1,m);
  nl1=n;
  for (k=1;k<=n;k++) l1[k]=izrov[k]=k;
  nl2=m;
  for (i=1;i<=m;i++) {
    if (a[i+1][1] < 0.0) nrerror("Bad input tableau in simplx");
    l2[i]=i;
    iposv[i]=n+i;
  }
  for (i=1;i<=m2;i++) l3[i]=1;
  ir=0;
  if (m2+m3) {
    ir=1;
    for (k=1;k<=(n+1);k++) {
      q1=0.0;
      for (i=m1+1;i<=m;i++) q1 += a[i+1][k];
      a[m+2][k] = -q1;
    }
    do {
      simp1(a,m+1,l1,nl1,0,&kp,&bmax);
      if (bmax <= EPS && a[m+2][1] < -EPS) {
        *icase = -1;
        FREEALL return;
      } else if (bmax <= EPS && a[m+2][1] <= EPS) {
        m12=m1+m2+1;
        if (m12 <= m) {
          for (ip=m12;ip<=m;ip++) {
            if (iposv[ip] == (ip+n)) {
              simp1(a,ip,l1,
                nl1,1,&kp,&bmax);
              if (bmax > 0.0)
                goto one;
            }
          }
        }
        ir=0;
        --m12;
        if (m1+1 <= m12)
          for (i=m1+1;i<=m12;i++)
            if (l3[i-m1] == 1)
              for (k=1;k<=n+1;k++)
                a[i+1][k] = -a[i+1][k];
        break;
      }
      simp2(a,n,l2,nl2,&ip,kp,&q1);
      if (ip == 0) {
        *icase = -1;
        FREEALL return;
      }
  one:  simp3(a,m+1,n,ip,kp);
      if (iposv[ip] >= (n+m1+m2+1)) {
        for (k=1;k<=nl1;k++)
          if (l1[k] == kp) break;
        --nl1;
        for (is=k;is<=nl1;is++) l1[is]=l1[is+1];
        ++a[m+2][kp+1];
        for (i=1;i<=m+2;i++) a[i][kp+1] = -a[i][kp+1];
      } else {
        if (iposv[ip] >= (n+m1+1)) {
          kh=iposv[ip]-m1-n;
          if (l3[kh]) {
            l3[kh]=0;
            ++a[m+2][kp+1];
            for (i=1;i<=m+2;i++)
              a[i][kp+1] = -a[i][kp+1];
          }
        }
      }
      is=izrov[kp];
      izrov[kp]=iposv[ip];
      iposv[ip]=is;
    } while (ir);
  }
  for (;;) {
    simp1(a,0,l1,nl1,0,&kp,&bmax);
    if (bmax <= 0.0) {
      *icase=0;
      FREEALL return;
    }
    simp2(a,n,l2,nl2,&ip,kp,&q1);
    if (ip == 0) {
      *icase=1;
      FREEALL return;
    }
    simp3(a,m,n,ip,kp);
    is=izrov[kp];
    izrov[kp]=iposv[ip];
    iposv[ip]=is;
  }
}
#undef FREEALL


/* Driver for routine simplx */
#define NM1M2 (sN+sM1+sM2)

mysimplx( c, sM, sN, sM1, sM2, sM3, NP, MP )
float **c;
int sM, sN, sM1, sM2, sM3;
{
  int i,icase,j,*izrov,*iposv;
  float **a;
  static char *txt[9]=
    {" ","x1","x2","x3","x4","y1","y2","y3"};


    printf("\n\n N: %d, M: %d, M1: %d, M2: %d, M3: %d, NP: %d, MP: %d",
	        sN,    sM,    sM1,    sM2,    sM3,     NP,     MP);		
    for (i = 0; i<MP; i++)      { printf( "\nA[%d] = ", i );
      for (j=0; j<NP; j++)   printf( " %9.6f ", c[i][j] ); }
    printf("\n");


  izrov=ivector(1,sN);
  iposv=ivector(1,sM);
  a=convert_matrix(&c[0][0],1,MP,1,NP);
  simplx(a,sM,sN,sM1,sM2,sM3,&icase,izrov,iposv);
  if (icase == 1)
    printf("\nunbounded objective function\n");
  else if (icase == -1)
    printf("\nno solutions satisfy constraints given\n");
  else {
    printf("\n%11s"," ");
    for (i=1;i<=sN;i++)
      if (izrov[i] <= NM1M2) printf("%10s",txt[izrov[i]]);
    printf("\n");
    for (i=1;i<=M+1;i++) {
      if (i == 1 || iposv[i-1] <= NM1M2) {
        if (i > 1)
          printf("%s",txt[iposv[i-1]]);
        else
          printf("  ");
        printf("%10.2f",a[i][1]);
        for (j=2;j<=N+1;j++)
          if (izrov[j-1] <= NM1M2)
            printf("%10.2f",a[i][j]);
        printf("\n");
      }
    }
  }
  free_convert_matrix(a,1,MP,1,NP);
  free_ivector(iposv,1,sM);
  free_ivector(izrov,1,sN);

  exit (1);
  return 0;
}



/* ____________________________________________________
  | name:    findt_without_Betas_SIMPLX                |
  | description:  		                       |
  |                                                    |
  | inputs:  FSP_data's betall and g vectors           |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  | c hacker     02/95     Created		       |
  |____________________________________________________| */

MATRIX *findt_without_Betas_SIMPLX(FSP_data, B, H, datafp)
FILE      *datafp;        /* data file declaration       */       
MATRIX    *B,		  /* B, not Beta, in Pins paper  */
          *H;		  /* H in Pins paper             */
Solutions *FSP_data;      /* See structures.h            */
{
  int 	  t_ind,
	  col_ind,
	  sN, sM, i, j; 
  float  **coeff;
  MATRIX *dq;

  if (DEBUG) {
    fprintf(datafp, "\n Executing SIMPLX Optimization \n");
    fprintf(datafp, "\n SPAN = %d  Bcols = %d gcols = %d\n", 
			SPAN, B->cols, FSP_data->g->cols);
    fmat_pr(datafp, " Solution Vectors ", FSP_data->g);
  }

  dq   = mat_malloc( FSP_data->M, 1 ); 
  coeff= (float **) malloc ( 3 * sizeof( float * ) );

  for (t_ind=0;t_ind<3;t_ind++)
    coeff[t_ind]= (float *) malloc ( (SPAN + 1) * sizeof( float ) );
  coeff[0][0] = coeff[2][0] = 0.0; coeff[1][0] = 1.0;

  /* This is the ONE SHOT Method, It is default or borig != 0      */
  if ((STEP == ONE) && (!FSP_data->Null_Space)) 
  {
    for (t_ind = 1; t_ind < SPAN+1; t_ind++)
    {
      coeff[0][t_ind] = coeff[2][t_ind] = 0.0; coeff[1][t_ind] = -1.0;
      for (col_ind=0; col_ind<B->cols; col_ind++)  coeff[0][t_ind] +=
	B->p[col_ind][col_ind] * FSP_data->g->p[t_ind-1][col_ind];
    }

      if (DEBUG) ;

    sN = SPAN;
    sM = 1;
    printf("\n N: %d, M: %d, M1: %d, M2: %d, M3: %d, NP: %d, MP: %d",
	      sN,    sM,     0,      0,      1,     sN+1,   sM+2);		
    for (i = 0; i<3; i++)      { printf( "\nA[%d] = ", i );
      for (j=0; j<SPAN+1; j++)   printf( " %9.6f ", coeff[i][j] ); }
    printf("\n");

    /*               sM  sN  sM1 sM2 sM3  NP    MP     */
    mysimplx( coeff, sM, sN,  0,  0,  1, sN+1, sM+2 );

  }
  else if ((STEP == TWO) || (FSP_data->Null_Space)) {
    IKerror(31, OK, " SIMPLX has no Null_Space Motions ");
    for (col_ind=0; col_ind<B->cols; col_ind++)
       dq->p[col_ind][0] = ZERO;
  }
  else IKerror(29,FATAL,"findt_without_Betas_SIMPLX");


  return (dq);
}






/* ________________________________________________________
  | name:    Build_Grammian2                               |
  | description:  The Grammian is a multiplication of one  |
  |     each solution vector with each solution vector.    |
  |     Each component is a dot product of two of the      |
  |     vectors.  The B values are used as a weighting     |
  |     factor.  If B is the diagonal matrix for the       |
  |     link weights, as the value goes from 1 to zero the |
  |     joint that it relates to becomes more active. B    |
  |     is the same as the matrix B (not Beta) in F.G.     |
  |     Pin's paper.                                       |
  |                                                        |
  | inputs:  Solution Vectors (g), and alphas              |
  | outputs: Allocates memory (the 2 in Build_Grammian2,   |
  |     means that it allocates memory which must be freed |
  |     by the calling routine), builds, and return the    |
  |     structure to the calling routine.                  |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  c hacker     10/94    Created		           |
  |                1/95    Alphas implemented              |
  |________________________________________________________| */

MATRIX *Build_Grammian2 (FSP_data, B, datafp)
FILE      *datafp;
MATRIX    *B;
Solutions *FSP_data;
{
  int i,j,k;
  MATRIX *Grammian,
         *gtemp,
	 *Btemp,
	 *gBtrans;

  Grammian = mat_malloc(SPAN, SPAN);
  Btemp = mat_malloc(FSP_data->g->cols, FSP_data->g->cols);   
                                     /* if b is completely reduced this */
                                     /* is smaller than B               */
  if (B->rows == Btemp->rows)
    mat_cp(B, Btemp);
  else
    for (i=0; i<Btemp->rows; i++)
      for (j=0; j<Btemp->rows; j++)
        Btemp->p[i][j] = B->p[i][j];
  gtemp = mat_mul2(FSP_data->g, Robot->Weights);
  gBtrans = mat_mul2(gtemp, Btemp);


  for (i = 0; i < gBtrans->rows; i++)
    for (j = 0; j < gBtrans->rows; j++)
    {		
      Grammian-> p[i][j] = 0;
      for (k = 0; k < gBtrans->cols; k++)
      {
        Grammian -> p[i][j] += gBtrans -> p[i][k] *  gBtrans -> p[j][k];
      }
    }

  if (DEBUG) {
    fprintf(datafp,"\n  ______________________________  \n");
    fprintf(datafp,"\n         Building Grammian        \n");
    fprintf(datafp,  "  ______________________________  \n\n");
    fmat_pr(datafp,"g",FSP_data->g);
    fmat_pr(datafp,"Angle Weights", Robot->Weights);
    fmat_pr(datafp,"B Vector", B);
    fmat_pr(datafp,"g*Weights*B",gtemp);
    fmat_pr(datafp,"Grammian",Grammian);
  }

  mat_free(gtemp);
  mat_free(gBtrans);
  mat_free(Btemp);

  return (Grammian);
}


/* ________________________________________________________
  | name:    find_jl_beta                                  |
  | description:  finds beta values for the joint defined  |
  |    by chk.  Restricts said joint by confining it to    |
  |    remain at a maximum or minimum limit.  The variable |
  |    distance is the amount that the joint must move to  |
  |    get to the max or the min.                          |
  |                                                        |
  | inputs:  See Declarations below                        |
  | outputs: Alters two variables within FSP_data.         |
  |     the beta vector for the joint is added to betall,  |
  |     and cn, the number of beta vectors present, is     |
  |     incremented.                                       |
  |________________________________________________________|
  | Authors:     Date:     Modifications:                  |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's    |
  |  c hacker     10/94    Ported from 2 g's to FSP        |
  |                2/95    Ported to Solutions struct      |
  |________________________________________________________| */

void find_jl_beta(FSP_data, chk, distance, datafp)
FILE  *datafp;         /* data file declaration            */
int    chk;            /* link number and collision flag   */
double distance;       /* distance over low/up joint limit */
Solutions *FSP_data;   /* see structures.h                 */
{
  int i;

  if (chk > (FSP_data->g->cols -1));
  else
    for( i=0; i < FSP_data->g->rows; i++ )
      FSP_data->betall->p[i][FSP_data->cn] = 
                             FSP_data->g->p[i][chk]/(distance);

  FSP_data->cn++;
}                 /* end find_jl_beta */


/* ________________________________________________________
  | name:    find_obs_beta                                 |
  | description:  finds beta values for a link[chk] at a   |
  |   distance, newl from its beginning, that has entered  |
  |   an obstacle's buffer zone.  A Jacobian is created    |
  |   for up to that portion of the link, and the beta     |
  |   vector is calculated such that the point pushed back |
  |   to the edge of the zone.                             |
  |                                                        |
  | inputs:  See Declarations below                        |
  | outputs: Alters two variables within FSP_data. The     |
  |   beta vector for the joint is added to betall, and    |
  |   cn, the number of beta vectors present, is           |
  |   incremented.                                         |
  |________________________________________________________|
  |  Authors:      Date:     Modifications:                |
  |  F Tulloch     4/94      Algorithm Creation for 2 g's  |
  |  c hacker     10/94      Ported from 2 g's to FSP      |
  |                2/95      Ported to Solutions struct    |
  |________________________________________________________| */

void find_obs_beta(FSP_data, chk, newl, delta, normal, datafp)
  FILE   *datafp;      /* data file declaration                */
  int     chk;         /* link that has moved into buffer      */
  double *newl,        /* distance on chk link of most contact */
          delta,       /* distance needed to move along normal */
          normal[3];   /* normal guide vector                  */
  Solutions *FSP_data; /* See structures.h                     */
{
  int    i, j, p, k;
  MATRIX *alpha,        /* Holds alphas        */
         *Jacob;        /* Holds Jacobian      */

  alpha = mat_malloc( 3, SPAN );
  Jacob = mat_malloc( N, M );

  /*****************************************************************
   *                                                               *
   *        get new jacob using using new links based on inters    *
   *****************************************************************/

  for (i=0; i<Robot->NL; i++) LL[i] = Robot->LINKS[i];

  LL[chk] = *newl;
  for (i=chk+1; i<Robot->NL; i++ )  LL[i] = 0.0;

  GET_JACOBIAN_ALTERED(Jacob, FSP_data->Qarray);      
 
  /***********************************************************
   *        Calculate Alphas and then betas necessary        *
   ***********************************************************/

  for (i = 0;i<3;i++) for (p = 0;p<SPAN;p++) alpha -> p[i][p] = 0.0;

  /* Calculate Alpha: J * g = (3 x VSpan) */
  for (i = 0; i < 3; i++)  
    for (j = 0; j < SPAN; j++)
      for (k = 0; k < M; k++)
        alpha->p[i][j] += Jacob->p[i][k] * FSP_data->g->p[j][k];

  if (DEBUG) {
    fmat_pr(datafp,"*** Jacobian @ Xj ***", Jacob);
    fmat_pr(datafp,"*** Alpha ***", alpha);
  }

  /* Calculate Beta: Alpha * normal = (VSpan x 1) */
  for (i = 0; i < SPAN; i++)
  {
    FSP_data->betall->p[i][FSP_data->cn] = 0;
    for (j = 0; j < 3; j++)
      FSP_data->betall->p[i][FSP_data->cn] += 
                 alpha->p[j][i] * normal[j];
    FSP_data->betall->p[i][FSP_data->cn] /= fabs(delta);
    if (DEBUG) fprintf(datafp,"b[%d]: %f  ", i, 
				FSP_data->betall->p[i][FSP_data->cn]);
  }
  FSP_data->cn++;

  mat_free(alpha);
  mat_free(Jacob);
}                 /* end find_obs_beta */


/* ____________________________________________________
  | name:    findt_without_Betas_BANGBANG              |
  | description:  		                       |
  |                                                    |
  | inputs:  FSP_data's betall and g vectors           |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  | c hacker     02/95     Created		       |
  |____________________________________________________| */

MATRIX *findt_without_Betas_BANGBANG(FSP_data, B, H, old, datafp)
FILE      *datafp;        /* data file declaration       */       
MATRIX    *B,		  /* B, not Beta, in Pins paper  */
	  *H,		  /* H in Pins paper             */
          *old;		  /* previous dq		 */
Solutions *FSP_data;      /* See structures.h            */
{
  int 	t_ind,
	col_ind,
	least_ind = -1,
	MISMATCH  = FALSE; 
  float least_flow,
	total_flow;
  MATRIX *dq, *dq1;

  if (DEBUG) {
    fprintf(datafp, "\n Executing BIGBANG Optimization \n");
    fprintf(datafp, "\n SPAN = %d  Bcols = %d gcols = %d\n", 
			SPAN, B->cols, FSP_data->g->cols);
    fmat_pr(datafp, " Solution Vectors ", FSP_data->g);
  }

  dq   = mat_malloc( FSP_data->M, 1 ); 

  /* This is the ONE SHOT Method, It is default or borig != 0      */
  if ((STEP == ONE) && (!FSP_data->Null_Space)) 
  {
    for (t_ind = 0; t_ind < SPAN; t_ind++)
    {
      total_flow=0.0;
      for (col_ind=0; col_ind<B->cols; col_ind++)
	total_flow += (old->p[col_ind][0]>0)?1.0:-1.0 *
		( B->p[col_ind][col_ind] * FSP_data->g->p[t_ind][col_ind] );

      if (DEBUG);{ fprintf(stderr, " %9.6f ", total_flow ); }
 
      if ((total_flow < least_flow) || (least_ind<0))
      { least_ind = t_ind; least_flow = total_flow; }
    }

    for (col_ind=0; col_ind<B->cols; col_ind++) {
      dq->p[col_ind][0] = FSP_data->g->p[least_ind][col_ind];
      if ((dq->p[col_ind][0]>=0)&&(old->p[col_ind][0]>=0));
      else if ((dq->p[col_ind][0]<=0)&&(old->p[col_ind][0]<=0)); 
      else {
	MISMATCH = TRUE;
	fprintf(stderr, "MISMATCH: dq[%d] = %9.6f, old[%d] = %9.6f \n",
		col_ind, dq->p[col_ind][0], col_ind, old->p[col_ind][0]);
	break;
      }
    }

    if (MISMATCH) {
	dq1 = findt_without_Betas_BANGBANG(FSP_data, B, H, dq, datafp);
	for (col_ind=0; col_ind<B->cols; col_ind++)
	  dq->p[col_ind][0] = dq1->p[col_ind][0];
	mat_free(dq1);
	MISMATCH = FALSE;
    }
    else fprintf(stderr, "OK");


    if (DEBUG) {
      fprintf(datafp, "\nLeast Flow: %f, Least Index: %d \n\n\n", 
		least_flow, least_ind );
    }
  }
  else if ((STEP == TWO) || (FSP_data->Null_Space)) {
    IKerror(31, OK, " BANGBANG has no Null_Space Motions ");
    for (col_ind=0; col_ind<B->cols; col_ind++)
      dq->p[col_ind][0] = ZERO;
  }
  else IKerror(29,FATAL,"findt_without_Betas_BANGBANG");

fprintf(stderr,"\n");

  return (dq);
}


/* ____________________________________________________
  | name:    findt_with_Betas_Holonomic                |
  | description:  find the parameterization for the    |
  |    solution vectors found using FSP blocking.  The |
  |    linear cmbination produced will either be a     |
  |    full space or null space motion based on the    |
  |    values of nu, and mu.  A macro is used to       |
  |    determine which method to use.  TWOSTEP means   |
  |    that a motion is first produced, and then       |
  |    obstacles and joint limits are avoided in the   |
  |    null space.  Otherwise, everything is done at   |
  |    once in the full space.                         |
  |                                                    |
  | inputs:  FSP_data's betall and g vectors           |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  | F Tulloch     4/94     Algorithm Creation for 2 g's|
  | c hacker     10/94     Ported from 2 g's to FSP    |
  |               2/95     Ported to Solutions struct  |
  |               3/95     Implemented ONE & TWO step  |
  |____________________________________________________| */

MATRIX *findt_with_Betas_Holonomic(FSP_data, B, H, datafp)
FILE      *datafp;        /* data file declaration       */       
MATRIX    *B,		  /* B, not Beta, in Pins paper  */
          *H;		  /* H in Pins paper             */
Solutions *FSP_data;      /* See structures.h            */
{
  int     i, j, k, a, b;
  double  sum1 = 0, sum2 = 0,
	  afinal;
  MATRIX *bfinal,
         *cfinal,
         *dfinal,
         *A,  *Ainv,
         *nu, *mu,
         *e, *eT, 
         *t, *temp, *temp2, *temp3,
         *G, *Ginv, *iden,
	 *dq;
  float  ctemp;

  dq   = mat_malloc(M   , 1); 
  e    = mat_malloc(SPAN, 1);
  eT   = mat_malloc( 1,   SPAN);

  if (DEBUG) fprintf(datafp, "\n\n# of constraints %d \n", FSP_data->cn);

  /* initialize the vectors of ones */
  for (i = 0; i < SPAN; i++)   
  { e -> p[i][0] = eT -> p[0][i] = 1.0e0;}

  /* See Methods.c, malloc's G, multiplies g's  and returns */
  G = Build_Grammian2(FSP_data, B, datafp);  

  Ginv = mat_cp2(G);
  mat_LU_inv(Ginv);
  iden = mat_mul2(Ginv, G); 

  if (DEBUG) {
    fmat_pr(datafp,"*** Identity Matrix Check (G * Ginv) ***", iden);
    fmat_pr(datafp,"*** betas ***", FSP_data->betall);
  }



  /***************************************************************
   *                                                             *
   *    a,b,c,d, A, mu, and nu are based on F.G. Pin's TM        *
   *                                                             *
   ***************************************************************/

  /********* Calculate a's *********/
  afinal = 0;
  for (i = 0; i < SPAN; i++)
    for (j = 0; j < SPAN; j++)
      afinal += Ginv -> p[i][j];

  /********* Calculate b's *********/

  temp   = mat_malloc (SPAN,1);
  bfinal = mat_malloc (1, FSP_data->cn);
  for (k = 0; k < FSP_data->cn; k++)
  {
    for (i = 0; i < SPAN; i++)
    {
      temp->p[i][0] = 0.0;
      for (j = 0; j < SPAN; j++)
        temp->p[i][0] += Ginv->p[i][j] * FSP_data->betall->p[j][k];
    }
    bfinal->p[0][k] = 0.0;
    for (i = 0; i < SPAN; i++)
      bfinal->p[0][k] += temp->p[i][0];
  }                 /* end k for */
  mat_free(temp);

  /********* Calculate c's ********/

  temp = mat_malloc (SPAN, 1);
  cfinal = mat_malloc (FSP_data->cn , 1);
  for (a = 0; a < FSP_data->cn; a++)
  {
    for (i = 0; i < SPAN; i++)
    {
      temp->p[i][0] = 0.0;
      for (j = 0; j < SPAN; j++)
    temp->p[i][0] += Ginv->p[i][j];
    }
    cfinal ->p[a][0] = 0.0;
    for (j = 0; j < SPAN; j++)
      cfinal->p[a][0] += FSP_data->betall->p[j][a] * temp->p[j][0];
   }                 /* end a for */
  mat_free(temp);

  /******** Calculate d's ********/
  temp   = mat_malloc(SPAN, 1);
  dfinal = mat_malloc(FSP_data->cn,FSP_data->cn);
  for (k = 0; k < FSP_data->cn; k++)
    for (b = 0; b < FSP_data->cn; b++)
    {
      dfinal->p[k][b] = 0.0;
      for (i = 0; i < SPAN; i++)
      {
        temp->p[i][0] = 0.0;
        for (j = 0; j < SPAN; j++)
          temp->p[i][0] += Ginv->p[i][j] * FSP_data->betall->p[j][k];
      }
      for (j = 0; j < SPAN; j++)
      dfinal->p[k][b] += FSP_data->betall->p[j][b] * temp->p[j][0];
    }                 /* end k for */
  mat_free(temp);

  /******** Calculate A  ********/

  temp = mat_malloc(FSP_data->cn,FSP_data->cn);
  for(a=0;a<FSP_data->cn;a++)
    for(b=0;b<FSP_data->cn;b++)
      temp ->p[a][b]=dfinal->p[a][b]*afinal;  

  temp2= mat_malloc(FSP_data->cn,FSP_data->cn);
    for (i = 0; i < FSP_data->cn; i++)
      for (j = 0; j < FSP_data->cn; j++)
        temp2->p[i][j]=cfinal->p[i][0]*bfinal->p[0][j];
      
  A    = mat_malloc(FSP_data->cn, FSP_data->cn);
  Ainv = mat_malloc(FSP_data->cn, FSP_data->cn);

  for (i = 0; i < FSP_data->cn; i++)
    for (j = 0; j < FSP_data->cn; j++)
      A ->p[i][j]=temp2->p[i][j]-temp->p[i][j];

  mat_free(temp);
  mat_free(temp2);
  mat_cp(A, Ainv);

  if (DEBUG) {    
     fprintf(datafp, "   a= %9.9f\n", afinal);
     fmat_prf(datafp, "b= ", bfinal);
     fmat_prf(datafp, "c= ", cfinal);
     fmat_prf(datafp, "d= ", dfinal);
     fmat_prf(datafp, "A = ", A);
  }
  if ((A->p[0][0]==0.0)&&(A->rows=1)&&(A->cols=1))
	IKerror(30,FATAL,"");
  else mat_LU_inv(Ainv);


  /**************************************************************
   *                                                            *
   *    Find mu and nu Lagrange multipliers for Null or Full    *
   *                                                            *
   **************************************************************/
  if ((STEP == ONE) /* This is the ONE SHOT Method, It is default      */
	&& (!FSP_data->Null_Space)) /* This is for the case borig == 0 */
  {
     /************<< find nu >>*********** FULLSPACE ********/
    temp = mat_malloc (FSP_data->cn, 1);
    for(i=0;i<FSP_data->cn;i++)
      temp->p[i][0]=afinal - cfinal->p[i][0];
    nu=mat_mul2(Ainv,temp);
    mat_free(temp);

    /*************<< find mu >>*********** FULLSPACE ********/
    mu = mat_mul2(bfinal, nu);
    mu->p[0][0] = -1 * (mu->p[0][0] + 1 )/ afinal;
  }
  else if ((STEP == TWO) || (FSP_data->Null_Space))
  {
    /************<< find nu >>************ NULLSPACE ********/
    temp = mat_malloc (FSP_data->cn, 1);
    for(i=0;i<FSP_data->cn;i++)
      temp->p[i][0]=afinal;
    nu=mat_mul2(Ainv,temp);
    mat_free(temp);

    /************<< find mu >>************ NULLSPACE ********/
    mu = mat_mul2(bfinal, nu);
    mu->p[0][0] /= -afinal;  
  }
  else IKerror(29,FATAL,"findt_with_Betas_Holonomic");

  /**************************************************************
   *                                                            *
   *    Find the parametric Analytical Solutions 't'            *
   *                                                            *
   **************************************************************/
  temp2 = mat_malloc(SPAN, 1);    /* temp2= Ge     */
  for(i=0;i<SPAN;i++)
  {
    temp2->p[i][0] = 0.0;
    for(j=0;j<SPAN;j++)
      temp2->p[i][0]+=Ginv->p[i][j];
  }
       
  temp3 = mat_malloc(SPAN, 1);    /* temp3= -muGe   */
  for(i=0;i<SPAN;i++)
    temp3->p[i][0]= -(mu->p[0][0])*temp2->p[i][0];
  mat_free(temp2);

  temp = mat_malloc (SPAN, 1);   /* temp = bNu     */
  for(i=0;i<SPAN;i++)
  {
    temp->p[i][0] = 0.0;
    for(a=0;a<FSP_data->cn;a++)
      temp->p[i][0]+=FSP_data->betall->p[i][a]*nu->p[a][0];
  }

  temp2= mat_malloc (SPAN, 1);   /* temp2= GbNu    */
  for(i=0;i<SPAN;i++)
  {
    temp2->p[i][0] = 0.0;        
    for(j=0;j<SPAN;j++)
      temp2->p[i][0]+=Ginv->p[i][j]*temp->p[j][0];
  }

  t = mat_malloc (SPAN, 1);
  for(i=0;i<SPAN;i++)
    t->p[i][0] = temp3->p[i][0] - temp2->p[i][0];        


  mat_free(temp );
  mat_free(temp2);
  mat_free(temp3);

  sum1 = 0;
  for (i = 0; i < SPAN; i++)
     sum1 += t -> p[i][0];
  if (!FSP_data->Null_Space) {
       if ((sum1< .99)||(sum1>1.01)) IKerror (21, FATAL, 
		"findt_with_Betas_Holonomic");
  }
  else if ((sum1<-.01)||(sum1> .01)) IKerror (21, FATAL,
		 "findt_with_Betas_Holonomic");


  if (DEBUG) {    
     fmat_prf(datafp, "Nu =", nu); 
     fprintf(datafp, "   Mu = %9.9f \n", mu->p[0][0]);
     fmat_prf(datafp, "Ginv * -mu", temp3);   
     fmat_prf(datafp, "Ginv * B * nu", temp2);
     fmat_prf(datafp, "*** t's for Collision ***", t);

     sum1 = 0;
     for (i = 0; i < SPAN; i++)
        sum1 += t -> p[i][0];
     fprintf(datafp, "\nsum ti:  %7.4f\n", sum1);

     for (j=0; j<FSP_data->cn; j++)
     {
        sum1 = 0;
        for (i = 0; i < SPAN; i++)
          sum1 += FSP_data->betall -> p[i][j] * t -> p[i][0];
        fprintf(datafp, "\nsum betai*ti:  %7.4f \n", sum1);
     }
  }

  for (i = 0; i < FSP_data->M; i++)  
  {
    dq->p[i][0]=0.0;
    for (j = 0; j < SPAN; j++)
      dq->p[i][0] += (t -> p[j][0]) * (FSP_data->g->p[j][i]);
  }


  
  mat_free(t);
  mat_free(nu);
  mat_free(mu);

  mat_free(iden);
  mat_free(Ginv);
  mat_free(bfinal);
  mat_free(cfinal);
  mat_free(dfinal);
  mat_free(A);
  mat_free(Ainv);
  mat_free(G);
  mat_free(e);
  mat_free(eT);
 return (dq);
}                 /* end findt */



/* ____________________________________________________
  | name:    findt_without_Betas_Holonomic             |
  | description:  find the parameterization for the    |
  |    solution vectors found using FSP blocking.  The |
  |    linear cmbination produced will be a full space |
  |    leastnorm motion.  This is called when there are|
  |    no constraints on the system.                   |
  |                                                    |
  | inputs:  g vectors                                 |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  |  F Tulloch     4/94    Algorithm Creation for 2 g's|
  |  c hacker     10/94    Ported from 2 g's to FSP    |
  |                2/95    Ported to Solutions struct  |
  |                3/95    Implemented ONE & TWO step  |
  |____________________________________________________| */

MATRIX *findt_without_Betas_Holonomic(FSP_data, B, H, datafp)
FILE      *datafp;        /* data file declaration       */
MATRIX    *B,		  /* B, not Beta, in Pins paper  */
          *H;		  /* H in Pins paper             */
Solutions *FSP_data;      /* See structures.h            */
{
  int    i, j, k;
  float  sum=0.0;
  double a, b;
  MATRIX *dq,
	 *G,*Gtemp,      /* Grammian formed of solution vectors	 */
         *t,	         /* weighting factors, one for each vect */
         *x,*y,*z,*iden, /* dummy variable for calculations	 */
         *e,	         /* vertical vector of ones		 */
         *eT;            /* horizontal vector of ones		 */


  dq    = mat_malloc (M   ,    1); 
  e     = mat_malloc (SPAN,    1);
  eT    = mat_malloc (   1, SPAN);
  z     = mat_malloc (H->rows, H->cols);

  /* initialize the vectors of ones */
  for (i = 0; i < (SPAN); i++)   
  { e  -> p[i][0] = eT -> p[0][i] = 1.0e0; }

  Gtemp = Build_Grammian2 (FSP_data, B, datafp);

  G = mat_cp2(Gtemp);
  mat_LU_inv(G);

  if (DEBUG) {       
    fprintf(datafp,"\n  ______________________________  \n");
    fprintf(datafp,"\n         LEAST NORM MOTION        \n");
    fprintf(datafp,  "  ______________________________  \n\n");

    iden = mat_mul2(G,Gtemp);  /* This code verifies Grammian Inversion */
    fmat_pr(datafp,"*** G  ***",Gtemp);
    fmat_pr(datafp,"*** Ginv ***", G);
    fmat_pr(datafp,"*** iden ***", iden);
    mat_free(iden);
  }                /* if DEBUGGING print out  (see file: info) */

  x = mat_mul2(eT, G);
  y = mat_mul2(x, e);

  a = y->p[0][0]; mat_free(y);

  y = mat_mul2(x, H);

  if (!FSP_data->Null_Space)
    b = -(1 + y->p[0][0])/a;
  else
    b = -y->p[0][0]/a;

  mat_sca(e, b);

  mat_add(H, e, z);
  mat_sca(G, -1.0);

  t = mat_mul2(G, z);  /* t's now complete */

  for (i = 0; i < M; i++)
  {
    dq->p[i][0] = 0.0e0;
    for (j = 0; j < SPAN; j++)
      dq->p[i][0] += (t -> p[j][0]) * (FSP_data->g -> p[j][i]);
  }  

  for (i = 0, sum = 0; i < SPAN; i++) sum += t -> p[i][0];
  if (!FSP_data->Null_Space) {
    if ((sum< .99)||(sum>1.01)) 
	IKerror (21, FATAL, "findt_without_Betas_Holonomic");
  }
  else if ((sum<-.01)||(sum> .01)) IKerror (21, FATAL, 
		"findt_without_Betas_Holonomic");


  if (DEBUG) {              /** print out t's, sums and New dq's **/
    fmat_pr(datafp, "*** t's ***", t);
    fprintf(datafp, "\nsum (no betas):  %7.4f \n", sum);
    for (i = 0, sum = 0; i < M; i++)
      sum += dq->p[i][0];
    fprintf(datafp, "\nsum t*g:  %7.4f\n", sum);
  }             /* if DEBUGGING, include code  (see file info) */

  mat_free(Gtemp);
  mat_free(G);
  mat_free(eT);
  mat_free(e);
  mat_free(t);
  mat_free(x);
  mat_free(y);

  return (dq);
}


/* ____________________________________________________
  | name:    findt_with_Betas_Nonholonomic             |
  | description:  find the parameterization for the    |
  |    solution vectors found using FSP blocking.  The |
  |    linear cmbination produced will either be a     |
  |    full space or null space motion based on the    |
  |    values of nu, and mu.  A macro is used to       |
  |    determine which method to use.  TWOSTEP means   |
  |    that a motion is first produced, and then       |
  |    obstacles and joint limits are avoided in the   |
  |    null space.  Otherwise, everything is done at   |
  |    once in the full space.                         |
  |                                                    |
  | inputs:  FSP_data's betall and g vectors           |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  | K Gower      10/95     Created                     |
  |                        e-mail: akgower@aol.com     |  
  |____________________________________________________| */

#include"general.h"

MATRIX *findt_with_Betas_Nonholonomic(FSP_data, B, H, datafp)
FILE      *datafp;        /* data file declaration       */
MATRIX    *B,             /* B, not Beta, in Pins paper  */
          *H;             /* H in Pins paper             */
Solutions *FSP_data;      /* See structures.h            */

{

MATRIX  *temp,
	*temp1,
	*temp2,
	*temp3,
	*temp4,
	*temp5,
	*temp6,
	*e, *eT,
	*G, *Ginv,
        *cfinal,
        *sfinal,
	*tfinal,
	*mfinal,
	*rfinal,
	*zfinal,
	*wfinal,
        *delta,
	*lambda,  *lambda_inv,
	*Afinal,
	*alpha,   *alphaT,
	*xfinal,
	*nu,
        *iden,
        *dq,
	*betas;
float    ctemp,
	 ctemp1,
	 ctemp2,
	 ctemp3,
         bfinal,
         dfinal,
	 lfinal,
	 kfinal,
	 pfinal,
	 eta,
	 mu;
double   sum1 = 0, sum2 = 0; 

int      i,j,k,
	 b;

  if (DEBUG)  fprintf(datafp, "\n\n# of constraints %d \n", FSP_data->cn);

  /* initialize the vectors of ones */
  e = mat_malloc(SPAN,1);
  eT= mat_malloc(1,SPAN);
  for (i = 0; i < SPAN; i++)   
  { e -> p[i][0] = eT -> p[0][i] = 1.0e0;}

  /* See Methods.c, malloc's G, multiplies g's  and returns */
  G = Build_Grammian2(FSP_data, B, datafp);  

  Ginv = mat_cp2(G);
  mat_LU_inv(Ginv);   

  /* Puts betall into column form to be used by Findt_with_Betas_
     Nonholonomic   */
  betas = mat_malloc(M, FSP_data->cn);  
  for (i = 0; i < M; i++)
      for (j = 0; j < FSP_data->cn; j++)
        betas->p[i][j] = FSP_data->betall->p[i][j];
  if (DEBUG)
    {
    fmat_pr(datafp,"betas (betall)", betas);
    iden = mat_mul2(Ginv, G); 
    fmat_pr(datafp,"*** Identity Matrix Check (G * Ginv) ***", iden);
    fmat_pr(datafp,"*** betas ***", FSP_data->betall);
    mat_free(iden);
    }
  /***************************************************************
   *                                                             *
   *   b, c, d, k, l, m, p, r, s, w, x, z, delta, lambda, eta,   *
   *      mu, and nu are based on Katie's paper.                 *
   *                                                             *
   ***************************************************************/

 /******** Generate A-vector ********/
  Afinal = mat_malloc(M,1);
  for (i = 0; i < M; i++) Afinal->p[i][0] = 0.0;
  Afinal->p[M - 3][0] = -sin(FSP_data->Qarray->p[M-1][0] + (PI/2));
  Afinal->p[M - 2][0] =  cos(FSP_data->Qarray->p[M-1][0] + (PI/2));
  Afinal->p[M - 1][0] = -(Robot->PLAT.Length)/2;
  /********* Generate alpha. *********/
  alpha = mat_malloc(SPAN,1);
    for (i = 0; i < SPAN; i++)
      alpha->p[i][0] = 0.0;
  for (i = 0; i < SPAN; i++)
    for (j = M - 3; j < M; j++)
      alpha->p[i][0] += Afinal->p[j][0] * FSP_data->g->p[i][j];
  alphaT=mat_malloc(1,SPAN);
  for (i = 0; i < SPAN; i++)
    alphaT->p[0][i] = alpha->p[i][0];

/********** Calculate b's ***********/                         
temp = mat_malloc(1,SPAN);
bfinal = 0.0;
for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += alphaT->p[0][j] * Ginv->p[j][i];
    }
for (i = 0; i < SPAN; i++)
    bfinal += temp->p[0][i];
mat_free(temp);                                                   

/********** Calculate d's **********  */                     
temp = mat_malloc(1,SPAN); 
dfinal = 0.0;
for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += alphaT->p[0][j] * Ginv->p[j][i];
    }
for (i = 0; i < SPAN; i++)
    dfinal += temp->p[0][i] * alpha->p[i][0];
mat_free(temp);
                                
/********** Calculate k's ********** */                    
kfinal = 0.0;
for (i = 0; i < SPAN; i++)
    for (j = 0; j < SPAN; j++)
	kfinal += Ginv->p[i][j];

/********** Calculate l's ********** */                     
temp = mat_malloc(1,SPAN);
lfinal = 0.0;
for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += Ginv->p[j][i];
    }
for (i = 0; i < SPAN; i++)
    lfinal += temp->p[0][i] * alpha->p[i][0];
mat_free(temp);

/********** Calculate p's **********  */ 
                     
pfinal = (bfinal * lfinal) - (dfinal * kfinal);

/********** Calculate c's **********  */                 
cfinal = mat_malloc(FSP_data->cn, 1);
temp  = mat_malloc(1, SPAN);
mat_mul(alphaT, Ginv, temp);
for (j = 0; j < FSP_data->cn; j++)
    {
    cfinal->p[j][0] = 0.0;
    for (i = 0; i < SPAN; i++)
	cfinal->p[j][0] += temp->p[0][i] * betas->p[i][j];
    }
mat_free(temp);

/********** Calculate s's **********  */                  
sfinal = mat_malloc(FSP_data->cn, 1);
temp   = mat_malloc(SPAN, 1);
mat_mul(Ginv, alpha, temp);
mat_tra(betas);
for (j = 0; j < FSP_data->cn; j++)
    {
    sfinal->p[j][0] = 0.0;
    for (i = 0; i < SPAN; i++)
	sfinal->p[j][0] += betas->p[j][i] * temp->p[i][0]; 
    }
mat_tra(betas);
mat_free(temp);

/********** Calculate m's **********  */	                      		
mfinal = mat_malloc(FSP_data->cn,1);
temp   = mat_malloc(1,SPAN);
mat_mul(eT, Ginv, temp);
for (i = 0; i < FSP_data->cn; i++)
    {
    mfinal->p[i][0] = 0;
    for (j = 0; j < SPAN; j++)
	mfinal->p[i][0] += temp->p[0][j] * betas->p[j][i];
    }
mat_free(temp);

/********** Calculate r's **********  */	                    		
rfinal = mat_malloc(FSP_data->cn, 1);
temp   = mat_malloc(SPAN,1);
mat_mul(Ginv, e, temp);
mat_tra(betas);
for (i = 0; i < FSP_data->cn; i++)
    {
    rfinal->p[i][0] = 0;
    for (j = 0; j < SPAN; j++)
	rfinal->p[i][0] += betas->p[i][j] * temp->p[j][0];
    }
mat_tra(betas);
mat_free(temp);

/********** Calculate z's **********  */        
zfinal = mat_malloc(FSP_data->cn, 1);
mat_sca(sfinal, bfinal);
mat_sca(rfinal, dfinal);
mat_sub(sfinal, rfinal, zfinal);
ctemp1 = 1 / bfinal;
ctemp2 = 1 / dfinal;
mat_sca(sfinal, ctemp1);
mat_sca(rfinal, ctemp2);
 
/********** Calculate w's **********  */                  
wfinal = mat_malloc(FSP_data->cn, 1);
mat_sca(cfinal, kfinal);
mat_sca(mfinal, bfinal);
mat_sub(cfinal, mfinal, wfinal);
ctemp1 = 1 / kfinal;
ctemp2 = 1 / bfinal;
mat_sca(cfinal, ctemp1);
mat_sca(mfinal, ctemp2);
/******** Calculate delta's ********   */
/* Something is fishy here with the limits for the mat_malloc and the 
   for loops */

  temp   = mat_malloc(SPAN, 1);
  delta = mat_malloc(FSP_data->cn, FSP_data->cn);
  for (k = 0; k < FSP_data->cn; k++)
    for (b = 0; b < FSP_data->cn; b++)
    {
      delta->p[k][b] = 0.0;
      for (i = 0; i < SPAN; i++)
      {
        temp->p[i][0] = 0.0;
        for (j = 0; j < SPAN; j++)
          temp->p[i][0] += Ginv->p[i][j] * betas->p[j][k];
      }      
      for (j = 0; j < SPAN; j++)
      delta->p[k][b] += betas->p[j][b] * temp->p[j][0];
    }                 /* end k for */
  mat_free(temp);

/********** Calculate lambda's **********/
lambda = mat_malloc(FSP_data->cn,FSP_data->cn);
for (j = 0; j < FSP_data->cn; j++)
    for (i = 0; i < FSP_data->cn; i++)
	{
	ctemp1 = bfinal * delta->p[j][i];
	ctemp2 = rfinal->p[j][0] * cfinal->p[i][0];
	lambda->p[j][i] = pfinal * (ctemp1 - ctemp2);
	lambda->p[j][i] += zfinal->p[j][0] * wfinal->p[i][0];
	} 

 /**************************************************************
   *                                                            *
   *Find x, eta, mu and nu Lagrange multipliers for Null or Full*
   *                                                            *
   **************************************************************/
  /* This is the ONE SHOT Method, It is default */
   
  if ((STEP == ONE) && (!FSP_data->Null_Space))
  {  
   /************<< find x's >>********** FULLSPACE ********/
    xfinal = mat_malloc(FSP_data->cn,1);
    mat_sca(zfinal, bfinal);
    for (i = 0; i < FSP_data->cn; i++)
        xfinal->p[i][0] = zfinal->p[i][0] - (bfinal * pfinal);
    ctemp1 = 1/bfinal;
    mat_sca(zfinal,ctemp1);

   /************<< find nu >>*********** FULLSPACE ********/
    nu = mat_malloc(FSP_data->cn,1);
    lambda_inv = mat_malloc(FSP_data->cn,FSP_data->cn);
    for (i = 0; i < FSP_data->cn;i++)
        for (j = 0; j < FSP_data->cn; j++)
            lambda_inv->p[i][j] = lambda->p[i][j];
    mat_LU_inv(lambda_inv);
    mat_mul(lambda_inv, xfinal, nu);

   /*************<< find eta >>********** FULLSPACE ********/
    temp1 = mat_malloc(1,1);
    temp2 = mat_malloc(1,1);
    mat_tra(nu);
    mat_mul(nu,cfinal,temp1);
    mat_sca(temp1,kfinal);
    mat_mul(nu,mfinal,temp2);
    mat_sca(temp2,bfinal);
    eta = -(bfinal - temp1->p[0][0] + temp2->p[0][0])/pfinal;
    mat_tra(nu);
    mat_free(temp1);
    mat_free(temp2);
          
   /*************<< find mu >>*********** FULLSPACE ********/
    temp1 = mat_malloc(1,1);
    mat_tra(nu);
    mat_mul(nu,cfinal,temp1);
    mu = -(temp1->p[0][0] + (eta * dfinal))/bfinal;
    mat_tra(nu);
    mat_free(temp1);   
  }
  else if ((STEP == TWO) || (FSP_data->Null_Space))
  {
   /**********<< find x's >>************** NULLSPACE ********/ 
    xfinal = mat_malloc(FSP_data->cn,1);
    for (i = 0; i < FSP_data->cn; i++)
        xfinal->p[i][0] = -bfinal * pfinal;

   /************<< find nu >>************ NULLSPACE ********/ 
    nu = mat_malloc(FSP_data->cn, 1);
    lambda_inv = mat_malloc(FSP_data->cn,FSP_data->cn);
    for (i = 0; i < FSP_data->cn;i++)
        for (j = 0; j < FSP_data->cn; j++)
	lambda_inv->p[i][j] = lambda->p[i][j];
    mat_LU_inv(lambda_inv);
    mat_mul(lambda_inv, xfinal, nu);

   /**********<< find eta >>************* NULLSPACE ********/ 
    temp1    = mat_malloc(1,1);
    temp2    = mat_malloc(1,1);
    mat_tra(nu);
    mat_mul(nu,cfinal,temp1);
    mat_sca(temp1,kfinal);
    mat_mul(nu,mfinal,temp2);
    mat_sca(temp2,bfinal);
    eta = -temp2->p[0][0] + temp1->p[0][0];
    eta /= pfinal;
    mat_tra(nu);
    mat_free(temp1);
    mat_free(temp2);

    /************<< find mu >>************ NULLSPACE ********/ 
    temp1 = mat_malloc(1,1);
    mat_tra(nu);
    mat_mul(nu,cfinal,temp1);
    mu = -(temp1->p[0][0] + (eta * dfinal));
    mu /= bfinal;
    mat_tra(nu);
    mat_free(temp1);

  }
  else IKerror(29,FATAL,"findt_with_Betas_Nonholonomic");   

  /**************************************************************
   *                                                            *
   *    Find the parametric Analytical Solutions 't'            *
   *                                                            *
   **************************************************************/
   /********** Calculate t's ***********/
   tfinal  = mat_malloc(SPAN,1);
   temp1   = mat_malloc(SPAN,1);
   temp2   = mat_malloc(SPAN,FSP_data->cn);
   temp3   = mat_malloc(SPAN,1);
   temp4   = mat_malloc(SPAN,1);
   temp5   = mat_malloc(SPAN,1);
   temp6   = mat_malloc(SPAN,1);
   mat_mul(Ginv,e,temp1);          /* temp1 = Ge         */
   mat_sca(temp1,mu);              /* temp1 = muGe       */
   mat_mul(Ginv,betas,temp2);     /* temp2 = GBeta      */
   mat_mul(temp2,nu,temp3);        /* temp3 = nuGBeta    */
   mat_mul(Ginv,alpha,temp4);      /* temp4 = Galpha     */
   mat_sca(temp4,eta);             /* temp4 = etaGalpha  */
   mat_add(temp1,temp3,temp5);
   mat_add(temp5,temp4,temp6);
   for (i = 0; i < SPAN; i++)  tfinal->p[i][0] = -(temp6->p[i][0]);
   mat_free(temp1);
   mat_free(temp2);
   mat_free(temp3);
   mat_free(temp4);
   mat_free(temp5);
   mat_free(temp6);

  for (i = 0, sum1 = 0; i < SPAN; i++)
      sum1 += tfinal -> p[i][0];
  if (!FSP_data->Null_Space) {
       if ((sum1< .99)||(sum1>1.01)) IKerror (21, FATAL,
		"findt_with_Betas_Nonholonomic");
  }
  else if ((sum1<-.01)||(sum1> .01)) IKerror (21, FATAL,
		"findt_with_Betas_Nonholonomic");

  if (DEBUG) {    
     fprintf(datafp, "  b= %9.4f\n", bfinal);
     fprintf(datafp, "  d= %9.4f\n", dfinal);
     fprintf(datafp, "  k= %9.4f\n", kfinal);
     fprintf(datafp, "  l= %9.4f\n", lfinal);
     fprintf(datafp, "  p= %9.4f\n", pfinal);
     fmat_pr(datafp, "c= ", cfinal);
     fmat_pr(datafp, "m= ", mfinal);
     fmat_pr(datafp, "r= ", rfinal);
     fmat_pr(datafp, "s= ", sfinal);
     fmat_pr(datafp, "z= ", zfinal);
     fmat_pr(datafp, "w= ", wfinal);
     fmat_pr(datafp,"delta = ", delta);
     fmat_pr (datafp, "Nu =", nu); 
     fprintf (datafp, "Mu = %9.4f \n", mu);
     fprintf (datafp, "Eta = %9.4f \n", eta);
     fmat_pr(datafp,"Term1", temp3);   
     fmat_pr(datafp,"Term2", temp2);
     fmat_pr (datafp, "*** t's for Collision ***", tfinal);

     fprintf(datafp, "\nsum ti:  %7.4f\n", sum1);
     for (j=0; j<FSP_data->cn; j++)
     {
        sum1 = 0;
        for (i = 0; i < SPAN; i++)
          sum1 += betas -> p[i][j] * tfinal -> p[i][0];
        fprintf(datafp, "\nsum betai*ti:  %7.4f \n", sum1);
     }
  }
  dq = mat_malloc(M, 1);
  for (i = 0; i < FSP_data->M; i++)  
  {
    dq->p[i][0]=0.0;
    for (j = 0; j < SPAN; j++)
      dq->p[i][0] += (tfinal -> p[j][0]) * (FSP_data->g->p[j][i]);
  }
  if (DEBUG) fmat_pr(datafp, "New dq reflecting constraints (t * g)", dq);

for (i=sum1=0;i<SPAN;i++) { sum1 = tfinal->p[i][0] * eta; } 
/*fprintf(stderr,"%11.6f  ",sum1);
fprintf(stderr,"%11.6f\n", Afinal->p[M-3][0]*dq->p[M-3][0] +
		 Afinal->p[M-2][0]*dq->p[M-2][0] +
		 Afinal->p[M-1][0]*dq->p[M-1][0]);
*/
  mat_free(Afinal);
  mat_free(tfinal);
  mat_free(nu);
  mat_free(Ginv);
  mat_free(cfinal);
  mat_free(sfinal);
  mat_free(mfinal);
  mat_free(rfinal);
  mat_free(zfinal);
  mat_free(wfinal);
  mat_free(xfinal);
  mat_free(delta);
  mat_free(lambda);
  mat_free(lambda_inv);
  mat_free(alphaT);
  mat_free(G);
  mat_free(e);
  mat_free(eT);
  mat_free(betas);

  return (dq);
}                 /* end findt */


/* ____________________________________________________
  | name:    findt_without_Betas_Nonholonomic          |
  | description:  find the parameterization for the    |
  |    solution vectors found using FSP blocking.  The |
  |    linear cmbination produced will be a full space |
  |    leastnorm motion.  This is called when there are|
  |    no constraints on the system.                   |
  |                                                    |
  | inputs:  g vectors                                 |
  | outputs: dq contains the new delta angles for step |
  |____________________________________________________|
  | Authors:     Date:     Modifications:              |
  | K Gower      10/95      Created                    |
  |                        e-mail: akgower@aol.com     |  
  |____________________________________________________| */

MATRIX *findt_without_Betas_Nonholonomic(FSP_data, B, H, datafp)
FILE      *datafp;        /* data file declaration       */
MATRIX    *B,             /* B, not Beta, in Pins paper  */
          *H;             /* H in Pins paper             */
Solutions *FSP_data;      /* See structures.h            */

{
MATRIX  *temp,
	*temp1,
	*temp2,
	*tfinal,	
	*alphaT,
 	*e, *eT,
	*G, *Ginv,
        *iden,
        *dq,
	*alpha,
	*Afinal;
float    ctemp,
         bfinal,
         dfinal,
	 lfinal,
	 kfinal,
	 pfinal,
	 eta,
	 mu;
int      i,j;
double   sum1 = 0, sum2 = 0; 

  if (DEBUG) fprintf(datafp, "\n\n# of constraints %d \n", FSP_data->cn);

  /* initialize the vectors of ones */
  e = mat_malloc(SPAN,1);
  eT= mat_malloc(1,SPAN);
  for (i = 0; i < SPAN; i++)   
  { e -> p[i][0] = eT -> p[0][i] = 1.0e0;}

  /* malloc's G, multiplies g's and returns */
  G = Build_Grammian2(FSP_data, B, datafp);  

  Ginv = mat_cp2(G);
  mat_LU_inv(Ginv);   
  if (DEBUG)
    {
    iden = mat_mul2(Ginv, G); 
    fmat_pr(datafp,"*** Identity Matrix Check (G * Ginv) ***", iden);
    mat_free(iden);
    }
  /***************************************************************
   *                                                             *
   *      b, d, k, l, p, eta, and mu are based on Katie's        *
   *      paper.                                  	         *
   *                                                             *
   ***************************************************************/

 /******** Generate A-vector ********/
  Afinal = mat_malloc(M,1);
  for (i = 0; i < M; i++) Afinal->p[i][0] = 0.0;
  Afinal->p[M - 3][0] = -sin(FSP_data->Qarray->p[M-1][0] + (PI/2));
  Afinal->p[M - 2][0] =  cos(FSP_data->Qarray->p[M-1][0] + (PI/2));
  Afinal->p[M - 1][0] = -(Robot->PLAT.Length)/2.25;  
/********* Generate alpha. *********/
  alpha = mat_malloc(SPAN,1);
    for (i = 0; i < SPAN; i++)
      alpha->p[i][0] = 0.0;
  for (i = 0; i < SPAN; i++)
    for (j = M - 3; j < M; j++)
      alpha->p[i][0] += Afinal->p[j][0] * FSP_data->g->p[i][j];
  alphaT=mat_malloc(1,SPAN);
  for (i = 0; i < SPAN; i++)
    alphaT->p[0][i] = alpha->p[i][0];

  /********** Calculate b's ***********/                         
  temp = mat_malloc(1,SPAN);
  bfinal = 0.0;
  for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += alphaT->p[0][j] * Ginv->p[j][i];
    }
  for (i = 0; i < SPAN; i++)
    bfinal += temp->p[0][i];
  mat_free(temp);                                                   

  /********** Calculate d's **********  */                     
  temp = mat_malloc(1,SPAN); 
  dfinal = 0.0;
  for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += alphaT->p[0][j] * Ginv->p[j][i];
    }
  for (i = 0; i < SPAN; i++)
    dfinal += temp->p[0][i] * alpha->p[i][0];
  mat_free(temp);
                                
  /********** Calculate k's ********** */                    
  kfinal = 0.0;
  for (i = 0; i < SPAN; i++)
    for (j = 0; j < SPAN; j++)
	kfinal += Ginv->p[i][j];

  /********** Calculate l's ********** */                     
  temp = mat_malloc(1,SPAN);
  lfinal = 0.0;
  for (i = 0; i < SPAN; i++)
    {
    temp->p[0][i] = 0.0;
    for (j = 0; j < SPAN; j++)
    	temp->p[0][i] += Ginv->p[j][i];
    }
  for (i = 0; i < SPAN; i++)
    lfinal += temp->p[0][i] * alpha->p[i][0];
  mat_free(temp);

  /********** Calculate p's **********  */ 
                     
  pfinal = (bfinal * lfinal) - (dfinal * kfinal);


 /**************************************************************
   *                                                            *
   *Find x, eta, mu and nu Lagrange multipliers for Null or Full*
   *                                                            *
   **************************************************************/
  /* This is the ONE SHOT Method, It is default */
/*  if ((STEP == ONE) && (!FSP_data->Null_Space))
  else if ((STEP == TWO) || (FSP_data->Null_Space))
  else IKerror(29,FATAL,"findt_without_Betas_Nonholonomic");    */
  { 
    /*************<< find eta >>********** FULLSPACE ********/
    eta = - (bfinal / pfinal );

    /*************<< find mu >>*********** FULLSPACE ********/
    mu = ( -( eta ) * dfinal ) / bfinal;
  }

  /**************************************************************
   *                                                            *
   *    Find the parametric Analytical Solutions 't'            *
   *                                                            *
   **************************************************************/
  /********** Calculate t's ***********/
  tfinal = mat_malloc(SPAN,1);
  temp1  = mat_malloc(SPAN,1);
  temp2  = mat_malloc(SPAN,1);
  mat_mul(Ginv,e,temp1);
  mat_mul(Ginv,alpha,temp2);       
  mat_sca(temp1,mu);
  mat_sca(temp2,eta);
  mat_add(temp1,temp2,tfinal);
  ctemp = -1;
  mat_sca(tfinal,ctemp);
  mat_free(temp1);
  mat_free(temp2);

  for (i = 0, sum1 = 0; i < SPAN; i++) sum1 += tfinal -> p[i][0];
  if (!FSP_data->Null_Space) {
       if ((sum1< .99)||(sum1>1.01)) IKerror (21, OK,
		"findt_without_Betas_Nonholonomic");
  }

  if (DEBUG) {    
     fprintf(datafp, "  b= %9.4f\n", bfinal);
     fprintf(datafp, "  d= %9.4f\n", dfinal);
     fprintf(datafp, "  k= %9.4f\n", kfinal);
     fprintf(datafp, "  l= %9.4f\n", lfinal);
     fprintf(datafp, "  p= %9.4f\n", pfinal);
     fprintf (datafp, "Mu = %9.4f \n", mu);
     fprintf (datafp, "Eta = %9.4f \n", eta);
     fmat_pr (datafp, "*** t's for Collision ***", tfinal);
     fprintf(datafp, "\nsum ti:  %7.4f\n", sum1);
  }

  dq = mat_malloc(M, 1);
  for (i = 0; i < FSP_data->M; i++)  
  {
    dq->p[i][0]=0.0;
    for (j = 0; j < SPAN; j++)
      dq->p[i][0] += (tfinal -> p[j][0]) * (FSP_data->g->p[j][i]);
  }

for (i=sum1=0;i<SPAN;i++) { sum1 = tfinal->p[i][0] * eta; } 
/*fprintf(stderr, "%11.6f  ", sum1);
fprintf(stderr, "<%11.6f, %11.6f, %11.6f> %11.6f\n",
		   Afinal->p[M-3][0]*dq->p[M-3][0],
		   Afinal->p[M-2][0]*dq->p[M-2][0],
		   Afinal->p[M-1][0]*dq->p[M-1][0],
		   Afinal->p[M-3][0]*dq->p[M-3][0] +
		   Afinal->p[M-2][0]*dq->p[M-2][0] +
		   Afinal->p[M-1][0]*dq->p[M-1][0]);

*/  if (DEBUG) fmat_pr(datafp, "New dq reflecting constraints (t * g)", dq);

  if (FSP_data->Null_Space) for(i=0;i<FSP_data->M; dq->p[i++][0] = 0.0);

  mat_free(Afinal);
  mat_free(tfinal);
  mat_free(Ginv);
  mat_free(alphaT);
  mat_free(G);
  mat_free(e);
  mat_free(eT);

  return (dq);
}                 /* end findt */
