/****************************************************************************
  Copyright (C)2002 David Jung <opensim@pobox.com>

  This program/file is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details. (http://www.gnu.org)
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  
  $Id: OldIKOR.cpp 1036 2004-02-11 20:48:55Z jungd $
  $Revision: 1.5 $
  $Date: 2004-02-11 15:48:55 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/control/kinematics/OldIKOR>

#ifdef BUILDOLDIKOR
extern "C" {
#include <robot/control/oldikor/IKOR/general.h>
#include <robot/control/oldikor/IKOR/headers.h>
}
#endif


using robot::control::kinematics::OldIKOR;

using base::Vector;



OldIKOR::OldIKOR(const robot::KinematicChain& chain, bool platformActive)
  : chain(chain), platformActive(platformActive)
{
#ifdef BUILDOLDIKOR
  Init_Globals(); 
#endif
}


// globals needed by the old IKOR C code
#ifdef BUILDOLDIKOR
extern "C" {

  //extern FILE* gcheck;

static MATRIX* staticJ;
void GET_JACOB(MATRIX *Jacob,  MATRIX *Qarray)
{
  mat_cp(staticJ, Jacob);
}

void GET_ACTUAL_X(MATRIX *Qarray, MATRIX *x_of_link)
{
  abort();
}

}
#else
extern "C" {
void GET_JACOB(void*, void*) {}
void GET_ACTUAL_X(void*,void*) {}
}
#endif


/// \todo implement orientation components (convert to omega using euler_to_velocity() and fill out the mdx[3,4,5] components)
Vector OldIKOR::solve(const Vector& dx, const Vector& x, const Vector& q,
		      const base::Matrix& J,
		      OptimizationMethod    optMethod,
		      OptimizationCriterion optCriterion,
		      OptimizationConstraints optConstraints,
		      base::Orient::Representation  orientationRepresentation)
{
  Vector dq;

#ifdef BUILDOLDIKOR

  const Int N=J.size1(); 
  const Int M=J.size2();
  ::N = N;
  ::M = M;
  Solutions *FSP_data = Solutions_init(M,N);
  Robot->Weights->rows = Robot->Weights->cols = M;
  MATRIX *mJacob = mat_malloc(N, M);
  staticJ = mat_malloc(N, M);
  for(Int r=0; r<N; r++)
    for(Int c=0; c<M; c++)
      staticJ->p[r][c] = J(r,c);
  GET_JACOBIAN(mJacob, FSP_data->Qarray);
  MATRIX *mdx = mat_malloc(N, 1);
  mdx->p[0][0] = dx[0];
  mdx->p[1][0] = dx[1];
  mdx->p[2][0] = dx[2];
  FSP_data->cn = 0; 
  History   Old_DQs; 
  Old_DQs.whereami = -1;
  for (int i=0; i<HIST_SIZE; i++)
     Old_DQs.dq[i].DQ = mat_malloc( M , 1 );
  MATRIX *mdq = mat_malloc(M,1);
  int IKCriterion[5];
  IKCriterion[0]=0;
  int IKMethod[4];
  double spheredata[4][4];
  IKMethod[0]=0; IKMethod[1]=0; IKMethod[2]=0; IKMethod[3]=0;
  MATRIX *x_of_link = mat_malloc(M+1,N);
printf("$$$$$$2 M=%d N=%d\n",M,N);
  mdq = FSP (FSP_data, &Old_DQs, mJacob, mdx, IKCriterion, IKMethod, 
	 		LAGRANGIAN, 0, spheredata, x_of_link, datafp);

  for(int j=0; j<mdq->rows; j++) {
    dq[j] = mdq->p[j][0];
  }

#else
  throw std::runtime_error(Exception("unimplemented. (Old IKOR code was not included in the build - #define BUILDOLDIKOR)"));
#endif

  return dq;
}





void OldIKOR::setParameter(const String& name, Real value)
{
  throw std::invalid_argument(Exception("unknown parameter name"));
}


