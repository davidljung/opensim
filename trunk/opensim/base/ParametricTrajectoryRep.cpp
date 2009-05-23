/****************************************************************************
  Copyright (C)2003 David Jung <opensim@pobox.com>

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
  
  $Id: ParametricTrajectoryRep.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/ParametricTrajectoryRep>

#include <base/Serializer>

using base::ParametricTrajectoryRep;

using base::Time;

ParametricTrajectoryRep::ParametricTrajectoryRep(const ExpressionVector& p)
{
  Assert( (p.size() == 4) || (p.size() == 7) || (p.size() == 8) );

  if (p.size() != 4)
    flags = HasOrient;

  v.resize(p.size()-1);
  for(Int i=0; i<v.size(); i++)
    v[i] = p[i];
  
  // the function f, t = f(s) is required to be linear, so reformulate it
  //  by fitting a line to it, then test the fit and complain if it is bad
  Expression f = p[p.size()-1];

  Real a,b;
  times = f;
  linearFit(a,b);

  if (a <= 0.0)
    throw std::invalid_argument(Exception("the function t(s) must be linear and monotonically increasing"));

  // make a new f, lf(s) = sa + b
  Expression lf = Expression::p[0]*a + b;
  lf.simplify();
  
  // check fit
  Vector param(1);
  for(Real s=0; s<=1.0; s+=1/10.0) {
    param[0]=s;
    if (!Math::equals( lf.evaluate(param), f.evaluate(param) ) )
      throw std::invalid_argument(Exception("the function t(s) must be linear"));
  }
  
  times = lf;
  
}


void ParametricTrajectoryRep::linearFit(Real& a, Real &b) const
{
  // determine a,b s.t. t = as + b
  Real t0 = eval(0); // t0 = times(0)
  Real t1 = eval(1); // t1 = times(1)
  
  a = t1-t0;
  b = t0;
}


Time ParametricTrajectoryRep::time(Real s) const
{
  Math::bound<Real>(s,0,1);
  return eval(s);
}


void ParametricTrajectoryRep::shiftTime(const Time& dt)
{
  times = times + Expression(dt.seconds());
  times.simplify();
}


void ParametricTrajectoryRep::scaleTime(Real s)
{
  times = Expression(s)*times;
  times.simplify();
}


Real ParametricTrajectoryRep::gets(const Time& t) const
{
  // fit times = as + b, then compute s = (t-b)/a

  Real t0 = eval(0);
  Real t1 = eval(1);
  Real bt(t.seconds());
  Math::bound<Real>(bt,t0,t1);

  Real a,b;
  linearFit(a,b);
  
  if (a==0) return 0;
  
  return(bt-b)/a;
}


void ParametricTrajectoryRep::serialize(Serializer& s)
{
  ParametricPathRep::serialize(s);
  s(times,"times");
}


