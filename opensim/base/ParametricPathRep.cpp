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
  
  $Id: ParametricPathRep.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/ParametricPathRep>

#include <base/Serializer>

using base::ParametricPathRep;

using base::Quat4;


ParametricPathRep::ParametricPathRep(const ExpressionVector& p)
  : v(p), flags(None)
{ 
  Assert( (p.size() == 3) || (p.size() == 6) || (p.size() == 7) );
  if (v.size() != 3)
    flags = HasOrient;
}



base::Point3 ParametricPathRep::position(Real s) const
{
  Vector param(1);
  param[0] = s;
  
  return Point3( v[0].evaluate(param), v[1].evaluate(param), v[2].evaluate(param) );
}


base::Orient ParametricPathRep::orientation(Real s) const
{
  if (!(flags & HasOrient))
    return Orient();

  Vector param(1);
  param[0] = s;

  if (v.size() == 6) // RPY
    return Orient( v[3].evaluate(param), v[4].evaluate(param), v[5].evaluate(param) );
  else
    return Orient( Quat4(v[3].evaluate(param), v[4].evaluate(param), v[5].evaluate(param), v[6].evaluate(param)) );
}


Real ParametricPathRep::distinguishedValue(Int i) const
{
  Math::bound<Int>(i,0,1); 
  return (i==0)?0.0:1.0;
}


Int ParametricPathRep::numDistinguishedValues() const
{
  return 2;
}






void ParametricPathRep::translate(const Vector3& t)
{
  v[0] = v[0] + Expression(t.x); v[0].simplify();
  v[1] = v[1] + Expression(t.y); v[1].simplify();
  v[2] = v[2] + Expression(t.z); v[2].simplify();
}


void ParametricPathRep::rotate(const Quat4& r)
{
  if (!(flags & HasOrient)) return;
  Unimplemented;
}


void ParametricPathRep::transform(const Matrix4& m)
{
  ExpressionMatrix em4 = base::toExpressionMatrix(base::fromMatrix4(m));
  
  // first transform position components
  ExpressionVector p(4);
  for(Int i=0; i<3; i++) p[i] = v[i]; // get position components
  p[3] = Expression(1);

  ExpressionVector tp = em4*p;
  
  for(Int i=0; i<3; i++) {
    tp[i].simplify();
    v[i] = tp[i];
  }
  
  
  // now transform orientations (if any)
  if (flags & HasOrient) {
  
    ExpressionMatrix em3(3,3); // extract rotation only component
    for(Int r=0; r<3; r++)
      for(Int c=0; c<3; c++)
        em3(r,c) = em4(r,c);
  
    Assertm(false,"transformations on the orientation components of a parametric path aren't implemented yet.");
  
  }
  
}


void ParametricPathRep::scalePosition(Real s)
{
  v[0] = v[0] * Expression(s); v[0].simplify();
  v[1] = v[1] * Expression(s); v[1].simplify();
  v[2] = v[2] * Expression(s); v[2].simplify();
}


void ParametricPathRep::serialize(Serializer& s)
{
  s(flags,"flags");
  Int dim = v.size();
  s(dim,"dim");
  if (v.size() != dim) v.resize(dim);
  for(Int i=0; i<dim; i++)
    s(v[i],String("v")+base::intToString(i));
}

