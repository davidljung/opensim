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
  
  $Id: PositionableOrientable.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/PositionableOrientable>

#include <base/Math>


using physics::PositionableOrientable;

using base::Math;
using base::Point3;
using base::Quat4;
using base::Orient;
using base::Vector3;
using base::Transform;



void PositionableOrientable::setPosition2D(const base::Point2& p, Real theta)
{
  Point3 pos( getPosition() );
  pos.x = p.x; pos.y = p.y;

  // convert the orientation to EulerZYXs, whos first component is the rotation about the Z axis
  Orient orient( getOrientation() ); orient.changeRepresentation(Orient::EulerZYXs);
  orient[0] = theta; // reset it with our new rotation about Z
  Transform configuration(orient);
  configuration.setTranslationComponent(pos);
  setConfiguration(configuration);
  
  //!!! temporarily just ignore the other orientation components
  /*
  Transform rotZTheta = Quat4(Vector3(0,0,1), theta);
  Transform configuration(rotZTheta);
  configuration.setTranslationComponent(pos);
  setConfiguration(configuration);
  */
  
}


Real PositionableOrientable::getOrientation2D() const
{
  // convert the orientation to EulerZYXs, whos first component is the rotation about the Z axis
  return Math::normalizeAngle(getOrientation().getVector(Orient::EulerZYXs)[0]);
}

