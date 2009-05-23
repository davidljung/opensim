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
  
  $Id: SpatialTransform.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.1 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SpatialTransform>


using physics::SpatialTransform;

using base::Point3;
using base::Orient;
using base::Transform;

using physics::Spatial;



SpatialTransform::SpatialTransform()
{
}

SpatialTransform::SpatialTransform(const base::Transform& childTransform)
  : t(childTransform), tinv(inverse(childTransform))
{
}

SpatialTransform::SpatialTransform(ref<Spatial> child, const base::Transform& childTransform)
  : t(childTransform), tinv(inverse(childTransform)), child(child)
{
}

SpatialTransform::SpatialTransform(ref<Spatial> child, ref<Spatial> transformedRelativeTo)
  : child(child)
{
  if (child) 
    t = transformedRelativeTo->getConfiguration() * inverse(child->getConfiguration());
  else
    t = transformedRelativeTo->getConfiguration();
  tinv = inverse(t);
}

SpatialTransform::SpatialTransform(const String& name)
  : Spatial(name)
{
}

SpatialTransform::SpatialTransform(const SpatialTransform& st)
 : Spatial(st), t(st.t), tinv(st.tinv), child(st.child)
{
}


SpatialTransform::~SpatialTransform()
{
}


SpatialTransform& SpatialTransform::operator=(const SpatialTransform& st)
{
  t = st.t;
  tinv = st.tinv;
  child = st.child;
  setName(st.getName());
  return *this;
} 

 


void SpatialTransform::setConfiguration(const base::Transform& configuration)
{
  if (child) child->setConfiguration( configuration*t );
}


base::Transform SpatialTransform::getConfiguration() const
{
  if (child)
    return child->getConfiguration()*tinv;
  return Transform();
}


void SpatialTransform::setPosition(const Point3& pos)
{
  setConfiguration( Transform(pos, getOrientation()) );
}
 
 
Point3 SpatialTransform::getPosition() const
{
  return getConfiguration().getTranslation();
}
 
 
void SpatialTransform::setOrientation(const Orient& orient)
{
  setConfiguration( Transform(getPosition(), orient) );
}
 
 
Orient SpatialTransform::getOrientation() const
{
  return getConfiguration().getRotation();
}


void SpatialTransform::setTransform(const base::Transform& transform)
{
  t = transform;
  tinv = inverse(t);
}


base::Transform SpatialTransform::getTransform() const
{
  return t;
}


void SpatialTransform::setChild(ref<Spatial> child)
{
  this->child = child;
}


ref<Spatial> SpatialTransform::getChild() const 
{
  return child; 
}


