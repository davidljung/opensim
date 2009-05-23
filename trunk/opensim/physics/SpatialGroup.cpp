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
  
  $Id: SpatialGroup.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/SpatialGroup>

using physics::SpatialGroup;

using base::Point3;
using base::Orient;
using base::Transform;


SpatialGroup::SpatialGroup()
  : implicit(false)
{
}

SpatialGroup::SpatialGroup(const String& name)
  : implicit(false)
{
  setName(name);
}

SpatialGroup::SpatialGroup(const SpatialGroup& sg)
 : Spatial(sg), spatials(sg.spatials), implicit(sg.implicit)
{
  if (!implicit) {
    position = sg.position;
    orient = sg.orient;
  }
  else
    spatial = sg.spatial;
    
}


SpatialGroup::~SpatialGroup()
{
}


SpatialGroup& SpatialGroup::operator=(const SpatialGroup& sg)
{
  spatials = sg.spatials;
  implicit = sg.implicit;
  if (!sg.implicit) {
    position = sg.position;
    orient = sg.orient;
  }
  else
    spatial = sg.spatial;
  
  setName(sg.getName());
  return *this;
} 


void SpatialGroup::setImplicitConfiguration(ref<Spatial> spatial)
{
  implicit = true;
  this->spatial = spatial;
}

void SpatialGroup::setExplicitConfiguration(const base::Transform& configuration)
{
  implicit = false;
  spatial = ref<Spatial>(0);
  position = configuration.getTranslation();
  orient = configuration.getRotation();
}

  
 
void SpatialGroup::add(ref<Spatial> spatial)
{
  Assert(spatial != 0);
  spatials.push_back(spatial); 
}

void SpatialGroup::remove(ref<Spatial> spatial)
{
  Assert(spatial != 0);
  spatials.remove(spatial);
}


void SpatialGroup::clear()
{
  spatials.clear();
}


void SpatialGroup::setConfiguration(const base::Transform& configuration)
{
  // for each Spatial, compute its relative position to us (i.e. its
  //  position in the coord. frame with origin (position, orient) (or spatial's configuration if implicit)
  // Then move it to a new world pos,orient such that its relative
  //  position is preserved in relation to our updated configuration
  Transform oldconfig( getPosition(), getOrientation() );
  if (configuration == oldconfig) return; // nothing to do
  Transform update = configuration*inverse(oldconfig);
  
  SpatialList::iterator s = spatials.begin();
  SpatialList::iterator end = spatials.end();
  while (s != end) {
    (*s)->setConfiguration( update * (*s)->getConfiguration() );
    s++;
  }

  if (!implicit) {
    position = configuration.getTranslation();
    orient = configuration.getRotation();
  }
}


void SpatialGroup::setPosition(const Point3& pos)
{
  setConfiguration( Transform(pos, getOrientation()) );
}
 
 
Point3 SpatialGroup::getPosition() const
{
  if (!implicit)
    return position;
  else
    return spatial->getPosition();
}
 
 
void SpatialGroup::setOrientation(const Orient& orient)
{
  setConfiguration( Transform(getPosition(), orient) );
}
 
 
Orient SpatialGroup::getOrientation() const
{
  if (!implicit)
    return orient;
  else
    return spatial->getOrientation();
}


void SpatialGroup::updateGroupPositionOrientation(const Point3& pos, const Orient& orient)
{
  if (!implicit) {
    position = pos;
    this->orient = orient;
  }
  else
    spatial->setPositionOrientation(pos, orient);
}


void SpatialGroup::updateGroupConfiguration(const base::Transform& configuration)
{
  if (!implicit) {
    position = configuration.getTranslation();
    orient = configuration.getRotation();
  }
  else
    spatial->setConfiguration(configuration);
}


