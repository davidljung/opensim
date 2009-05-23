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
  
  $Id: Robot.cpp 1039 2004-02-11 20:50:52Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:50:52 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <robot/Robot>

#include <base/Matrix4>


using robot::Robot;

using base::Matrix4;
using base::Orient;
using base::array;


array<std::pair<String,String> > Robot::controlInterfaces() const
{
  return array<std::pair<String,String> >(); // empty (i.e. we don't have any interfaces by default)
}

  

Matrix4 Robot::coordFrameTransform(CoordFrame from, CoordFrame to,
				   Int manipulatorIndex,
				   const Matrix4& T,
				   const Point3& platformPosition, 
				   const Orient& platformOrientation) const
{
  if ((from == UnknownFrame) || (to == UnknownFrame))
    throw std::invalid_argument(Exception("can't convert to or from an unknown coordinate frame"));

  Matrix4 t;

  switch (from) {

  case EndEffectorFrame: {
    switch (to) {

    case EndEffectorFrame: {
    } break;
    
    case EndEffectorBaseFrame: {
      Unimplemented;
    } break;
    
    case BaseFrame: {
      t = T;
    } break;
    
    case MountFrame: {
      t = robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() * T;
    } break;
    
    case PlatformFrame: {
      Unimplemented;
    } break;
    
    case WorldFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      Matrix4 mtp; mtp.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      Matrix4 etm( robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() * T );
      t = ptw * mtp * etm;
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;



  case EndEffectorBaseFrame: {
    switch (to) {

    case EndEffectorFrame: {
      Unimplemented;
    } break;
    
    case EndEffectorBaseFrame: {
    } break;
    
    case BaseFrame: {
      Unimplemented;
    } break;
    
    case MountFrame: {
      Unimplemented;
    } break;
    
    case PlatformFrame: {
      Unimplemented;
    } break;
    
    case WorldFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      Matrix4 mtp; mtp.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      Matrix4 btm( robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() );
      Matrix4 eebtb; eebtb.setToTranslation( T.column(4).toVector3() );
      t = ptw * mtp * btm * eebtb;
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;



  case BaseFrame: {
    switch (to) {

    case EndEffectorFrame: { 
      t = T;
      t.invert();
    } break;
    
    case EndEffectorBaseFrame: {
      Unimplemented;
    } break;
    
    case BaseFrame: {
    } break;
    
    case MountFrame: {
      t = robotDescription->manipulators()[manipulatorIndex]->getBaseTransform();
    } break;
    
    case PlatformFrame: {
      Unimplemented;
    } break;
    
    case WorldFrame: {
      Matrix4 btm(robotDescription->manipulators()[manipulatorIndex]->getBaseTransform());
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      t.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      t = ptw * t * btm;
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;



  case MountFrame: {
    switch (to) {

    case EndEffectorFrame: {
      t = robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() * T;
      t.invert();
    } break;
    
    case EndEffectorBaseFrame: {
      Unimplemented;
    } break;
    
    case BaseFrame: {
      t = robotDescription->manipulators()[manipulatorIndex]->getBaseTransform();
      t.invert();
    } break;
    
    case MountFrame: {
    } break;
    
    case PlatformFrame: {
      t.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
    } break;
    
    case WorldFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      t.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      t = ptw * t;
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;



  case PlatformFrame: {
    switch (to) {

    case EndEffectorFrame: {
      Unimplemented;
    } break;
    
    case EndEffectorBaseFrame: {
      Unimplemented;
    } break;
    
    case BaseFrame: {
      Unimplemented;
    } break;
    
    case MountFrame: {
      t.setToTranslation(-robotDescription->manipulatorOffsets()[manipulatorIndex]);
    } break;
    
    case PlatformFrame: {
    } break;
    
    case WorldFrame: {
      t = Matrix4(platformOrientation.getRotationMatrix3());
      t.setTranslationComponent(platformPosition);
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;



  case WorldFrame: {
    switch (to) {

    case EndEffectorFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      Matrix4 mtp; mtp.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      Matrix4 etm( robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() * T );
      t = ptw * mtp * etm;
      t.invert();
    } break;
    
    case EndEffectorBaseFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      Matrix4 mtp; mtp.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      Matrix4 btm( robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() );
      Matrix4 eebtb; eebtb.setToTranslation( T.column(4).toVector3() );
      t = ptw * mtp * btm * eebtb;
      t.invert(); 
    } break;
    
    case BaseFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      Matrix4 mtp; mtp.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      Matrix4 btm( robotDescription->manipulators()[manipulatorIndex]->getBaseTransform() );
      t = ptw * mtp * btm;
      t.invert();      
    } break;
    
    case MountFrame: {
      Matrix4 ptw(platformOrientation.getRotationMatrix3());
      ptw.setTranslationComponent(platformPosition);
      t.setToTranslation(robotDescription->manipulatorOffsets()[manipulatorIndex]);
      t = ptw * t;
      t.invert();
    } break;
    
    case PlatformFrame: {
      t = Matrix4(platformOrientation.getRotationMatrix3());
      t.setTranslationComponent(platformPosition);
      t.invert();
    } break;
    
    case WorldFrame: {
    } break;
    
    default:
      throw std::invalid_argument(Exception("unknown to CoordFrame"));
    }
  } break;

  default:
    throw std::invalid_argument(Exception("unknown from CoordFrame"));
  }

  return t;
}



Robot::CoordFrame Robot::coordFrame(const String& frameString)
{
  CoordFrame frame = UnknownFrame;

  if (frameString == "world")
    frame = Robot::WorldFrame;
  else if (frameString == "ee")
    frame = Robot::EndEffectorFrame;
  else if (frameString == "eebase")
    frame = Robot::EndEffectorBaseFrame;
  else if (frameString == "base")
    frame = Robot::BaseFrame;
  else if (frameString == "mount")
    frame = Robot::MountFrame;
  else if (frameString == "platform")
    frame = Robot::PlatformFrame;

  return frame;
}


base::String Robot::coordFrame(Robot::CoordFrame coordFrame)
{
  switch (coordFrame) {
    case WorldFrame: return "world";
    case EndEffectorFrame: return "ee";
    case EndEffectorBaseFrame: return "eebase";
    case BaseFrame: return "base";
    case MountFrame: return "mount";
    case PlatformFrame: return "platform";
    case UnknownFrame: return "unknown";
    default: Assertm(false, "unknown coordFrame");
  }
  return "";
}

