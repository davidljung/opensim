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
  
  $Id: LookAtCameraManipulator.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.6 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/LookAtCameraManipulator>

#include <base/Math>
#include <base/Vector3>
#include <base/Quat4>

using gfx::LookAtCameraManipulator;

using base::Math;
using base::Vector3;
using base::Quat4;


#include <osgGA/TrackballManipulator>
#include <osg/Notify>

using namespace osg;
using namespace osgGA;




LookAtCameraManipulator::LookAtCameraManipulator(Real alpha, Real theta, Real d,
						 Real targetx, Real targety, Real targetz)
{  
  halpha = Math::degToRad(alpha);
  htheta = Math::degToRad(theta);
  hd = Math::abs(d);
  target[0] = targetx;
  target[1] = targety;
  target[2] = targetz;
}


LookAtCameraManipulator::~LookAtCameraManipulator()
{
}


void LookAtCameraManipulator::setNode(osg::Node*)
{
}


const osg::Node* LookAtCameraManipulator::getNode() const
{
    return 0;
}


osg::Node* LookAtCameraManipulator::getNode()
{
    return 0;
}
/*
void LookAtCameraManipulator::updateCamera(GUIActionAdapter& us)
{
  _camera->setView(calcPos(), target, Vec3(0,0,1));
  _camera->ensureOrthogonalUpVector();
  us.requestRedraw();
}
*/

osg::Matrixd LookAtCameraManipulator::getMatrix() const
{
  osg::Matrix m;
  m.makeLookAt(calcPos(), target, osg::Vec3(0,0,1));
  return m;
}
                                                                                                                                                                                                                                            
osg::Matrixd LookAtCameraManipulator::getInverseMatrix() const
{
  return Matrixd::inverse(getMatrix());
}
                                                                                                                                                            
void LookAtCameraManipulator::setByMatrix(const osg::Matrixd& matrix)
{
  osg::Vec3 eye, center, up;
  // bug workaround (getLookAt() should be a const method)
  osg::Matrixd m(matrix);
  m.getLookAt(eye, center, up,1.0f);
  target = center;
  d = (eye-center).length();
}

void LookAtCameraManipulator::setByInverseMatrix(const osg::Matrixd& matrix)
{
  Matrixd im( Matrixd::inverse(matrix) );
  setByMatrix(im);
}
 
 


void LookAtCameraManipulator::home(const GUIEventAdapter& ,GUIActionAdapter& us)
{
  alpha = halpha;
  theta = htheta;
  d = hd;
}


void LookAtCameraManipulator::init(const GUIEventAdapter& ,GUIActionAdapter& )
{
}


bool LookAtCameraManipulator::handle(const GUIEventAdapter& ea,GUIActionAdapter& us)
{
  switch(ea.getEventType())
    {
    case(GUIEventAdapter::PUSH): {
      x = ea.getX();
      y = ea.getY();
      ialpha = alpha;
      itheta = theta;
      id = d;
      itarget = target;
      us.requestContinuousUpdate(true);
      return true;
    } break;
      
    case(GUIEventAdapter::RELEASE):
      {
        us.requestContinuousUpdate(tracking);
        return true;
      }
      
    case(GUIEventAdapter::DRAG): {
      Int buttonMask = ea.getButtonMask();

      if (buttonMask == GUIEventAdapter::LEFT_MOUSE_BUTTON) {
        Real dx = ea.getX() - x;
        Real dy = ea.getY() - y;
        alpha = ialpha - Math::degToRad(dx/8.0);
        if (alpha > 2*consts::Pi) alpha -= 2*consts::Pi;
        if (alpha < -2*consts::Pi) alpha += 2*consts::Pi;
        theta = itheta + Math::degToRad(dy/8.0);
        if (theta>Math::degToRad(89.9)) theta=Math::degToRad(89.9);
        if (theta<-Math::degToRad(89.9)) theta=-Math::degToRad(89.9);
        //updateCamera(us);
      }
      else if (buttonMask == GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
        Real dy = ea.getY() - y;
        d = id + dy/100.0;
        if (d<0.1) d=0.1;
        //updateCamera(us);
      }
      else if (buttonMask == (GUIEventAdapter::LEFT_MOUSE_BUTTON | GUIEventAdapter::RIGHT_MOUSE_BUTTON)) {
        Real dx = ea.getX() - x;
        Real dy = ea.getY() - y;
        Vector3 slide(-dx/100.0,dy/100.0,0);
        Quat4 rotz(Vector3(0,0,1), alpha);
        slide = rotz.rotate(slide);
        Quat4 rotx(Vector3(1,0,0), -theta); 
        slide = rotx.rotate(slide);
        target[0] = itarget[0] - slide.x;
        target[1] = itarget[1] - slide.y;
        target[2] = itarget[2] - slide.z; 
        //updateCamera(us);
      }
    } break;
      
    case(GUIEventAdapter::MOVE): {

      return false;
    } break;
      
    case(GUIEventAdapter::KEYDOWN): {
      if (ea.getKey()==' ') {
        home(ea,us);
        us.requestContinuousUpdate(tracking);
        return true;
      }
      else if (ea.getKey()=='C') {
        Logcln("Camera parameters: alpha=" << Math::radToDeg(alpha) << " theta=" << Math::radToDeg(theta) << " d=" << d << "  target=" << target);
      }
    } break;

    case(GUIEventAdapter::FRAME):
      //if (tracking) updateCamera(us);
      return tracking;
    default:
      return false;
    }
  return false;
}


Vec3 LookAtCameraManipulator::calcPos() const
{
  Vector3 eye(0,-d,0);
  Quat4 rotz(Vector3(0,0,1), alpha); // rot about z by alpha
  Quat4 rotx(Vector3(1,0,0), -theta); // rot about x by -theta
  Vector3 neweye( rotz.rotate(rotx.rotate(eye)) );

  return target+Vec3(neweye.x, neweye.y, neweye.z);
}

