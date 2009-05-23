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

  $Id: OrientTest.cpp 1092 2004-09-13 17:37:31Z jungd $
  $Revision: 1.6 $
  $Date: 2004-09-13 13:37:31 -0400 (Mon, 13 Sep 2004) $
  $Author: jungd $

****************************************************************************/

#include <base/OrientTest>

#include <base/Math>

#include <base/Transform>

using base::OrientTest;

using base::Math;
using base::Quat4;
using base::Vector3;
using base::Matrix;
using base::Vector;
using base::Transform;



OrientTest::OrientTest()
  : m(3,3)
{
}


void OrientTest::setUp()
{

}


void OrientTest::tearDown()
{
}



void OrientTest::testQuatMat()
{
  // test a basic Quat4 rotation
  Quat4 qr(Vector3(1,1,0),consts::Pi/2.0);
  Point3 up(1,0,0);
  CPPUNIT_ASSERT( qr.rotate(up).equals(Point3(0.5,0.5,-0.707107),1e-5) );


  // test some basic interconversions first
  Quat4 rotx45(Vector3(1,0,0),Math::degToRad(45));
  Orient o(rotx45);

  CPPUNIT_ASSERT( o.getQuat4() == rotx45 );

  m.reset( o.getRotationMatrix() );
  CPPUNIT_ASSERT(m.size1() == 3);
  CPPUNIT_ASSERT(m.size2() == 3);

  Orient o2(m);

  Matrix m2( o2.getRotationMatrix() );

  CPPUNIT_ASSERT( base::equals(m2,m) );


  Quat4 q(Vector3(1,1,0),consts::Pi/4.0);

  // test two paths for getting the same matrix into an Orient
  Orient oq(q);
  Matrix4 oqm(oq.getRotationMatrix3());
  Orient  om = Matrix3(oqm);
  Matrix4 omm(om.getRotationMatrix3());
  CPPUNIT_ASSERT( omm.equals(oqm) );

  // test Quat4 & Orient to matrix conversions are identical
  Orient q3(q);
  Matrix3 m3(q3.getRotationMatrix3());
  CPPUNIT_ASSERT( Matrix4(m3).equals(Matrix4(q)) );


  // test quat->Mat->quat
  Orient q3m(m3);
  Orient q3mq(q3m.getQuat4());

  Quat4 q31(q3.getQuat4());
  Quat4 q32(q3mq.getQuat4());
  Point3 u(1,2,3);
  Point3 u1(q31.rotate(u));
  Point3 u2(q32.rotate(u));
  Point3 u3(m3*u);
  Point3 u4(q.rotate(u));

  CPPUNIT_ASSERT( u1.equals(u2) );
  CPPUNIT_ASSERT( u1.equals(u3) );
  CPPUNIT_ASSERT( u1.equals(u4) );

  // test quat inverse
  Quat4 q4(Vector3(1,2,4), consts::Pi/3.0);
  Quat4 q4i( inverse(q4) );
  CPPUNIT_ASSERT( q4i.rotate(q4.rotate(u)).equals(u) );
  Matrix4 m4(q4);
  Matrix4 m4i(q4i);
  CPPUNIT_ASSERT( (m4i*m4*u).equals(u) );
  Matrix4 m4id( m4i*m4 );
  Matrix4 I; I.setIdentity();
  CPPUNIT_ASSERT( m4id.equals(I) );


  // test quat interpolation boundary cases
  Quat4 i1(q); i1.w=-i1.w; i1.normalize();
  Quat4 i2(q4);

  CPPUNIT_ASSERT( Quat4::interpolate(i1,i2,0).equals(i1) || Quat4::interpolate(i1,i2,0).equals(-i1) );
  CPPUNIT_ASSERT( Quat4::interpolate(i1,i2,1).equals(i2) || Quat4::interpolate(i1,i2,1).equals(-i2) );


  // test orient inverse
  Orient o4 = Quat4(Vector3(1,2,4), consts::Pi/3.0);
  Orient o4i = inverse(o4.getQuat4());
  CPPUNIT_ASSERT( o4i.rotate(o4.rotate(u)).equals(u) );
  Matrix4 om4(o4);
  Matrix4 om4i(o4i);
  CPPUNIT_ASSERT( (om4i*om4*u).equals(u) );
  Matrix4 om4id( om4i*om4 );
  CPPUNIT_ASSERT( om4id.equals(I) );
}



void OrientTest::testTransform()
{
  Quat4 q(0.158724,0.508859,0.458874,0.710847);
  Orient o(q);
  Transform t(q);

  Point3 p(1,2,3);
  CPPUNIT_ASSERT( q.rotate(p).equals(o.rotate(p)) );
  CPPUNIT_ASSERT( q.rotate(p).equals(t.rotate(p)) );
}



void OrientTest::testEulerRPY()
{
  // construct a EulerRPY manually as rotations about
  //  axes and see if we get what we expect

  Real roll = consts::Pi/3.1;
  Real pitch = consts::Pi/5.1;
  Real yaw = consts::Pi/2.1;

  // construct rotation about X, Y, Z axes in that order (static/fixed frames)
  Transform rotx = Quat4(Vector3(1,0,0), roll);
  Transform roty = Quat4(Vector3(0,1,0), pitch);
  Transform rotz = Quat4(Vector3(0,0,1), yaw);
  Transform qt( rotz*roty*rotx );

  Orient rpyt(roll,pitch, yaw);

  Point3 p(1,2,3); // test point to transform
  Point3 pqt( qt.rotate(p) ); // transform using manually constructed quat
  Point3 prpyt( rpyt.rotate(p) ); // transform using RPY
  Point3 pit( rotz.rotate(roty.rotate(rotx.rotate(p))) ); // transform via sucessive application of individual rotations in order

  CPPUNIT_ASSERT( pqt.equals(prpyt) );
  CPPUNIT_ASSERT( pqt.equals(pit) );


  // convert to a Quat and back and test preservation
  Orient q(rpyt);
  q.changeRepresentation(Orient::Quat);
  CPPUNIT_ASSERT( q.rotate(p).equals(pqt) ); // check is same transformation
  q.changeRepresentation(Orient::EulerRPY);

  CPPUNIT_ASSERT( q.equals(rpyt) );

  // convert to matrix and back
  q.changeRepresentation(Orient::Mat);
  CPPUNIT_ASSERT( Point3(q.getRotationMatrix3() * p).equals(pqt) ); // check is same transformation
  q.changeRepresentation(Orient::EulerRPY);

  CPPUNIT_ASSERT( q.equals(rpyt) );

  // convert to EulerZYXs and back
  q.changeRepresentation(Orient::EulerZYXs);
  q.changeRepresentation(Orient::EulerRPY);
  CPPUNIT_ASSERT( q.equals(rpyt) );

  // Try converting from EulerRPY to Quat, to Mat and back
  Orient o1(1.57071,-1.00007,0.800039);
  Orient o2(o1);
  o2.changeRepresentation(Orient::Quat);
  o2.changeRepresentation(Orient::Mat);
  o2.changeRepresentation(Orient::EulerRPY);
  CPPUNIT_ASSERT( o1.equals(o2) );

  // try another conversion from RPY->Quat and back at gymbal-lock config
  Orient o3(0,0,1.5708);
  Orient o4(o3);
  o3.changeRepresentation(Orient::Quat);
  o3.changeRepresentation(Orient::EulerRPY);
  CPPUNIT_ASSERT( o3.equals(o4) );
}


void OrientTest::testAngularVelConversion()
{
  // construct to angles close to each other
  //  using both Quat & EulerRPY reps, take the difference
  //  and convert to angular velocities and check the results
  //  are close
  Real eps = consts::Pi/50.0;
  Orient o1RPY(consts::Pi/5, -3*consts::Pi/4, 1.4*consts::Pi);
  Orient o2RPY(consts::Pi/5+eps, -3*consts::Pi/4+eps, 1.4*consts::Pi+2.0*eps);

  Orient o1Q(o1RPY); o1Q.changeRepresentation(Orient::Quat);
  Orient o2Q(o2RPY); o2Q.changeRepresentation(Orient::Quat);

  Vector diffRPY( Vector(o2RPY) - Vector(o1RPY) );
  Vector diffQ( Vector(o2Q) - Vector(o1Q) );

  Vector omegaFromRPY = o1RPY.getBinv()*diffRPY;
  Vector omegaFromQ = o1Q.getBinv()*diffQ;

  CPPUNIT_ASSERT( equals(omegaFromRPY,omegaFromQ, 0.005) );
}


void OrientTest::testRotationConcatenation()
{
  // with Quat rep

  Orient o1 = Quat4(Vector3(1,1,1), consts::Pi/4);
  Orient o2 = Quat4(Vector3(2,2,2), consts::Pi/3);

  // o1, o2
  Orient o3 = Quat4(Vector3(3,3,3), consts::Pi/4 + consts::Pi/3);

  CPPUNIT_ASSERT( o3.equals( Orient::concatenate(o1,o2) ) );

  // o2,-o1
  Orient mo1(o1); mo1.invert();
  Orient od = Quat4(Vector3(4,4,4), consts::Pi/3 - consts::Pi/4);
  Orient o4 = Orient::concatenate(o2,mo1);

  CPPUNIT_ASSERT( o4.equals(od) );


  // check by transforming a point
  Point3 p(1,2,-3);
  Point3 o2_p = o2.rotate(p);
  Point3 mo1_o2_p = mo1.rotate(o2_p);

  Point3 o4_p = o4.rotate(p);

  CPPUNIT_ASSERT( mo1_o2_p.equals(o4_p) );


  // check transforms concat properly
  Orient o5 = Quat4(Vector3(1,2,3), consts::Pi/2.1);
  Orient mo5(o5); mo5.invert();
  Orient o1mo5 = Orient::concatenate(mo5, o1);
  Point3 o5_p = o5.rotate(p);
  Point3 o1_mo5_o5_p = o1mo5.rotate(o5_p);
  Point3 o1_p = o1.rotate(p);

  CPPUNIT_ASSERT( o1_mo5_o5_p.equals(o1_p) );

}



#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( OrientTest );
#endif

