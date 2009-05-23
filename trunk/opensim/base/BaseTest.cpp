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
  
  $Id: BaseTest.cpp 1029 2004-02-11 20:45:54Z jungd $
  $Revision: 1.3 $
  $Date: 2004-02-11 15:45:54 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <base/BaseTest>

#include <base/array>
#include <base/Math>
#include <base/Transform>


using base::BaseTest;

using base::array;
using base::Math;
using base::Transform;
using base::inverse;



BaseTest::BaseTest()
{
}


void BaseTest::setUp() 
{ 

}
  

void BaseTest::tearDown()
{ 
}



void BaseTest::testarray()
{
  array<int> a0;          // size 0
  CPPUNIT_ASSERT(a0.size() == 0);

  array<int> a1(1);       // size 1
  a1[0]=0;
  CPPUNIT_ASSERT(a1.size() == 1);
  CPPUNIT_ASSERT(a1[0] == 0);

  array<int> a2(2,6);     // size 2, capacity 6
  CPPUNIT_ASSERT(a2.size() == 2);
  CPPUNIT_ASSERT(a2.capacity() == 6);

  // array assign
  a2[0]=a2[1]=-1;
  a1 = a2;
  CPPUNIT_ASSERT(a1.size() == 2);
  CPPUNIT_ASSERT(a1[0] == -1);
  CPPUNIT_ASSERT(a1[1] == -1);
  
  // auto resize
  a1.at(3) = -4;
  CPPUNIT_ASSERT(a1.size() == 4);
  CPPUNIT_ASSERT(a1[3] == -4);

  // swap
  a1.swap(a2);
  CPPUNIT_ASSERT(a1.size() == 2);
  CPPUNIT_ASSERT(a1[0] == -1);
  a1[0] = 1;

  // resize
  a1.resize(10);
  CPPUNIT_ASSERT(a1.size() == 10);
  a1[9] = 9;
  CPPUNIT_ASSERT(a1[0] == 1);
  CPPUNIT_ASSERT(a1[9] == 9);

  // clear
  a1.clear();
  CPPUNIT_ASSERT(a1.size() == 0);
  CPPUNIT_ASSERT(a1.empty());

  // operator==
  a1 = a2;
  CPPUNIT_ASSERT( a1 == a2 );
  a1.at(a1.size()) = -2;
  CPPUNIT_ASSERT( a1 != a2 );

  // sucessive resizing
  a1 = array<int>(0,2);
  for(int i=0; i<32; i++)
    a1.push_back(i);

  CPPUNIT_ASSERT( a1.size() == 32 );
  for(int i=0; i<32; i++)
    CPPUNIT_ASSERT( a1[i] == i );

  // iterators
  a2.resize(1);
  a2[0] = 5;
  CPPUNIT_ASSERT( a2.size() == 1 );
  CPPUNIT_ASSERT( (*a2.begin()) == 5 );

  array<int>::const_iterator i = a1.begin();
  array<int>::const_iterator e = a1.end();
  int c=0;
  while (i != e) {
    CPPUNIT_ASSERT( (*i) == c );
    ++i; ++c;
  }

}




void BaseTest::testTransform()
{
  // check identity
  Transform ti;
  CPPUNIT_ASSERT( ti.identity() );
  CPPUNIT_ASSERT( ti.isTransRotationOnly() );
  CPPUNIT_ASSERT( !ti.containsRotation() );
  CPPUNIT_ASSERT( !ti.containsTranslation() );
  
  Quat4 q1( Vector3(1,2,3), consts::Pi/3.0 ); // rot about axis by angle
  Transform r1(q1);
  
  // check construction from Quat4
  CPPUNIT_ASSERT( !r1.containsTranslation() );
  CPPUNIT_ASSERT( r1.containsRotation() );
  CPPUNIT_ASSERT( !r1.identity() );
  CPPUNIT_ASSERT( r1.isTransRotationOnly() );
  
  Quat4 q2(Vector3(1,2,1), consts::Pi/4.0);
  Transform r2(q2);
  
  // multiply and check against raw matrix multiply
  Transform r3( r1*r2 );

  Matrix3 m1 = Orient(q1);
  Matrix3 m2 = Orient(q2);
  
  Matrix3 m3( m1*m2 );
  
  CPPUNIT_ASSERT( Matrix4(m3,Point3()).equals( r3.getTransform() ) );
  CPPUNIT_ASSERT( Orient(m3).equals( r3.getRotation() ) );
  
  Matrix3 m3i( inverse(m3) );
  Transform r3i( inverse(r3) );
  
  CPPUNIT_ASSERT( Matrix4(m3i,Point3()).equals( r3i.getTransform() ) );
  CPPUNIT_ASSERT( Orient(m3i).equals( r3i.getRotation() ) );
  
  Transform ti2( r3*r3i );
  CPPUNIT_ASSERT( ti2.equals(ti) );
  
  
  // now do some checks that involve translation as well as rotation
  Transform t1(Vector3(3,2,4), q1);
  CPPUNIT_ASSERT( t1.containsTranslation() );
  CPPUNIT_ASSERT( t1.containsRotation() );
  CPPUNIT_ASSERT( !t1.identity() );
  CPPUNIT_ASSERT( t1.isTransRotationOnly() );

  Transform t1i( inverse(t1) );  
  Transform ti3( t1*t1i );
  CPPUNIT_ASSERT( ti3.equals(ti) );

  // check that converting the rotation component from mat->orient and back is invariant  
  Transform t2(Vector3(2,2,3), q2);
  Transform t2i( inverse(t2) ); // inverse always forces conversion to mat somewhere internally
  Transform t2ib( t2i );
  t2ib.setRotationComponent( t2ib.getRotation() );
  CPPUNIT_ASSERT( t2i.equals(t2ib) );
  
}



#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( BaseTest );
#endif

