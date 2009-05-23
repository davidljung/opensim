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
  
  $Id: GfxTest.cpp 1030 2004-02-11 20:46:17Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:17 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <gfx/GfxTest>

using gfx::GfxTest;

using gfx::Line3;
using gfx::Segment3;
using gfx::Triangle3;
using gfx::Plane;
using gfx::Quad3;
using gfx::Disc3;



GfxTest::GfxTest()
{
}


void GfxTest::setUp() 
{ 
  ul = Line3(Point3(0,0,0), Vector3(1,1,1));
  l = Line3(Point3(0,0,0), Vector3(1,2,3));
  us = Segment3(Point3(0,0,0), Point3(1,1,1).normalize());
  s = Segment3(Point3(0,0,0), Point3(1,1,1));
  s2 = Segment3(Point3(1,1,1), Point3(2,2,2));
  s3 = Segment3(Point3(1,2,3), Point3(4,6,7));
  
  q1 = Quad3(Point3(0,0,0),Point3(1,0,0),Point3(1,1,0), Point3(0,1,0));
  q2 = Quad3(Point3(2,0,0),Point3(3,0,0),Point3(3,1,0), Point3(2,1,0));
}
  

void GfxTest::tearDown()
{ 
}

 
void GfxTest::testLine()
{
  CPPUNIT_ASSERT( ul.contains(Point3(0,0,0)) );
  CPPUNIT_ASSERT( ul.contains(Point3(0.5,0.5,0.5)) );
  CPPUNIT_ASSERT( ul.contains(Point3(2,2,2)) );
  CPPUNIT_ASSERT( ul.contains(Point3(-2,-2,-2)) );
  
  CPPUNIT_ASSERT( l.contains(Point3(1,2,3)) );
  CPPUNIT_ASSERT( l.contains(Point3(2,4,6)) );
  
  CPPUNIT_ASSERT( Line3(Point3(0,0,0), Vector3(1,0,0)).pointClosestTo(Point3(0,1,0)).equals(Point3(0,0,0)) );
  
  CPPUNIT_ASSERT( Math::equals( l.distanceTo(Point3(2,4,6)), 0 ) ); 
  CPPUNIT_ASSERT( Math::equals( Line3(Point3(0,0,0), Vector3(1,0,0)).distanceTo(Point3(0.5,0,0)), 0 ) ); 
  CPPUNIT_ASSERT( Math::equals( Line3(Point3(0,0,0), Vector3(1,0,0)).distanceTo(Point3(0,2,0)), 2 ) ); 
  
  Line3 xl(Point3(0,0,0), Vector3(1,0,0));
  Line3 yl(Point3(0,-1,1), Vector3(0,1,0));
  Segment3 bxy(Point3(0,0,0), Point3(0,0,1));
  CPPUNIT_ASSERT( xl.shortestSegmentBetween(yl).equals(bxy) );
  CPPUNIT_ASSERT( Math::equals(xl.distanceTo(yl),1) );
}


void GfxTest::testSegment()
{
  CPPUNIT_ASSERT( s.contains(Point3(0,0,0)) );
  CPPUNIT_ASSERT( s.contains(Point3(0.5,0.5,0.5)) );
  CPPUNIT_ASSERT( s.contains(Point3(1,1,1)) );
  CPPUNIT_ASSERT( !s.contains(Point3(2,2,2)) );
  CPPUNIT_ASSERT( !s.contains(Point3(-2,-2,-2)) );
  CPPUNIT_ASSERT( !s.contains(Point3(1,1,1.5)) );
  CPPUNIT_ASSERT( !s.contains(Point3(2,2,2.5)) );
  CPPUNIT_ASSERT( !s.contains(Point3(-1,-1,-0.5)) );

  CPPUNIT_ASSERT( s.pointClosestTo(Point3(2,2,2)).equals(Point3(1,1,1)) );
  CPPUNIT_ASSERT( s.pointClosestTo(Point3(2,3,4)).equals(Point3(1,1,1)) );
  CPPUNIT_ASSERT( s.pointClosestTo(Point3(-1,-1,-1)).equals(Point3(0,0,0)) );
  CPPUNIT_ASSERT( s.pointClosestTo(Point3(-2,-1,-1)).equals(Point3(0,0,0)) );
  
  CPPUNIT_ASSERT( Segment3(Point3(0,0,0),Point3(1,0,0)).pointClosestTo(Point3(0.5,2,1)).equals(Point3(0.5,0,0)) );
  CPPUNIT_ASSERT( Math::equals(Segment3(Point3(0,0,0),Point3(1,0,0)).distanceTo(Point3(0.5,2,0)),2) );
  
  CPPUNIT_ASSERT( Math::equals(s.distanceTo(s2), 0) );
  Segment3 s4(Point3(3,3,3),Point3(4,4,4));
  CPPUNIT_ASSERT( Math::equals(s.distanceTo(s4), Math::sqrt(12)) );
  
  Segment3 xs(Point3(-0.5,0,0), Point3(1,0,0));
  Segment3 ys(Point3(0,-1,1), Point3(0,1,1));
  CPPUNIT_ASSERT( Math::equals( xs.distanceTo(ys), 1) );
  CPPUNIT_ASSERT( xs.shortestSegmentBetween(ys).equals( Segment3(Point3(0,0,0),Point3(0,0,1)) ) );
}


void GfxTest::testPlane()
{
  Plane pl1(Vector3(0,0,1),0);
  
  CPPUNIT_ASSERT( Math::equals( pl1.distanceTo(Point3(0,0,1)), 1) );
  CPPUNIT_ASSERT( pl1.contains(Point3(0,0,0)) );

  Plane pl2(Vector3(0,0,1),-1);

  CPPUNIT_ASSERT( pl2.contains(Point3(0,0,1)) );
  
  Plane pl3(Vector3(1,1,1),-1);
  Point3 p(1,1,1); p.normalize();
  CPPUNIT_ASSERT( pl3.contains(p) );
  CPPUNIT_ASSERT( !pl3.contains(Point3(2,2,2)) );
}

  
void GfxTest::testTriangle()
{
  Triangle3 txy(Point3(1,0,0), Point3(0,0,0), Point3(0,1,0));

  CPPUNIT_ASSERT( txy.pointClosestTo(Point3(20,-1,1)).equals(Point3(1,0,0)) );
  CPPUNIT_ASSERT( txy.pointClosestTo(Point3(0.3,0.3,1)).equals(Point3(0.3,0.3,0)) );
  CPPUNIT_ASSERT( txy.pointClosestTo(Point3(0.3,0.3,0)).equals(Point3(0.3,0.3,0)) );

  Segment3 ts(Point3(0.3,0.3,1),Point3(0.4,0.4,0.2));
  CPPUNIT_ASSERT( txy.shortestSegmentBetween(ts).equals(Segment3(Point3(0.4,0.4,0),Point3(0.4,0.4,0.2))) );
  
  Segment3 ts2(Point3(2,2,2), Point3(3,3,3));
  CPPUNIT_ASSERT( txy.shortestSegmentBetween(ts2).equals(Segment3(Point3(0.5,0.5,0), Point3(2,2,2))) );
}


void GfxTest::testQuad()
{
  CPPUNIT_ASSERT( Math::equals( q1.distanceTo(q2), 1) );
}



void GfxTest::testDisc()
{
  Disc3 d1(Point3(0,0,0), Vector3(0,0,1), 1);
  CPPUNIT_ASSERT( Math::equals( d1.distanceTo(Point3(0,0,1)), 1) );
  CPPUNIT_ASSERT( Math::equals( d1.distanceTo(Point3(0.5,0.5,1)), 1) );
  CPPUNIT_ASSERT( Math::equals( d1.distanceTo(Point3(0,2,0)), 1) );
}




#ifdef DEBUG
CPPUNIT_TEST_SUITE_REGISTRATION( GfxTest );
#endif

