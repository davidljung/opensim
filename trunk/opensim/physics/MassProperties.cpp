/****************************************************************************
  Copyright (C)1996 David Jung <opensim@pobox.com>

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
  
  $Id: MassProperties.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.4 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/MassProperties>
#include <physics/Material>
#include <gfx/TriangleContainer>
#include <gfx/TriangleIterator>
#include <gfx/Triangle3>

using physics::MassProperties;
using physics::Material;
using gfx::TriangleContainer;
using gfx::TriangleIterator;
using gfx::Triangle3;



MassProperties::MassProperties()
	: mass(1.0), centerOfMass(0,0,0)
{
}

MassProperties::MassProperties(const TriangleContainer& triangles, ref<const Material> material)
{
  computeMassProperties(triangles, material);
}

void MassProperties::setIbody(const Matrix3& Ibody) 
{
  _Ibody = Ibody;
  try {
    _IbodyInv = base::inverse(Ibody);
  } catch (std::invalid_argument& e) {
    Logln("Warning: Invalid Inertia matrix (body frame) - cannot be inverted. Setting to identity.");
    _Ibody.setIdentity();
    _IbodyInv.setIdentity();
  }
}

class MassProperties::VolData {
public:
  SInt A,B,C;
  Real P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb; // projection integrals
  Real Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca; // face integrals
  /// volume integrals
  Real T0;
  Real T1[4]; // using only 1..3
  Real T2[4];
  Real TP[4];
};


class MassProperties::WTriangle : public gfx::Triangle3
{
public:
  WTriangle() { compNorm(); compW(); }
  WTriangle(const gfx::Triangle3& t) : gfx::Triangle3(t) { compNorm(); compW(); }
  WTriangle(const WTriangle& t) : gfx::Triangle3(t), norm(t.norm), w(t.w) {}
  virtual ~WTriangle() {}

  void compNorm() { norm=normal(); }
  void compW() { w = -norm.x*_p1.x - norm.y*_p1.y - norm.z*_p1.z; }

  Vector3 norm;
  Real w;
};



void MassProperties::computeMassProperties(const TriangleContainer& triangles, ref<const Material> material)
{
  Int X=1, Y=2, Z=3;

  Matrix3 J; // inertia tensor
  Point3  r; // center of mass

  VolData v;
  compVolumeIntegrals(triangles, v);

  Real density = material->density();

  if ( Math::isNAN(v.T0) || Math::equals(v.T0,0) ) {
    Logln("Warning: Mass property calculation failed.  Setting inertia matrix to identity, mass to 1*density and center of mass to 0");
    _Ibody.setIdentity();
    _IbodyInv.setIdentity();
    mass = density * 1.0;
    centerOfMass = Point3(0,0,0);
  }
  else {
    mass = density * v.T0;
  
    // compute center of mass
    r.x = v.T1[X]/v.T0;
    r.y = v.T1[Y]/v.T0;
    r.z = v.T1[Z]/v.T0;

    // compute inertia tensor 
    J.e(1,1) = density * (v.T2[Y] + v.T2[Z]);
    J.e(2,2) = density * (v.T2[Z] + v.T2[X]);
    J.e(3,3) = density * (v.T2[X] + v.T2[Y]);
    J.e(1,2) = J.e(2,1) = - density * v.TP[X];
    J.e(2,3) = J.e(3,2) = - density * v.TP[Y];
    J.e(3,1) = J.e(1,3) = - density * v.TP[Z];

    // translate inertia tensor to center of mass 
    J.e(1,1) -= mass * (r.y*r.y + r.z*r.z);
    J.e(2,2) -= mass * (r.z*r.z + r.x*r.x);
    J.e(3,3) -= mass * (r.x*r.x + r.y*r.y);
    J.e(1,2) = J.e(2,1) += mass * r.x * r.y; 
    J.e(2,3) = J.e(3,2) += mass * r.y * r.z; 
    J.e(3,1) = J.e(1,3) += mass * r.z * r.x; 
    
    _Ibody = J;
    try {
      _IbodyInv = base::inverse(_Ibody);
    } catch (std::invalid_argument& e) {
      Logln("Warning: Inertia matrix (body frame) cannot be inverted - setting to identity");
      _Ibody.setIdentity();
      _IbodyInv.setIdentity();
    }
    
    centerOfMass = r;

    if (!centerOfMass.equals(Point3(0,0,0))) {
      Logln("Warning: Center of mass is not at coordinate origin - it is " << centerOfMass << ".");
    }

  }

}


void MassProperties::compVolumeIntegrals(const TriangleContainer& triangles, VolData& v)
{
  SInt X=1, Y=2, Z=3;

  TriangleContainer::const_iterator tris = triangles.begin();
  TriangleContainer::const_iterator endtris = triangles.end();
  
  Real nx, ny, nz;

  v.T0 = 0;
  v.T1[X] = v.T1[Y] = v.T1[Z] = 0;
  v.T2[X] = v.T2[Y] = v.T2[Z] = 0.0;
  v.TP[X] = v.TP[Y] = v.TP[Z] = 0.0;
  
  while (tris != endtris) {

    WTriangle f(*tris);

    nx = base::abs(f.norm.x);
    ny = base::abs(f.norm.y);
    nz = base::abs(f.norm.z);
    if (nx > ny && nx > nz)
      v.C = X;
    else
      v.C = (ny > nz) ? Y : Z;
    v.A = ((v.C + 1 -1)%3)+1;
    v.B = ((v.A + 1 -1)%3)+1;
    
    compFaceIntegrals(f,v);
    
    v.T0 += f.norm.x * ((v.A == X) ? v.Fa : ((v.B == X) ? v.Fb : v.Fc));
    
    v.T1[v.A] += f.norm[v.A] * v.Faa;
    v.T1[v.B] += f.norm[v.B] * v.Fbb;
    v.T1[v.C] += f.norm[v.C] * v.Fcc;
    v.T2[v.A] += f.norm[v.A] * v.Faaa;
    v.T2[v.B] += f.norm[v.B] * v.Fbbb;
    v.T2[v.C] += f.norm[v.C] * v.Fccc;
    v.TP[v.A] += f.norm[v.A] * v.Faab;
    v.TP[v.B] += f.norm[v.B] * v.Fbbc;
    v.TP[v.C] += f.norm[v.C] * v.Fcca;

    ++tris;
  }
  
  v.T1[X] /= 2.0; v.T1[Y] /= 2.0; v.T1[Z] /= 2.0;
  v.T2[X] /= 3.0; v.T2[Y] /= 3.0; v.T2[Z] /= 3.0;
  v.TP[X] /= 2.0; v.TP[Y] /= 2.0; v.TP[Z] /= 2.0;

}

void MassProperties::compFaceIntegrals(const WTriangle& t, VolData& v)
{
  Real w;
  Real k1, k2, k3, k4;

  compProjectionIntegrals(t,v);

  w = t.w;
  k1 = 1 / t.norm[v.C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

  v.Fa = k1 * v.Pa;
  v.Fb = k1 * v.Pb;
  v.Fc = -k2 * (t.norm[v.A]*v.Pa + t.norm[v.B]*v.Pb + w*v.P1);
  
  v.Faa = k1 * v.Paa;
  v.Fbb = k1 * v.Pbb;
  v.Fcc = k3 * (Math::sqr(t.norm[v.A])*v.Paa + 2*t.norm[v.A]*t.norm[v.B]*v.Pab + Math::sqr(t.norm[v.B])*v.Pbb
		+ w*(2*(t.norm[v.A]*v.Pa + t.norm[v.B]*v.Pb) + w*v.P1));
  
  v.Faaa = k1 * v.Paaa;
  v.Fbbb = k1 * v.Pbbb;
  v.Fccc = -k4 * (Math::cube(t.norm[v.A])*v.Paaa + 3*Math::sqr(t.norm[v.A])*t.norm[v.B]*v.Paab 
		  + 3*t.norm[v.A]*Math::sqr(t.norm[v.B])*v.Pabb + Math::cube(t.norm[v.B])*v.Pbbb
		  + 3*w*(Math::sqr(t.norm[v.A])*v.Paa + 2*t.norm[v.A]*t.norm[v.B]*v.Pab + Math::sqr(t.norm[v.B])*v.Pbb)
		  + w*w*(3*(t.norm[v.A]*v.Pa + t.norm[v.B]*v.Pb) + w*v.P1));
  
  v.Faab = k1 * v.Paab;
  v.Fbbc = -k2 * (t.norm[v.A]*v.Pabb + t.norm[v.B]*v.Pbbb + w*v.Pbb);
  v.Fcca = k3 * (Math::sqr(t.norm[v.A])*v.Paaa + 2*t.norm[v.A]*t.norm[v.B]*v.Paab + Math::sqr(t.norm[v.B])*v.Pabb
		 + w*(2*(t.norm[v.A]*v.Paa + t.norm[v.B]*v.Pab) + w*v.Pa));
}


void MassProperties::compProjectionIntegrals(const WTriangle& t, VolData& v)
{
   Real a0, a1, da;
   Real b0, b1, db;
   Real a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
   Real a1_2, a1_3, b1_2, b1_3;
   Real C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
   Real Cab, Kab, Caab, Kaab, Cabb, Kabb;

   v.P1 = v.Pa = v.Pb = v.Paa = v.Pab = v.Pbb = v.Paaa = v.Paab = v.Pabb = v.Pbbb = 0.0;

   for (int i = 1; i <= 3; i++) {
     a0 = t(i)[v.A];
     b0 = t(i)[v.B];
     a1 = t(((i+1-1)%3)+1)[v.A];
     b1 = t(((i+1-1)%3)+1)[v.B];
     da = a1 - a0;
     db = b1 - b0;
     a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
     b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
     a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
     b1_2 = b1 * b1; b1_3 = b1_2 * b1;
     
     C1 = a1 + a0;
     Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
     Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
     Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
     Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
     Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
     Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;
     
     v.P1 += db*C1;
     v.Pa += db*Ca;
     v.Paa += db*Caa;
     v.Paaa += db*Caaa;
     v.Pb += da*Cb;
     v.Pbb += da*Cbb;
     v.Pbbb += da*Cbbb;
     v.Pab += db*(b1*Cab + b0*Kab);
     v.Paab += db*(b1*Caab + b0*Kaab);
     v.Pabb += da*(a1*Cabb + a0*Kabb);
   }
   
   v.P1 /= 2.0;
   v.Pa /= 6.0;
   v.Paa /= 12.0;
   v.Paaa /= 20.0;
   v.Pb /= -6.0;
   v.Pbb /= -12.0;
   v.Pbbb /= -20.0;
   v.Pab /= 24.0;
   v.Paab /= 60.0;
   v.Pabb /= -60.0;
}


std::ostream& physics::operator<<(std::ostream& out, const MassProperties& mp) // Output
{
  return out << "Ibody:" << mp.Ibody() << std::endl 
	     << "mass:" << mp.mass << std::endl
	     << "centerOfMass:" << mp.centerOfMass << std::endl;
}
