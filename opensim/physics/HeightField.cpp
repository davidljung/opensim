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
  
  $Id: HeightField.cpp 1031 2004-02-11 20:46:36Z jungd $
  $Revision: 1.2 $
  $Date: 2004-02-11 15:46:36 -0500 (Wed, 11 Feb 2004) $
  $Author: jungd $
 
****************************************************************************/

#include <physics/HeightField>

#include <fstream>


using std::valarray;
using physics::HeightField;


HeightField::HeightField()
  : _nx(10), _ny(10), _dx(0.5), _dy(0.5), hf(new valarray<Real>(_nx*_ny))
{
}


HeightField::HeightField(const HeightField& heightfield)
  : _nx(heightfield._nx), _ny(heightfield._ny), _dx(heightfield._dx), _dy(heightfield._dy),
  hf(new valarray<Real>(*heightfield.hf))
{
}


HeightField::HeightField(Int nx, Int ny, Real dx, Real dy) throw(std::invalid_argument)
  : _nx(nx), _ny(ny), _dx(dx), _dy(dy), hf(0)
{
  if ((nx<2) || (ny<2) || (dx<=0.0) || (dy<=0.0))
    throw std::invalid_argument(Exception("Dimension or resolution arguments too small"));

  hf = new valarray<Real>(nx*ny);
}


HeightField::HeightField(ref<base::VFile> file) throw(std::invalid_argument, base::io_error)
  : hf(0)
{
  load(file);
}


HeightField::~HeightField()
{
  if (hf!=0)
    delete hf;
}


Real& HeightField::height(Real x, Real y) throw(std::out_of_range)
{
  Int offset = Int((y/_dy)*Real(_nx) + x/_dx);
  if (offset >= hf->size())
    throw std::out_of_range(Exception("x or y out of range"));

  return (*hf)[offset];
}

const Real& HeightField::height(Real x, Real y) const throw(std::out_of_range)
{
  Int offset = Int((y/_dy)*Real(_nx) + x/_dx);
  if (offset >= hf->size())
    throw std::out_of_range(Exception("x or y out of range"));

  return (*hf)[offset];
}


void HeightField::scale(Real s)
{
  _dx *= s;
  _dy *= s;
  scaleHeights(s);
}


void HeightField::scaleHeights(Real s)
{
  (*hf) *= s;
}

Real HeightField::maxHeight()
{
  return hf->max();
}

Real HeightField::minHeight()
{
  return hf->min();
}


void HeightField::setResolution(Real dx, Real dy) throw(std::invalid_argument)
{
  if ((dx<=0.0) || (dy<=0.0))
    throw std::invalid_argument(Exception("Resolution parameters too small"));

  _dx = dx; 
  _dy = dy;
}


void HeightField::load(ref<base::VFile> file) throw(std::invalid_argument, base::io_error)
{
  String extension = file->extension();
  std::istream& in = ((extension=="dat") || (extension=="thf"))?file->istream() // ASCII
		      :file->iostream(std::istream::in | std::istream::binary); // BINARY

  if (extension=="dat") {
    loadDATFile(in);
    return;
  }
  if (extension=="thf") {
    loadTHFFile(in);
    return;
  }

  throw std::invalid_argument(Exception("unsupported/unknown image file extension"));
}


void HeightField::save(ref<base::VFile> file) throw(std::invalid_argument, base::io_error)
{
  throw std::runtime_error(Exception("unimplemented"));
}

void HeightField::loadDATFile(std::istream& in) throw(base::io_error)
{
  throw base::io_error(Exception("DAT file loading not yet supported, sorry."));
}


void HeightField::loadTHFFile(std::istream& in) throw(base::io_error)
{
  valarray<Real>* hf;
  Int nx, ny;
  Real dx,dy;

  // !!!This is currently not very flexible about the layout of # directives etc.
  String header, version;
  in >> header;
  if (header != "#TextHeightField")
    throw base::io_error(Exception("not a valid TextHeightField (.thf) file - doesn't have correct #TextHeightField header line"));

  in >> version;
  if (version != "#version")
    throw base::io_error(Exception("not a valid TextHeightField (.thf) file - no #version directive"));

  Real v;
  in >> v;
  if (v != 1.0)
    throw base::io_error(Exception(String("unknown/unsupported TextHeightField (.thf) version ")+base::realToString(v)));

  String tag;
  in >> tag;
  if (tag=="xdim")
    in >> nx;
  else
    throw base::io_error(Exception(String(String("expected xdim got ")+tag)));

  in >> tag;
  if (tag=="ydim")
    in >> ny;
  else
    throw base::io_error(Exception(String(String("expected ydim got ")+tag)));

  in >> tag;
  if (tag=="xres")
    in >> dx;
  else
    throw base::io_error(Exception(String(String("expected xres got ")+tag)));

  in >> tag;
  if (tag=="yres")
    in >> dy;
  else
    throw base::io_error(Exception(String(String("expected yres got ")+tag)));
  
  if ((nx<2) || (ny<2) || (dx<=0.0) || (dy<=0.0))
    throw base::io_error(Exception("Dimension or resolution parameters too small"));

  hf = new valarray<Real>(nx*ny);

  Int i=0;
  while ( i<(nx*ny) && in>>(*hf)[i]) i++;

  this->hf = hf;
  _nx = nx;
  _ny = ny;
  _dx = dx;
  _dy = dy;
}


