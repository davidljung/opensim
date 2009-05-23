#include <gfx/CLODTerrainDrawable>

using namespace gfx;
using namespace demeter;
using namespace osg;

DemeterDrawable::DemeterDrawable()
{
  _useDisplayList = false;
}

DemeterDrawable::DemeterDrawable(const DemeterDrawable& other,const osg::CopyOp& copyop):Drawable() 
{
  *this = other;
}

DemeterDrawable::~DemeterDrawable()
{
  m_RefTerrain->unref();
}

DemeterDrawable& DemeterDrawable::operator = (const DemeterDrawable& other) 
{ 
  m_RefTerrain = other.m_RefTerrain; // Increments ref count
  return *this;
}

/*
Object* DemeterDrawable::clone() const 
{ 
  DemeterDrawable* pNew = new DemeterDrawable;
  *pNew = *this;
  return pNew;
}
*/

void DemeterDrawable::SetTerrain(CLODTerrainRenderer* pTerrain)
{
  m_RefTerrain = pTerrain;
}

/*
const char* DemeterDrawable::className() const 
{ 
  return "CLODTerrainDrawable"; 
}
*/

void DemeterDrawable::setUseDisplayList(const bool flag)
{
  if (flag)
    Logln("CLODTerrainDrawable: Attempt to use display lists with DemeterDrawable ignored (not supported)");
  _useDisplayList = false;
}

void DemeterDrawable::drawImplementation(State& state) const
{
  CLODTerrainRenderer* pTerrain = m_RefTerrain.get();
  if (pTerrain)
    {
      pTerrain->ModelViewMatrixChanged();
      pTerrain->Render();
      pTerrain->DisableTextures();
    }
}

bool DemeterDrawable::computeBound() const
{
  //	Terrain* pTerrain = m_RefTerrain.get();
  if (m_RefTerrain.valid())
    {
      _bbox._min.x() = _bbox._min.y() = _bbox._min.z() = 0.0f;
      _bbox._max.x() = m_RefTerrain->GetWidth();
      _bbox._max.y() = m_RefTerrain->GetHeight();
      _bbox._max.z() = m_RefTerrain->GetMaxElevation();
      _bbox_computed = true;
    }
  return true;
}

