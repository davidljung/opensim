//
// This code is a hacked up version of the Demeter Terrain
// code from http://www.terrainengine.com
// (hacked up because I removed the SDL and CommonC++
//  dependencies, and helpded it compile with gcc3.x)
// -David Jung.
//

// Demeter Terrain Visualization Library by Clay Fowler
// Copyright (C) 2001 Clay Fowler

/*
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.

You should have received a copy of the GNU Library General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA  02111-1307, USA.
*/

#include <iostream>
extern "C" {
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
}

#include <gfx/CLODTerrainRenderer>
#include <base/BitArray>


#include <string>

#include <osg/Image>

#include <osgDB/Registry>
#include <osgDB/ReadFile>

using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::string;

using namespace gfx;
using namespace demeter;

using base::BitArray;

#ifndef GLAPIENTRY
#define GLAPIENTRY
#endif

#define GL_CLAMP_TO_EDGE_EXT                0x812F
#define GL_TEXTURE0_ARB                     0x84C0
#define GL_TEXTURE1_ARB                     0x84C1
#define GL_COMBINE_RGB_EXT					0x8571
#define GL_ARRAY_ELEMENT_LOCK_FIRST_EXT     0x81A8
#define GL_ARRAY_ELEMENT_LOCK_COUNT_EXT     0x81A9
#define COMPRESSED_RGB_S3TC_DXT1_EXT        0x83F0

typedef void (GLAPIENTRY *PFNGLMULTITEXCOORD2FARBPROC)(GLenum texture,GLfloat s,GLfloat t);
typedef void (GLAPIENTRY *PFNGLACTIVETEXTUREARBPROC)(GLenum texture);
#ifdef _USE_VERTEX_ARRAYS_
typedef void (GLAPIENTRY *PFNGLLOCKARRAYSEXTPROC) (GLint first, GLsizei count);
typedef void (GLAPIENTRY *PFNGLUNLOCKARRAYSEXTPROC) (void);
#endif


PFNGLMULTITEXCOORD2FARBPROC glMultiTexCoord2fARB_ptr = NULL;
PFNGLACTIVETEXTUREARBPROC glActiveTextureARB_ptr = NULL;
#ifdef _USE_VERTEX_ARRAYS_
    PFNGLLOCKARRAYSEXTPROC glLockArraysEXT_ptr = NULL;
    PFNGLUNLOCKARRAYSEXTPROC glUnlockArraysEXT_ptr = NULL;
#endif


Settings* pSettingsInstance = NULL; // Singleton instance of the global settings

float numBlocks = 0.0f;
float numLevels = 0.0f;
float hashDelta = 0.0f;

GLuint CreateTexture(Uint8* pTexels,int width,int height,int rowLength,int border,int internalFormat,bool bClamp,bool bColorKey = false);
void LoadImage(char* szFilename,int* pWidth,int* pHeight,Uint8** pBuffer,bool bColorKey = false);
int RayPlaneIntersect(Ray *ray,Plane *plane,Vector* point,float *distance);
int RayBoxIntersect(Ray *ray,Box *box,Vector *point,float *distance);
bool IsPowerOf2(double number);

TerrainBlock::TerrainBlock(TerrainBlock* pParent)
{
    m_pChildren = NULL;
    m_pParent = pParent;
    m_pTriangleStrip = NULL;
#ifdef _USE_RAYTRACING_SUPPORT_
        m_pTriangles = NULL;
#endif
}

TerrainBlock::TerrainBlock(int homeVertex,int stride,Terrain* pTerrain,TerrainBlock* pParent)
{
    m_pTriangleStrip = NULL;
    m_pParent = pParent;
    m_HomeIndex = homeVertex;
    m_Stride = stride;
    static int numBlocksBuilt = 0;

        if (Settings::GetInstance()->IsVerbose())
        {
        if (hashDelta <= numBlocksBuilt++)
        {
            cout << "#" << flush;
            numBlocksBuilt = 0;
        }
        }
// Find this block's bounding box.
    float minElevation = pTerrain->GetElevation(homeVertex);
    float maxElevation = pTerrain->GetElevation(homeVertex);
    for (int i = homeVertex; i <= homeVertex + pTerrain->GetWidthVertices() * m_Stride; i += pTerrain->GetWidthVertices())
    {
      for (int j = i; j < i + m_Stride; j++)
        {
          float elevation = pTerrain->GetElevation(j);
          if (elevation < minElevation)
            minElevation = elevation;
          if (maxElevation < elevation)
            maxElevation = elevation;
        }
    }
    Vector boxUpperLeft,boxLowerRight,boxCenter;
    boxUpperLeft.x = pTerrain->m_pVertices[homeVertex].x + pTerrain->m_OffsetX;
    boxUpperLeft.y = pTerrain->m_pVertices[homeVertex].y + pTerrain->m_OffsetY;
    boxUpperLeft.z = maxElevation;
    boxLowerRight.x = pTerrain->m_pVertices[homeVertex + m_Stride * pTerrain->GetWidthVertices() + m_Stride].x + pTerrain->m_OffsetX;
    boxLowerRight.y = pTerrain->m_pVertices[homeVertex + m_Stride * pTerrain->GetWidthVertices() + m_Stride].y + pTerrain->m_OffsetY;
    boxLowerRight.z = minElevation;
    boxCenter.x = (boxUpperLeft.x + boxLowerRight.x) * 0.5f;
    boxCenter.y = (boxUpperLeft.y + boxLowerRight.y) * 0.5f;
    boxCenter.z = (boxUpperLeft.z + boxLowerRight.z) * 0.5f;
    m_BoundingBoxUpperCenter = boxCenter;
    m_BoundingBoxLowerCenter = boxCenter;
    m_BoundingBoxUpperCenter.z = boxUpperLeft.z;
    m_BoundingBoxLowerCenter.z = boxLowerRight.z;

// Recursively build children blocks of this block.
    if (2 < m_Stride)
    {
      m_pChildren = new TerrainBlock*[4];
      int childrenStride = m_Stride / 2;
      m_pChildren[0] = new TerrainBlock(homeVertex,																	childrenStride,pTerrain,this);
      m_pChildren[1] = new TerrainBlock(homeVertex + childrenStride,													childrenStride,pTerrain,this);
      m_pChildren[2] = new TerrainBlock(homeVertex + childrenStride * pTerrain->GetWidthVertices() + childrenStride,	childrenStride,pTerrain,this);
      m_pChildren[3] = new TerrainBlock(homeVertex + childrenStride * pTerrain->GetWidthVertices(),					childrenStride,pTerrain,this);
    }
    CalculateGeometry(pTerrain);
}

TerrainBlock::~TerrainBlock()
{
  m_pParent = NULL;
  m_pTriangleStrip = NULL;
  if (m_pChildren != NULL && 2 < m_Stride)
    {
      for (int i = 0; i < 4; i++)
        {
          delete m_pChildren[i];
          m_pChildren[i] = NULL;
        }
      delete[] m_pChildren;
    }
}

bool TerrainBlock::IsActive()
{
  return m_bIsActive;
}

void TerrainBlock::Tessellate(double* pMatModelView,double* pMatProjection,int* pViewport,TriangleStrip* pTriangleStrips,TriangleFan* pTriangleFans,int* pCountStrips,int* pCountFans,Terrain* pTerrain)
{
  if ((*pCountStrips < pTerrain->m_MaxNumberOfPrimitives) && pTerrain->CubeInFrustum(m_BoundingBoxUpperCenter.x,m_BoundingBoxUpperCenter.y,m_CubeCenterZ,m_CubeSize))
    {
      if (m_Stride == 2)
        {
          int offset;

          pTerrain->SetVertexStatus(m_HomeIndex,1);
          pTriangleStrips[*pCountStrips].m_pVertices[0] = m_HomeIndex;
          offset = m_HomeIndex + pTerrain->GetWidthVertices();
          pTerrain->SetVertexStatus(offset,1);
          pTriangleStrips[*pCountStrips].m_pVertices[1] = offset;
          offset = m_HomeIndex + 1;
          pTerrain->SetVertexStatus(offset,1);
          pTriangleStrips[*pCountStrips].m_pVertices[2] = offset;
          offset = m_HomeIndex + 1 + pTerrain->GetWidthVertices();
          pTerrain->SetVertexStatus(offset,1);
          pTriangleStrips[*pCountStrips].m_pVertices[3] = offset;
          offset = m_HomeIndex + 2;
          pTerrain->SetVertexStatus(offset,1);
          pTriangleStrips[*pCountStrips].m_pVertices[4] = offset;
          offset = m_HomeIndex + 2 + pTerrain->GetWidthVertices();
          pTerrain->SetVertexStatus(offset,1);
          pTriangleStrips[*pCountStrips].m_pVertices[5] = offset;
          pTriangleStrips[*pCountStrips].m_NumberOfVertices = 6;
          pTriangleStrips[*pCountStrips].m_bEnabled = true;
          *pCountStrips = *pCountStrips + 1;

          if (*pCountStrips < pTerrain->m_MaxNumberOfPrimitives)
            {
              offset = pTerrain->GetWidthVertices() + m_HomeIndex;
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[0] = offset;
              offset = pTerrain->GetWidthVertices() + m_HomeIndex + pTerrain->GetWidthVertices();
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[1] = offset;
              offset = pTerrain->GetWidthVertices() + m_HomeIndex + 1;
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[2] = offset;
              offset = pTerrain->GetWidthVertices() + m_HomeIndex + 1 + pTerrain->GetWidthVertices();
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[3] = offset;
              offset = pTerrain->GetWidthVertices() + m_HomeIndex + 2;
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[4] = offset;
              offset = pTerrain->GetWidthVertices() + m_HomeIndex + 2 + pTerrain->GetWidthVertices();
              pTerrain->SetVertexStatus(offset,1);
              pTriangleStrips[*pCountStrips].m_pVertices[5] = offset;
              pTriangleStrips[*pCountStrips].m_NumberOfVertices = 6;
              pTriangleStrips[*pCountStrips].m_bEnabled = true;
              *pCountStrips = *pCountStrips + 1;
            }

          m_bIsActive = true;
        }
      else
        {
          if (pTerrain->m_MaximumVisibleBlockSize < m_Stride)
            {
              m_pChildren[0]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
              m_pChildren[1]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
              m_pChildren[2]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
              m_pChildren[3]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
              m_bIsActive = false;
              m_bChildrenActive = true;
            }
          else
            {
              double screenTopX,screenTopY,screenTopZ,screenBottomX,screenBottomY,screenBottomZ;
              float halfWidth = m_CubeSize * 0.5f;
              // Check the vertical screen size of the block
              gluProject(m_BoundingBoxUpperCenter.x,m_BoundingBoxUpperCenter.y,m_BoundingBoxUpperCenter.z,pMatModelView,pMatProjection,pViewport,&screenTopX,&screenTopY,&screenTopZ);
              gluProject(m_BoundingBoxLowerCenter.x,m_BoundingBoxLowerCenter.y,m_BoundingBoxLowerCenter.z,pMatModelView,pMatProjection,pViewport,&screenBottomX,&screenBottomY,&screenBottomZ);
              float screenDistVertical = fabs(screenTopY - screenBottomY);
                                // Use both the X and Y axes to find the horizontal screen size of the block by using the larger of the two.
              gluProject(m_BoundingBoxUpperCenter.x - halfWidth,m_BoundingBoxUpperCenter.y,m_BoundingBoxUpperCenter.z,pMatModelView,pMatProjection,pViewport,&screenTopX,&screenTopY,&screenTopZ);
              gluProject(m_BoundingBoxLowerCenter.x + halfWidth,m_BoundingBoxLowerCenter.y,m_BoundingBoxLowerCenter.z,pMatModelView,pMatProjection,pViewport,&screenBottomX,&screenBottomY,&screenBottomZ);
              float screenDistX = fabs(screenTopX - screenBottomX);
              gluProject(m_BoundingBoxUpperCenter.x,m_BoundingBoxUpperCenter.y - halfWidth,m_BoundingBoxUpperCenter.z,pMatModelView,pMatProjection,pViewport,&screenTopX,&screenTopY,&screenTopZ);
              gluProject(m_BoundingBoxLowerCenter.x,m_BoundingBoxLowerCenter.y + halfWidth,m_BoundingBoxLowerCenter.z,pMatModelView,pMatProjection,pViewport,&screenBottomX,&screenBottomY,&screenBottomZ);
              float screenDistY = fabs(screenTopX - screenBottomX);
              float screenDistHorizontal = screenDistX < screenDistY ? screenDistY : screenDistX;

                                // Use the smaller of vertical and horizontal screen size to decide whether or not the block should be simplified.
              float screenDist = screenDistHorizontal < screenDistVertical ? screenDistHorizontal : screenDistVertical;

              if (screenDist <= pTerrain->GetDetailThreshold())
                {
                  // This block is simplified, so add its triangles to the list and stop recursing.
                  CreateTriangleStrip(pTriangleStrips,pCountStrips,pTerrain);
                  m_bIsActive = true;
                  m_bChildrenActive = false;
                }
              else
                {
                  m_pChildren[0]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
                  m_pChildren[1]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
                  m_pChildren[2]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
                  m_pChildren[3]->Tessellate(pMatModelView,pMatProjection,pViewport,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans,pTerrain);
                  m_bIsActive = false;
                  m_bChildrenActive = true;
                }
            }
        }
    }
  else
    {
      m_bIsActive = false;
      m_bChildrenActive = false;
    }
}

inline void TerrainBlock::CreateTriangleStrip(TriangleStrip* pTriangleStrips,int* pCount,Terrain* pTerrain)
{
  if (*pCount < pTerrain->m_MaxNumberOfPrimitives)
    {
      pTerrain->SetVertexStatus(m_HomeIndex,1);
      pTriangleStrips[*pCount].m_pVertices[0] = m_HomeIndex;
      int offset = m_HomeIndex + pTerrain->GetWidthVertices() * m_Stride;
      pTerrain->SetVertexStatus(offset,1);
      pTriangleStrips[*pCount].m_pVertices[1] = offset;
      offset = m_HomeIndex + m_Stride;
      pTerrain->SetVertexStatus(offset,1);
      pTriangleStrips[*pCount].m_pVertices[2] = offset;
      offset = m_HomeIndex + m_Stride + pTerrain->GetWidthVertices() * m_Stride;
      pTerrain->SetVertexStatus(offset,1);
      pTriangleStrips[*pCount].m_pVertices[3] = offset;
      pTriangleStrips[*pCount].m_bEnabled = true;
      pTriangleStrips[*pCount].m_NumberOfVertices = 4;

      m_pTriangleStrip = &pTriangleStrips[*pCount];
      *pCount = *pCount + 1;
    }
}

void TerrainBlock::EnableStrip(bool bEnabled)
{
  m_pTriangleStrip->m_bEnabled = false;
}

inline int TerrainBlock::GetStride()
{
  return m_Stride;
}

inline TerrainBlock* TerrainBlock::GetParent()
{
  return m_pParent;
}

inline int TerrainBlock::GetHomeIndex()
{
  return m_HomeIndex;
}

void TerrainBlock::RepairCracks(Terrain* pTerrain,TriangleStrip* pTriangleStrips,TriangleFan* pTriangleFans,int* pCountStrips,int* pCountFans)
{
  if (2 < m_Stride)
    {
      if (m_bIsActive)
        {
          int halfStride = m_Stride / 2;
          int bottomLeft = m_HomeIndex + m_Stride * pTerrain->GetWidthVertices();
          int bottomRight = bottomLeft + m_Stride;
          int i,previousVertex=0;
          int v0;
          int numVertices = 0;

          bool bNeedToFix = false;
          for (i = m_HomeIndex + m_Stride - 1; m_HomeIndex < i && !bNeedToFix; i--)
            bNeedToFix = (pTerrain->GetVertexStatus(i) == 1);
          if (!bNeedToFix)
            {
              for (i = m_HomeIndex + pTerrain->GetWidthVertices(); i < m_HomeIndex + m_Stride * pTerrain->GetWidthVertices() && !bNeedToFix; i += pTerrain->GetWidthVertices())
                bNeedToFix = (pTerrain->GetVertexStatus(i) == 1);
              if (!bNeedToFix)
                {
                  for (i = bottomLeft + 1; i < bottomRight && !bNeedToFix; i++)
                    bNeedToFix = (pTerrain->GetVertexStatus(i) == 1);
                  if (!bNeedToFix)
                    {
                      for (i = bottomRight - pTerrain->GetWidthVertices(); m_HomeIndex + m_Stride < i && !bNeedToFix; i -= pTerrain->GetWidthVertices())
                        bNeedToFix = (pTerrain->GetVertexStatus(i) == 1);
                    }
                }
            }

          if (bNeedToFix)
            {
              EnableStrip(false);
              v0 = m_HomeIndex + halfStride + halfStride * pTerrain->GetWidthVertices();
              Assert(0 <= v0);
              Assert(v0 < pTerrain->GetNumberOfVertices());
              pTriangleFans[*pCountFans].m_pVertices[0] = v0;
              numVertices = 0;
              for (i = m_HomeIndex + m_Stride; m_HomeIndex <= i; i--)
                {
                  Assert(0 <= i);
                  Assert(i < pTerrain->GetNumberOfVertices());
                  if (pTerrain->GetVertexStatus(i) == 1)
                    {
                      if (++numVertices == MAX_VERTICES_PER_FAN - 1)
                        {
                          // We have reached the maximum size for a fan, so start a new fan.
                          pTriangleFans[*pCountFans].m_NumberOfVertices = numVertices;
                          *pCountFans = *pCountFans + 1;
                          pTriangleFans[*pCountFans].m_pVertices[0] = v0;
                          pTriangleFans[*pCountFans].m_pVertices[1] = previousVertex;
                          numVertices = 2;
                        }
                      pTriangleFans[*pCountFans].m_pVertices[numVertices] = i;
                      previousVertex = i;
                    }
                }
              for (i = m_HomeIndex + pTerrain->GetWidthVertices(); i <= m_HomeIndex + m_Stride * pTerrain->GetWidthVertices(); i += pTerrain->GetWidthVertices())
                {
                  Assert(0 <= i);
                  Assert(i < pTerrain->GetNumberOfVertices());
                  if(pTerrain->GetVertexStatus(i) == 1)
                    {
                      if (++numVertices == MAX_VERTICES_PER_FAN - 1)
                        {
                          // We have reached the maximum size for a fan, so start a new fan.
                          pTriangleFans[*pCountFans].m_NumberOfVertices = numVertices;
                          *pCountFans = *pCountFans + 1;
                          pTriangleFans[*pCountFans].m_pVertices[0] = v0;
                          pTriangleFans[*pCountFans].m_pVertices[1] = previousVertex;
                          numVertices = 2;
                        }
                      pTriangleFans[*pCountFans].m_pVertices[numVertices] = i;
                      previousVertex = i;
                    }
                }
              for (i = bottomLeft; i <= bottomRight; i++)
                {
                  Assert(0 <= i);
                  Assert(i < pTerrain->GetNumberOfVertices());
                  if (pTerrain->GetVertexStatus(i) == 1)
                    {
                      if (++numVertices == MAX_VERTICES_PER_FAN - 1)
                        {
                          // We have reached the maximum size for a fan, so start a new fan.
                          pTriangleFans[*pCountFans].m_NumberOfVertices = numVertices;
                          *pCountFans = *pCountFans + 1;
                          pTriangleFans[*pCountFans].m_pVertices[0] = v0;
                          pTriangleFans[*pCountFans].m_pVertices[1] = previousVertex;
                          numVertices = 2;
                        }
                      pTriangleFans[*pCountFans].m_pVertices[numVertices] = i;
                      previousVertex = i;
                    }
                }
              for (i = bottomRight - pTerrain->GetWidthVertices(); m_HomeIndex + m_Stride <= i; i -= pTerrain->GetWidthVertices())
                {
                  Assert(0 <= i);
                  Assert(i < pTerrain->GetNumberOfVertices());
                  if(pTerrain->GetVertexStatus(i) == 1)
                    {
                      if (++numVertices == MAX_VERTICES_PER_FAN - 1)
                        {
                          // We have reached the maximum size for a fan, so start a new fan.
                          pTriangleFans[*pCountFans].m_NumberOfVertices = numVertices;
                          *pCountFans = *pCountFans + 1;
                          pTriangleFans[*pCountFans].m_pVertices[0] = v0;
                          pTriangleFans[*pCountFans].m_pVertices[1] = previousVertex;
                          numVertices = 2;
                        }
                      pTriangleFans[*pCountFans].m_pVertices[numVertices] = i;
                      previousVertex = i;
                    }
                }
              pTriangleFans[*pCountFans].m_NumberOfVertices = numVertices + 1;
              *pCountFans = *pCountFans + 1;
            }
        }
      else if (m_bChildrenActive)
        {
          m_pChildren[0]->RepairCracks(pTerrain,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans);
          m_pChildren[1]->RepairCracks(pTerrain,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans);
          m_pChildren[2]->RepairCracks(pTerrain,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans);
          m_pChildren[3]->RepairCracks(pTerrain,pTriangleStrips,pTriangleFans,pCountStrips,pCountFans);
        }
    }
}

void TerrainBlock::CalculateGeometry(Terrain* pTerrain)
{
  float height = m_BoundingBoxUpperCenter.z - m_BoundingBoxLowerCenter.z;
  float width = m_Stride * pTerrain->GetVertexSpacing();
  if (width < height)
    m_CubeSize = height;
  else
    m_CubeSize = width;
  m_CubeCenterZ = 0.5f * (m_BoundingBoxUpperCenter.z + m_BoundingBoxLowerCenter.z);

  float halfCube = m_CubeSize / 2.0f;
  m_BoundingBox.m_Min.x = m_BoundingBoxUpperCenter.x - halfCube;
  m_BoundingBox.m_Min.y = m_BoundingBoxUpperCenter.y - halfCube;
  m_BoundingBox.m_Min.z = m_CubeCenterZ - halfCube;
  m_BoundingBox.m_Max.x = m_BoundingBoxUpperCenter.x + halfCube;
  m_BoundingBox.m_Max.y = m_BoundingBoxUpperCenter.y + halfCube;
  m_BoundingBox.m_Max.z = m_CubeCenterZ + halfCube;

#ifdef _USE_RAYTRACING_SUPPORT_
  // Build triangles for ray intersection and collision detection.
  if (m_Stride == 2)
    {
      m_pTriangles = new Triangle[8];
      m_pTriangles[0].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex + 1]);
      m_pTriangles[1].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + 1],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 1]);
      m_pTriangles[2].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + 1],pTerrain->m_pVertices[m_HomeIndex + 1 + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex + 2]);
      m_pTriangles[3].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + 2],pTerrain->m_pVertices[m_HomeIndex + 1 + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex + 2 + pTerrain->GetWidthVertices()]);
      m_pTriangles[4].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex  + pTerrain->GetWidthVertices() + 1]);
      m_pTriangles[5].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 1],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex  + pTerrain->GetWidthVertices() + pTerrain->GetWidthVertices() + 1]);
      m_pTriangles[6].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 1],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 1 + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex  + pTerrain->GetWidthVertices() + 2]);
      m_pTriangles[7].DefineFromPoints(pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 2],pTerrain->m_pVertices[m_HomeIndex + pTerrain->GetWidthVertices() + 1 + pTerrain->GetWidthVertices()],pTerrain->m_pVertices[m_HomeIndex  + pTerrain->GetWidthVertices() + 2 + pTerrain->GetWidthVertices()]);*/
    }
#endif
}

void TerrainBlock::DummyFunc(Terrain* pTerrain)
{
  // Forces all exported methods to be referenced on GNU compilers - NEVER CALL
  printf("%d",pTerrain->GetHeightVertices());
  pTerrain->GetElevation(0.0f,0.0f);
  pTerrain->GetVertexSpacing();
  pTerrain->GetWidth();
  pTerrain->GetHeight();
  Settings::GetInstance()->GetScreenHeight();
  Settings::GetInstance()->GetScreenWidth();
}

void TerrainBlock::Write(FILE* file)
{
  fwrite(&m_Stride,sizeof(int),1,file);
  fwrite(&m_HomeIndex,sizeof(int),1,file);
  fwrite(&m_BoundingBoxUpperCenter,sizeof(Vector),1,file);
  fwrite(&m_BoundingBoxLowerCenter,sizeof(Vector),1,file);
  if (2 < m_Stride)
    {
      for (int i = 0; i < 4; i++)
        m_pChildren[i]->Write(file);
    }
}

void TerrainBlock::Read(FILE* file,Terrain* pTerrain)
{
  fread(&m_Stride,sizeof(int),1,file);
  fread(&m_HomeIndex,sizeof(int),1,file);
  fread(&m_BoundingBoxUpperCenter,sizeof(Vector),1,file);
  fread(&m_BoundingBoxLowerCenter,sizeof(Vector),1,file);
  if (2 < m_Stride)
    {
      m_pChildren = new TerrainBlock*[4];
      for (int i = 0; i < 4; i++)
        {
          m_pChildren[i] = new TerrainBlock(this);
          m_pChildren[i]->Read(file,pTerrain);
        }
    }
  CalculateGeometry(pTerrain);
}

Terrain::Terrain(char* szElevationsFilename,char* szTextureFilename,char* szDetailTextureFilename,float vertexSpacing,float elevationScale,int maxNumTriangles,bool bUseBorders) : Referenced()
{
  // Set all default values
  m_szSourceURL = NULL;
  m_MaxNumberOfPrimitives = maxNumTriangles / 4;
  m_pTriangleStrips = NULL;
  m_pTriangleFans = NULL;
  m_szTextureFilename = NULL;
  m_szDetailTextureFilename = new char[5];
  sprintf(m_szDetailTextureFilename,"none");
  m_pVertices = NULL;
  m_CommonTextureRepeats = 20.0f;
  m_pVertexStatus = NULL;
  m_pRootBlock = NULL;
  m_pCommonTexture = NULL;
  m_VertexSpacing = vertexSpacing;
  m_MaximumVisibleBlockSize = 128;
  Uint8 red;
  Uint8 green;
  Uint8 blue;
  // Load elevation data
  char* szFullFilename;
  Settings::GetInstance()->PrependMediaPath(szElevationsFilename,&szFullFilename);
  osg::ref_ptr<osg::Image> pImage = osgDB::Registry::instance()->readImage(szFullFilename,osgDB::Registry::CACHE_NONE).getImage();
  if (pImage == NULL)
    {
      String msg("Failed to load elevation image file '");
      throw new std::invalid_argument(Exception(msg));
    }
  else
    {
      // Test to see if the image is a power of 2 in both width and height.
      if (!IsPowerOf2((double)pImage->s()) || !IsPowerOf2((double)pImage->t()))
        {
          pImage->ensureValidSizeForTexturing(8192);
        }

      // Test to see if the image is 24-bit color.
      if (pImage->getPixelSizeInBits() != 24)
        {
          String msg("The elevation image file '");
          msg += szFullFilename;
          msg += "' is NOT a 24-bit image. Elevation files must be 24-bit images.";
          throw new std::invalid_argument(Exception(msg));
        }

      int pitch = 3 * pImage->s(); // why 3? !!!
      m_WidthVertices = pImage->s() + 1; // Add 1 dummy pixel line to edge for block strides
      m_HeightVertices = pImage->t() + 1;
      m_NumberOfVertices = m_WidthVertices * m_HeightVertices;
      m_pVertices = new Vector[m_WidthVertices * m_HeightVertices];
      int i,j;
      float x,y;
      Uint8* pImagePixels = (Uint8*)pImage->data();
      y = m_HeightVertices * m_VertexSpacing;
      m_MaxElevation = 0.0f;
      for (i = pImage->t() * pitch - pitch,j = 0; i >= 0; i -= pitch,y -= m_VertexSpacing)
        {
          if (i == pImage->t() * pitch)
            {
              x=0.0f;
              for (int m = 0; m < m_WidthVertices; m++,j++,x += m_VertexSpacing)
                {
                  m_pVertices[j].x = x;
                  m_pVertices[j].y = y;
                  m_pVertices[j].z = m_pVertices[j - m_WidthVertices].z;
                }
            }
          else
            {
              Uint8* pImageRow = pImagePixels + i;
              x=0.0f;
              int bytesPerPixel = pImage->getPixelSizeInBits() / 8;
              for (Uint8* pImagePixel = pImageRow; pImagePixel < pImageRow + pImage->s() * bytesPerPixel; pImagePixel += bytesPerPixel,j++,x += m_VertexSpacing)
                {
                  Uint32* pCurrentPixel = (Uint32*)pImagePixel;
                  //		  SDL_GetRGB(*pCurrentPixel,pImage->dataType(),&red,&green,&blue);
                  red   = (*pCurrentPixel) & 0xff; // !!!! check this is ok for Intel endian
                  green = ((*pCurrentPixel) & 0xff00) >> 8; // (we should really use dataType intelligently!)
                  blue  = ((*pCurrentPixel) & 0xff0000) >> 16;
                  m_pVertices[j].x = x;
                  m_pVertices[j].y = y;
                  m_pVertices[j].z = red * elevationScale;
                  if (m_MaxElevation < m_pVertices[j].z)
                    m_MaxElevation = m_pVertices[j].z;
                }
              m_pVertices[j].x = m_WidthVertices * m_VertexSpacing;
              m_pVertices[j].y = y;
              m_pVertices[j].z = m_pVertices[j - 1].z;
              if (m_MaxElevation < m_pVertices[j].z)
                m_MaxElevation = m_pVertices[j].z;
              j++; // Account for dummy column on right edge
              x += m_VertexSpacing;
            }
        }

      x = 0.0f;
      for (i = m_NumberOfVertices - m_WidthVertices; i < m_NumberOfVertices; i++)
        m_pVertices[i].z = m_pVertices[i - m_WidthVertices].z;

    }
  m_DetailThreshold = 4.0f;
  BuildBlocks();
  SetTexture(szTextureFilename,bUseBorders);
  if (szDetailTextureFilename != NULL)
    SetCommonTexture(szDetailTextureFilename);
  delete[] szFullFilename;
}

Terrain::Terrain(char* szCompiledMapFilename,int maxNumTriangles,bool bUseBorders,float offsetX,float offsetY) : Referenced()
{
  m_szSourceURL = NULL;
  Init(szCompiledMapFilename,maxNumTriangles,bUseBorders,offsetX,offsetY);
}

Terrain::Terrain(char* szURL,char* szCompiledMapFilename,int maxNumTriangles,bool bUseBorders) : Referenced()
{
  m_szSourceURL = new char[strlen(szURL) + 1];
  sprintf(m_szSourceURL,szURL);
  if (DownloadFile(szCompiledMapFilename))
    Init(szCompiledMapFilename,maxNumTriangles,bUseBorders);
}

Terrain::~Terrain()
{
  while (!m_Textures.empty())
    {
      vector<Texture*>::iterator iter = m_Textures.begin();
      Texture* pTexture = *iter;
      m_Textures.erase(iter);
      delete pTexture;
    }

  if (m_pCommonTexture != NULL)
    delete m_pCommonTexture;

  delete[] m_pTriangleStrips;
  delete[] m_pTriangleFans;
  delete[] m_pVertices;
  delete m_pVertexStatus;
  delete[] m_szTextureFilename;
  delete[] m_szDetailTextureFilename;
  delete m_pRootBlock;
}

void Terrain::Init(char* szCompiledMapFilename,int maxNumTriangles,bool bUseBorders,float offsetX,float offsetY)
{
  m_MaxNumberOfPrimitives = maxNumTriangles / 4;
  m_pTriangleStrips = NULL;
  m_pTriangleFans = NULL;
  m_pCommonTexture = NULL;
  m_DetailThreshold = 4.0f;
  m_pVertices = NULL;
  m_pRootBlock = NULL;
  m_pVertexStatus = NULL;
  m_szTextureFilename = NULL;
  m_szDetailTextureFilename = new char[5];
  sprintf(m_szDetailTextureFilename,"none");
  m_MaximumVisibleBlockSize = 24;
  m_CommonTextureRepeats = 20.0f;
  m_OffsetX = offsetX;
  m_OffsetY = offsetY;

  if(!Settings::GetInstance()->IsHeadless())
    {
#ifdef _USE_VERTEX_ARRAYS_
      glLockArraysEXT_ptr = (PFNGLLOCKARRAYSEXTPROC)SDL_GL_GetProcAddress("glLockArraysEXT");
      glUnlockArraysEXT_ptr = (PFNGLUNLOCKARRAYSEXTPROC)SDL_GL_GetProcAddress("glUnlockArraysEXT");
#endif
      if (glActiveTextureARB_ptr == NULL)
        {
          // !!! put something back here that check properly
          //	  glActiveTextureARB_ptr = (PFNGLACTIVETEXTUREARBPROC)SDL_GL_GetProcAddress("glActiveTextureARB");
          glActiveTextureARB_ptr = glActiveTextureARB;
          //	  glMultiTexCoord2fARB_ptr = (PFNGLMULTITEXCOORD2FARBPROC)SDL_GL_GetProcAddress("glMultiTexCoord2fARB");
          glMultiTexCoord2fARB_ptr = glMultiTexCoord2fARB;
          if(!glActiveTextureARB_ptr || !glMultiTexCoord2fARB_ptr)
            {
              cout << "TERRAIN: ERROR: Multitexture extensions not supported by this OpenGL vendor!" << endl;
              exit(-1);
            }
        }
      m_bMultiTextureSupported = true;
#ifdef _USE_VERTEX_ARRAYS_
      glVertexPointer(3,GL_FLOAT,0,m_pVertices);

      // Generate texture coordinate arrays
      float* pTextureMain = new float[m_NumberOfVertices * 2];
      float* pTextureDetail = new float[m_NumberOfVertices * 2];
      float u = 0.0f;
      float v = 0.0f;
      float deltaU = (float)m_NumberOfTextureTilesWidth / (float)m_WidthVertices;
      float deltaV = (float)m_NumberOfTextureTilesHeight / (float)m_HeightVertices;
      int k = 0;
      for (int i = 0; i < m_NumberOfVertices; i += m_WidthVertices)
        {
          u = 0.0f;
          for (int j = i; j < i + m_WidthVertices; j++)
            {
              pTextureMain[k] = u;
              pTextureMain[k + 1] = v;
              pTextureDetail[k] = u;// * 100.0f;
              pTextureDetail[k + 1] = v;// * 100.0f;
              k += 2;
              u += deltaU;
            }
          v += deltaV;
        }

      glActiveTextureARB_ptr(GL_TEXTURE0_ARB);
      glTexCoordPointer(2,GL_FLOAT,0,pTextureMain);
      glEnableClientState(GL_TEXTURE_COORD_ARRAY);

      glEnableClientState(GL_VERTEX_ARRAY);

      glLockArraysEXT_ptr(0,m_NumberOfVertices);
#endif
    }
  Read(szCompiledMapFilename,bUseBorders);
}

inline bool Terrain::IsMultiTextureSupported()
{
  return m_bMultiTextureSupported;
}

inline int Terrain::GetNumberOfVertices()
{
  return m_NumberOfVertices;
}

bool Terrain::Write(char* szCompiledMapFilename)
{
  bool bSuccess = false;
  try
    {
      char* szFullFilename;
      Settings::GetInstance()->PrependMediaPath(szCompiledMapFilename,&szFullFilename);
      FILE* file = fopen(szFullFilename,"wb");
      delete[] szFullFilename;
      int byteCount = 0;
      if (file != NULL)
        {
          fwrite(&m_VertexSpacing,sizeof(float),1,file);
          fwrite(&m_WidthVertices,sizeof(int),1,file);
          fwrite(&m_HeightVertices,sizeof(int),1,file);

          for (int i = 0; i < m_WidthVertices * m_HeightVertices; i++)
            {
              fwrite(&m_pVertices[i].z,sizeof(float),1,file);
              byteCount += sizeof(float);
            }

          int len = strlen(m_szTextureFilename);
          fwrite(&len,sizeof(int),1,file);
          byteCount += sizeof(int);
          fwrite(m_szTextureFilename,sizeof(char),strlen(m_szTextureFilename),file);
          byteCount += sizeof(char) * strlen(m_szTextureFilename);
          len = strlen(m_szDetailTextureFilename);
          fwrite(&len,sizeof(int),1,file);
          byteCount += sizeof(int);
          fwrite(m_szDetailTextureFilename,sizeof(char),strlen(m_szDetailTextureFilename),file);
          byteCount += sizeof(char) * strlen(m_szDetailTextureFilename);
          cout << "TERRAIN: Size = " << byteCount << endl;
          fclose(file);
          bSuccess = true;
        }
      else
        cout << "TERRAIN: Unable to create output file = " << szCompiledMapFilename << endl;
    }
  catch(...)
    {
    }
  return bSuccess;
}

bool Terrain::Read(char* szCompiledMapFilename,bool bUseBorders)
{
  bool bSuccess = false;
  char* szFullFilename;
  if (szCompiledMapFilename[0] != '/') // not absolute path
    Settings::GetInstance()->PrependMediaPath(szCompiledMapFilename,&szFullFilename);
  else {
    szFullFilename = new char[strlen(szCompiledMapFilename)+1];
    strcpy(szFullFilename, szCompiledMapFilename);
  }
  FILE* file = fopen(szFullFilename,"rb");
  if (file != NULL)
    {
      fread(&m_VertexSpacing,sizeof(float),1,file);
      fread(&m_WidthVertices,sizeof(int),1,file);
      fread(&m_HeightVertices,sizeof(int),1,file);
      m_NumberOfVertices = m_WidthVertices * m_HeightVertices;
      delete[] m_pVertices;
      m_pVertices = new Vector[m_WidthVertices * m_HeightVertices];

      int i = 0;

      m_MaxElevation = 0.0f;
      for (i = 0; i < m_NumberOfVertices; i++)
        {
          fread(&m_pVertices[i].z,sizeof(float),1,file);
          if (m_MaxElevation < m_pVertices[i].z)
            m_MaxElevation = m_pVertices[i].z;
        }

      for (i = 0; i < m_HeightVertices; i++)
        {
          for (int j = 0; j < m_WidthVertices; j++)
            {
              long int index = i * m_WidthVertices + j;
              Assert(index < m_NumberOfVertices);
              m_pVertices[index].x = (float)j * m_VertexSpacing;
              m_pVertices[index].y = (float)i * m_VertexSpacing;
            }
        }

      int len;
      fread(&len,sizeof(int),1,file);
      m_szTextureFilename = new char[len + 1];
      fread(m_szTextureFilename,sizeof(char),len,file);
      m_szTextureFilename[len] = '\0';

      fread(&len,sizeof(int),1,file);
      m_szDetailTextureFilename = new char[len + 1];
      fread(m_szDetailTextureFilename,sizeof(char),len,file);
      m_szDetailTextureFilename[len] = '\0';

      delete m_pVertexStatus;
      m_pVertexStatus = new base::BitArray(m_WidthVertices * m_HeightVertices);
      delete m_pRootBlock;
      BuildBlocks();
      if (SetTexture(m_szTextureFilename,bUseBorders))
        {
          if (strcmp(m_szDetailTextureFilename,"none") != 0)
            bSuccess = SetCommonTexture(m_szDetailTextureFilename);
          else
            bSuccess = true;
        }
      fclose(file);
    }
  else
    {
      string msg("Compiled map file '");
      msg += szFullFilename;
      msg += "' not found.";
      throw new std::invalid_argument(Exception(msg));
    }
  delete[] szFullFilename;
  return bSuccess;
}

void Terrain::UpdateNeighbor(Terrain* pTerrain,Terrain::DIRECTION direction)
{
  int thisVertex,otherVertex;
  if (direction == Terrain::DIR_SOUTH)
    {
      for (thisVertex = 0,otherVertex = m_NumberOfVertices - m_WidthVertices; thisVertex < m_WidthVertices; thisVertex++,otherVertex++)
        {
          if (GetVertexStatus(thisVertex))
            pTerrain->SetVertexStatus(otherVertex,true);
        }
    }
  else if (direction == Terrain::DIR_NORTH)
    {
      for (thisVertex = m_NumberOfVertices - m_WidthVertices,otherVertex = 0; thisVertex < m_NumberOfVertices; thisVertex++,otherVertex++)
        {
          if (GetVertexStatus(thisVertex))
            pTerrain->SetVertexStatus(otherVertex,true);
        }
    }
  else if (direction == Terrain::DIR_WEST)
    {
      for (thisVertex = 0,otherVertex = m_WidthVertices - 1; thisVertex < m_NumberOfVertices; thisVertex += m_WidthVertices,otherVertex += m_WidthVertices)
        if (GetVertexStatus(thisVertex))
          pTerrain->SetVertexStatus(otherVertex,true);
    }
  else if (direction == Terrain::DIR_EAST)
    {
      for (thisVertex = m_WidthVertices - 1,otherVertex = 0; thisVertex < m_NumberOfVertices; thisVertex += m_WidthVertices,otherVertex += m_WidthVertices)
        if (GetVertexStatus(thisVertex))
          pTerrain->SetVertexStatus(otherVertex,true);
    }
  else if (direction == Terrain::DIR_NORTHWEST)
    {
      if (GetVertexStatus(m_NumberOfVertices - m_WidthVertices))
        pTerrain->SetVertexStatus(m_WidthVertices - 1,true);
    }
  else if (direction == Terrain::DIR_NORTHEAST)
    {
      if (GetVertexStatus(m_NumberOfVertices - 1))
        pTerrain->SetVertexStatus(0,true);
    }
  else if (direction == Terrain::DIR_SOUTHEAST)
    {
      if (GetVertexStatus(m_WidthVertices - 1))
        pTerrain->SetVertexStatus(m_NumberOfVertices - m_WidthVertices,true);
    }
  else if (direction == Terrain::DIR_SOUTHWEST)
    {
      if (GetVertexStatus(0))
        pTerrain->SetVertexStatus(m_NumberOfVertices - 1,true);
    }
}

inline float Terrain::GetElevation(int index)
{
  return m_pVertices[index].z;
}

inline float Terrain::GetElevation(float x,float y)
{
  Plane plane;
  int	vertexID;
  float elevation;

  x -= m_OffsetX;
  y -= m_OffsetY;

#ifdef _PROTECT_ACCESS_
  if (x < 0.0f || y < 0.0f || GetWidth() < x || GetHeight() < y)
    elevation = 0.0f;
  else
    {
#endif
      vertexID = ((int)(y / m_VertexSpacing)) * m_WidthVertices + ((int)(x / m_VertexSpacing));

      if ((fmod(y,m_VertexSpacing) + fmod(x,m_VertexSpacing)) <= m_VertexSpacing)
        plane.defineFromPoints(m_pVertices[vertexID],m_pVertices[vertexID + m_WidthVertices],m_pVertices[vertexID + 1]);
      else
        plane.defineFromPoints(m_pVertices[vertexID + 1],m_pVertices[vertexID + 1 + m_WidthVertices],m_pVertices[vertexID + m_WidthVertices]);

      elevation = -1.0f * ((plane.a * x + plane.b * y + plane.d) / plane.c);
#ifdef _PROTECT_ACCESS_
    }
#endif

  return elevation;
}

inline void Terrain::GetNormal(float x,float y,float& normalX,float& normalY,float& normalZ)
{
  Plane plane;
  int	vertexID;
#ifdef _PROTECT_ACCESS_
  if (x < 0.0f || y < 0.0f || GetWidth() < x || GetHeight() < y)
    {
      normalX = normalY = 0.0f;
      normalZ = 1.0f;
    }
  else
    {
#endif
      vertexID = (int)(y / m_VertexSpacing) * m_WidthVertices + (int)(x / m_VertexSpacing);

      if ((fmod(y,m_VertexSpacing) - fmod(x,m_VertexSpacing)) >= 0.0f)
        plane.defineFromPoints(m_pVertices[vertexID],m_pVertices[vertexID + 1],m_pVertices[vertexID + m_WidthVertices]);
      else
        plane.defineFromPoints(m_pVertices[vertexID + 1],m_pVertices[vertexID + 1 + m_WidthVertices],m_pVertices[vertexID + m_WidthVertices]);

      normalX = plane.a;
      normalY = plane.b;
      normalZ = plane.c;
#ifdef _PROTECT_ACCESS_
    }
#endif
}

void Terrain::SetDetailThreshold(float threshold)
{
  m_DetailThreshold = threshold;
}

float Terrain::GetDetailThreshold()
{
  return m_DetailThreshold;
}

int Terrain::GetWidthVertices()
{
  return m_WidthVertices;
}

int Terrain::GetHeightVertices()
{
  return m_HeightVertices;
}

float Terrain::GetWidth() const
{
  return (float)(m_WidthVertices - 1) * m_VertexSpacing;
}

float Terrain::GetHeight() const
{
  return (float)(m_HeightVertices - 1) * m_VertexSpacing;
}

float Terrain::GetMaxElevation() const
{
  return m_MaxElevation;
}

float Terrain::GetVertexElevation(int index) const
{
#ifdef _PROTECT_ACCESS_
  if (index < 0 || m_NumberOfVertices <= index)
    return 0.0f;
  else
#endif
    return m_pVertices[index].z;
}

void Terrain::SetVertexElevation(int index,float newElevation)
{
#ifdef _PROTECT_ACCESS_
  if (0 <= index && index < m_NumberOfVertices)
#endif
    m_pVertices[index].z = newElevation;
}

float Terrain::GetVertexSpacing()
{
  return m_VertexSpacing;
}

void Terrain::BuildBlocks()
{
  if (Settings::GetInstance()->IsHeadless())
    return;
  numLevels = 0.0f;
  numBlocks = 0.0f;
  for (int i = m_WidthVertices - 1; 2 <= i; i /= 2)
    numLevels += 1.0f;
  for (double j = 0.0f; j < numLevels; j += 1.0f)
    numBlocks += pow(4,j);
  if (Settings::GetInstance()->IsVerbose())
    {
      cout << "TERRAIN: Building " << numBlocks << " blocks; please wait..." << endl;
#ifdef _USE_RAYTRACING_SUPPORT_
      cout << "TERRAIN: Memory required at runtime for blocks = " << numBlocks * (sizeof(TerrainBlock) + 8 * sizeof(Triangle)) << " bytes" << endl;
#else
      cout << "TERRAIN: Memory required at runtime for blocks = " << numBlocks * sizeof(TerrainBlock) << " bytes" << endl;
#endif
      cout << ".............................." << endl;
      hashDelta = (double)numBlocks / 30.0f;
      cout << "#" << flush;
    }
  m_pVertexStatus = new BitArray(m_WidthVertices * m_HeightVertices);
  // We assume that the terrain's width is always a power of 2 + 1!
  m_pRootBlock = new TerrainBlock(0,m_WidthVertices - 1,this,NULL);
  if (Settings::GetInstance()->IsVerbose())
    cout << endl;
}

inline void Terrain::SetVertexStatus(int vertexIndex,bool status)
{
  if (status)
    m_pVertexStatus->SetBit(vertexIndex);
  else
    m_pVertexStatus->ClearBit(vertexIndex);
}

inline bool Terrain::GetVertexStatus(int vertexIndex)
{
  return m_pVertexStatus->IsBitSet(vertexIndex);
}

int Terrain::Tessellate()
{
  if (m_pTriangleStrips == NULL)
    {
      long int maxNumStrips = (GetWidthVertices() - 1) * (GetHeightVertices() - 1);
      try
        {
          if (m_MaxNumberOfPrimitives < maxNumStrips)
            maxNumStrips = m_MaxNumberOfPrimitives;
          if (Settings::GetInstance()->IsVerbose())
            cout << "TERRAIN: Allocating " << maxNumStrips << " triangle strips and fans (" << maxNumStrips * sizeof(TriangleStrip) + maxNumStrips * sizeof(TriangleFan) << " bytes)\n" << endl;
          m_pTriangleStrips = new TriangleStrip[maxNumStrips];
          m_pTriangleFans = new TriangleFan[maxNumStrips];
          if (m_pTriangleStrips == NULL || m_pTriangleFans == NULL)
            {
              cout << "TERRAIN: " << "Not enough memory to build terrain triangles" << endl;
              exit(1);
            }
        }
      catch(...)
        {
          cout << "TERRAIN: " << "Not enough memory to build terrain triangles" << endl;
          exit(1);
        }
    }

  double matModelview[16];
  double matProjection[16];
  GLint viewport[4];
  glGetDoublev(GL_MODELVIEW_MATRIX, matModelview);
  glGetDoublev(GL_PROJECTION_MATRIX, matProjection);
  glGetIntegerv(GL_VIEWPORT,viewport);

  ExtractFrustum();

  m_pVertexStatus->Clear();
  m_CountStrips = m_CountFans = 0;
  m_pRootBlock->Tessellate(matModelview,matProjection,viewport,m_pTriangleStrips,m_pTriangleFans,&m_CountStrips,&m_CountFans,this);
  return m_CountStrips * 2 + m_CountFans * 6;
}

bool Terrain::SetCommonTexture(char* szFilename)
{
  bool bSuccess = false;
  if (m_szSourceURL != NULL)
    DownloadFile(szFilename);
  m_szDetailTextureFilename = new char[strlen(szFilename) + 1];
  sprintf(m_szDetailTextureFilename,szFilename);
  char* szFullFilename;
  Settings::GetInstance()->PrependMediaPath(szFilename,&szFullFilename);
  if (!Settings::GetInstance()->IsCompilerOnly())
    {
      if (Settings::GetInstance()->IsVerbose())
        cout << "TERRAIN: Setting common texture to " << szFilename << endl;
      int width,height;
      Uint8* pBuffer;
      LoadImage(szFullFilename,&width,&height,&pBuffer);
      if (width == 0)
        {
          string msg("Failed to load detail texture image file '");
          msg += szFullFilename;
          msg += "'; This means that the file was not found or it is not an image type that Demeter can read.";
          throw new std::invalid_argument(Exception(msg));
        }
      else
        {
          // Test to see if the image is a power of 2 in both width and height.
          if (!IsPowerOf2(width) || !IsPowerOf2(height))
            {
              string msg("The detail texture image file '");
              msg += szFullFilename;
              msg += "' is NOT a power of 2 in both width and height.\nTexture files must be a power of 2 in both width and height.";
              throw new std::invalid_argument(Exception(msg));
            }
          m_pCommonTexture = new Texture(pBuffer,width,height,width,0,false,Settings::GetInstance()->IsTextureCompression());
          delete[] pBuffer;
          bSuccess = true;
          if (Settings::GetInstance()->IsVerbose())
            cout << "TERRAIN: Common texture set with successfully" << endl;
        }
      delete[] szFullFilename;
    }
  else
    {
      FILE* testFile = fopen(szFullFilename,"rb");
      if (testFile)
        {
          bSuccess = true;
          fclose(testFile);
        }
    }
  return bSuccess;
}

bool Terrain::DownloadFile(char* szFilename)
{
  char szFullURLName[1024];
  char* szTempFilename;
  char* szLocalFilename;

  sprintf(szFullURLName,"%s/%s",m_szSourceURL,szFilename);
  Settings::GetInstance()->PrependMediaPath("temp.bin",&szTempFilename);
  Settings::GetInstance()->PrependMediaPath(szFilename,&szLocalFilename);
  bool success = false; //download(szFullURLName,szTempFilename,szLocalFilename);

  delete[] szTempFilename;
  delete[] szLocalFilename;

  return success;
}

bool Terrain::SetTexture(char* szFilename,bool bUseBorders)
{
  if (Settings::GetInstance()->IsHeadless())
    return true;
  bool bSuccess = false;
  if (m_szSourceURL != NULL)
    DownloadFile(m_szTextureFilename);
  m_szTextureFilename = new char[strlen(szFilename) + 1];
  sprintf(m_szTextureFilename,szFilename);
  char* szFullFilename;
  Settings::GetInstance()->PrependMediaPath(szFilename,&szFullFilename);
  if (!Settings::GetInstance()->IsCompilerOnly())
    {
      if (Settings::GetInstance()->IsVerbose())
        cout << "TERRAIN: Setting texture to '" << szFullFilename << "'" << endl;
      int width,height;
      Uint8* pBuffer;
      LoadImage(szFullFilename,&width,&height,&pBuffer);
      if (width == 0)
        {
          string msg("Failed to load texture image file '");
          msg += szFullFilename;
          msg += "'; This means that the file was not found or it is not an image type that Demeter can read.";
          throw new std::invalid_argument(Exception(msg));
        }
      else
        {
          // Test to see if the image is a power of 2 in both width and height.
          if (!IsPowerOf2(width) || !IsPowerOf2(height))
            {
              string msg("The texture image file '");
              msg += szFullFilename;
              msg += "' is NOT a power of 2 in both width and height.\nTexture files must be a power of 2 in both width and height.";
              throw new std::invalid_argument(Exception(msg));
            }
          ChopTexture(pBuffer,width,height,256,bUseBorders);
          delete[] pBuffer;
          bSuccess = true;
          if (Settings::GetInstance()->IsVerbose())
            cout << "TERRAIN: Texture set successfully" << endl;
        }
    }
  else
    {
      FILE* fileTest = fopen(szFullFilename,"rb");
      if (fileTest)
        {
          bSuccess = true;
          fclose(fileTest);
        }
    }
  delete[] szFullFilename;
  return bSuccess;
}

void Terrain::ChopTexture(Uint8* pImage,int width,int height,int tileSize,bool bUseBorders)
{
  // It is assumed that the image is in a 3-byte per pixel, RGB format, with no padding on the pixel rows
  m_TextureTileWidth = (int)(((float)(m_WidthVertices / (width / tileSize))) * m_VertexSpacing);
  m_TextureTileHeight = (int)(((float)(m_HeightVertices / (height / tileSize))) * m_VertexSpacing);
  m_NumberOfTextureTilesWidth = width / tileSize;
  m_NumberOfTextureTilesHeight = height / tileSize;
  m_NumberOfTextureTiles = (width / tileSize) * (height / tileSize);
  m_TileSize = tileSize;

  if (bUseBorders)
    {
      unsigned char* pBorderedImage = new Uint8[(width + 2) * (height + 2) * 3];

      int i;
      // Create a temporary version of the original texture that is surrounded by a 1-pixel border.
      for (i = 0; i < height; i++)
        for (int j = 0; j < width * 3; j++)
          pBorderedImage[(i + 1) * (width + 2) * 3 + j + 3] = pImage[i * width * 3 + j];

      // Create texture tiles by roaming across the bordered image.
      //      int k = 0;
      for (i = 0; i < height; i += tileSize)
        {
          for (int j = 0; j < width; j += tileSize)
            {
              Uint8* pTile = pBorderedImage + i * (width + 2) * 3 + j * 3;
#ifdef _USE_VERTEX_ARRAYS_
              Texture* pTexture = new Texture(pTile,tileSize,tileSize,width + 2,1,false,Settings::GetInstance()->IsTextureCompression());
              m_Textures.push_back(pTexture);
#else
              Texture* pTexture = new Texture(pTile,tileSize,tileSize,width + 2,1,true,Settings::GetInstance()->IsTextureCompression());
              m_Textures.push_back(pTexture);
#endif
            }
        }
      delete[] pBorderedImage;
    }
  else
    {
      // Create texture tiles by roaming across the bordered image.
      //int k = 0;
      for (int i = 0; i < height; i += tileSize)
        {
          for (int j = 0; j < width; j += tileSize)
            {
                Uint8* pTile = pImage + i * width * 3 + j * 3;
#ifdef _USE_VERTEX_ARRAYS_
                Texture* pTexture = new Texture(pTile,tileSize,tileSize,width,0,false,Settings::GetInstance()->IsTextureCompression());
                m_Textures.push_back(pTexture);
#else
                Texture* pTexture = new Texture(pTile,tileSize,tileSize,width,0,true,Settings::GetInstance()->IsTextureCompression());
                m_Textures.push_back(pTexture);
#endif
            }
        }
    }
}

/*
void Terrain::DrawTile(SDL_Surface* pSurface,int index,int width,int height)
{
  int texWidth = m_TileSize;
  int texHeight = m_TileSize;
  Uint8* pBuffer = m_pTiles[index];
  if (0 < texWidth)
    {
      if (SDL_LockSurface(pSurface) < 0)
        cout << "TERRAIN: ERROR: Couldn't lock the display surface" << endl;
      Uint8* pTargetBuffer = (Uint8 *)pSurface->pixels;

      for (int i = 0; i < height - 1; i++)
        {
          for (int j = 0; j < width - 1; j++)
            {
              int texOffset = i * texWidth * 3 + j * 3;
              int surfaceOffset = i * pSurface->pitch + j * pSurface->format->BytesPerPixel;
              int red = pBuffer[texOffset];
              int green = pBuffer[texOffset + 1];
              int blue = pBuffer[texOffset + 2];
              Uint32 pixel = SDL_MapRGB(pSurface->format,red,green,blue);
              Uint32* pTargetPixel = (Uint32*)(pTargetBuffer + surfaceOffset);
              *pTargetPixel = pixel;
            }
        }

      SDL_UnlockSurface(pSurface);
      SDL_UpdateRect(pSurface,0,0,0,0);
      delete[] pBuffer;
    }
}

void Terrain::DrawTexture(SDL_Surface* pSurface,int width,int height)
{
  int texWidth,texHeight;
  Uint8* pBuffer;
  LoadImage(m_szTextureFilename,&texWidth,&texHeight,&pBuffer);
  if (0 < texWidth)
    {
      if (SDL_LockSurface(pSurface) < 0)
        cout << "TERRAIN: ERROR: Couldn't lock the display surface" << endl;
      Uint8* pTargetBuffer = (Uint8 *)pSurface->pixels;

      for (int i = 0; i < height - 1; i++)
        {
          for (int j = 0; j < width - 1; j++)
            {
              int texOffset = i * texWidth * 3 + j * 3;
              int surfaceOffset = i * pSurface->pitch + j * pSurface->format->BytesPerPixel;
              int red = pBuffer[texOffset];
              int green = pBuffer[texOffset + 1];
              int blue = pBuffer[texOffset + 2];
              Uint32 pixel = SDL_MapRGB(pSurface->format,red,green,blue);
              Uint32* pTargetPixel = (Uint32*)(pTargetBuffer + surfaceOffset);
              *pTargetPixel = pixel;
            }
        }

      SDL_UnlockSurface(pSurface);
      SDL_UpdateRect(pSurface,0,0,0,0);
      delete[] pBuffer;
    }
}
*/

void Terrain::SetMaximumVisibleBlockSize(int stride)
{
  m_MaximumVisibleBlockSize = stride;
}

float Terrain::GetCommonTextureRepeats()
{
  return m_CommonTextureRepeats;
}

void Terrain::SetCommonTextureRepeats(float commonTextureRepeats)
{
  m_CommonTextureRepeats = commonTextureRepeats;
}

int Terrain::ModelViewMatrixChanged()
{
  int count = Tessellate();
  m_pRootBlock->RepairCracks(this,m_pTriangleStrips,m_pTriangleFans,&m_CountStrips,&m_CountFans);
  return count;
}

void Terrain::Render()
{
  int i,j;
  bool bUseMultiTexture = (m_bMultiTextureSupported && m_pCommonTexture != NULL);

  glFrontFace(GL_CW);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  if (bUseMultiTexture)
    {
      glActiveTextureARB_ptr(GL_TEXTURE0_ARB);
      glEnable(GL_TEXTURE_2D);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexEnvf(GL_TEXTURE_ENV, GL_COMBINE_RGB_EXT, GL_REPLACE);

      glActiveTextureARB_ptr(GL_TEXTURE1_ARB);
      glEnable(GL_TEXTURE_2D);
      glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexEnvf(GL_TEXTURE_ENV,GL_COMBINE_RGB_EXT,GL_ADD);
      glBindTexture(GL_TEXTURE_2D,m_pCommonTexture->UploadTexture());
    }
  else
    glEnable(GL_TEXTURE_2D);

  for (i = 0; i < m_CountStrips; i++)
    m_pTriangleStrips[i].Setup(this);

  for (i = 0; i < m_CountFans; i++)
    m_pTriangleFans[i].Setup(this);

  if (bUseMultiTexture)
    glActiveTextureARB_ptr(GL_TEXTURE0_ARB);

  glNormal3f(0.0f,0.0f,1.0f);

  // Prevent state thrashing by rendering one texture at a time...
  for (i = 0; i < m_NumberOfTextureTiles; i++)
    {
      bool firstTime = true;
      for (j = 0; j < m_CountStrips; j++)
        {
          if (m_pTriangleStrips[j].textureId == i)
            {
              if (firstTime)
                {
                  glBindTexture(GL_TEXTURE_2D,GetTerrainTile(i));
                  firstTime = false;
                }
              m_pTriangleStrips[j].Render(this);
            }
        }
      for (j = 0; j < m_CountFans; j++)
        {
          if (m_pTriangleFans[j].textureId == i)
            {
              if (firstTime)
                {
                  glBindTexture(GL_TEXTURE_2D,GetTerrainTile(i));
                  firstTime = false;
                }
              m_pTriangleFans[j].Render(this);
            }
        }
      if (Settings::GetInstance()->UseDynamicTextures() && firstTime)
        {
          UnloadTerrainTile(i);
        }
    }
  glFrontFace(GL_CCW);
  // Turn multi-texture back off again so the client application doesn't end up in an unexpected state.
  //	if (bUseMultiTexture)
  //	{
  //		glActiveTextureARB_ptr(GL_TEXTURE1_ARB);
  //		glDisable(GL_TEXTURE_2D);
  //		glActiveTextureARB_ptr(GL_TEXTURE0_ARB);
  //		glDisable(GL_TEXTURE_2D);
  //	}
}

void Terrain::DisableTextures()
{
  if (m_bMultiTextureSupported)
    {
      glActiveTextureARB_ptr(GL_TEXTURE0_ARB);
      glDisable(GL_TEXTURE_2D);
      glActiveTextureARB_ptr(GL_TEXTURE1_ARB);
      glDisable(GL_TEXTURE_2D);
    }
  else
    glDisable(GL_TEXTURE_2D);
}

void Terrain::EnableTextures()
{
  if (m_bMultiTextureSupported)
    {
      glActiveTextureARB_ptr(GL_TEXTURE0_ARB);
      glEnable(GL_TEXTURE_2D);
      glActiveTextureARB_ptr(GL_TEXTURE1_ARB);
      glEnable(GL_TEXTURE_2D);
    }
  else
    glEnable(GL_TEXTURE_2D);
}

TriangleStrip::TriangleStrip()
{
}

TriangleStrip::~TriangleStrip()
{
}

inline void TriangleStrip::Setup(Terrain* pTerrain)
{
  if (m_bEnabled)
    {
      minX = pTerrain->m_pVertices[m_pVertices[0]].x;
      minY = pTerrain->m_pVertices[m_pVertices[0]].y;
      for (int i = 1; i < m_NumberOfVertices; i++)
        {
          if (pTerrain->m_pVertices[m_pVertices[i]].x < minX)
            minX = pTerrain->m_pVertices[m_pVertices[i]].x;
          if (pTerrain->m_pVertices[m_pVertices[i]].y < minY)
            minY = pTerrain->m_pVertices[m_pVertices[i]].y;
        }
      int tileY = (int)minY / pTerrain->GetTextureTileHeight();
      int tileX = (int)minX / pTerrain->GetTextureTileWidth();
      textureId = tileY * pTerrain->GetNumberOfTextureTilesWidth() + tileX;
    }
}

inline void TriangleStrip::Render(Terrain* pTerrain)
{
  if (m_bEnabled)
    {
      // IMPORTANT NOTE: Using glDrawElements() is a modest speed gain, but we lose multi-texturing and the use of different
      // texture coordinates for a single vertex! If there is some way to still have these features while using glDrawElements,
      // then re-enable this code in place of the rest of this method.
#ifdef _USE_VERTEX_ARRAYS_
      glDrawElements(GL_TRIANGLE_STRIP,m_NumberOfVertices,GL_UNSIGNED_INT,m_pVertices);
#else
      float texU,texV;
      glBegin(GL_TRIANGLE_STRIP);
      for (int i = 0; i < m_NumberOfVertices; i++)
        {
          texU = pTerrain->m_pVertices[m_pVertices[i]].x / (float)pTerrain->GetTextureTileWidth();
          texV = pTerrain->m_pVertices[m_pVertices[i]].y / (float)pTerrain->GetTextureTileHeight();
          if ((float)texU - (int)texU < EPSILON)
            texU = 1.0f;
          else
            texU = (float)texU - (int)texU;
          if ((float)texV - (int)texV < EPSILON)
            texV = 1.0f;
          else
            texV = (float)texV - (int)texV;
          if (fabs(1.0f - texU) <= EPSILON && fabs(minX - pTerrain->m_pVertices[m_pVertices[i]].x) <= EPSILON)
            texU = 0.0f;
          if (fabs(1.0f - texV) <= EPSILON && fabs(minY - pTerrain->m_pVertices[m_pVertices[i]].y) <= EPSILON)
            texV = 0.0f;
          if (!pTerrain->m_bMultiTextureSupported)
            glTexCoord2f(texU,texV);
          else
            {
              glMultiTexCoord2fARB_ptr(GL_TEXTURE0_ARB,texU,texV);
              glMultiTexCoord2fARB_ptr(GL_TEXTURE1_ARB,texU * pTerrain->GetCommonTextureRepeats(),texV * pTerrain->GetCommonTextureRepeats());
            }
          glVertex3f(pTerrain->m_pVertices[m_pVertices[i]].x + pTerrain->m_OffsetX,pTerrain->m_pVertices[m_pVertices[i]].y + pTerrain->m_OffsetY,pTerrain->m_pVertices[m_pVertices[i]].z);
        }
      glEnd();
#endif
    }
}

TriangleFan::TriangleFan()
{
}

TriangleFan::~TriangleFan()
{
}

inline void TriangleFan::Setup(Terrain* pTerrain)
{
  minX = pTerrain->m_pVertices[m_pVertices[0]].x;
  minY = pTerrain->m_pVertices[m_pVertices[0]].y;
  for (int i = 1; i < m_NumberOfVertices; i++)
    {
      if (pTerrain->m_pVertices[m_pVertices[i]].x < minX)
        minX = pTerrain->m_pVertices[m_pVertices[i]].x;
      if (pTerrain->m_pVertices[m_pVertices[i]].y < minY)
        minY = pTerrain->m_pVertices[m_pVertices[i]].y;
    }
  int tileY = (int)minY / pTerrain->GetTextureTileHeight();
  int tileX = (int)minX / pTerrain->GetTextureTileWidth();
  textureId = tileY * pTerrain->GetNumberOfTextureTilesWidth() + tileX;
}

inline void TriangleFan::Render(Terrain* pTerrain)
{
  // IMPORTANT NOTE: Using glDrawElements() is a modest speed gain, but we lose multi-texturing and the use of different
  // texture coordinates for a single vertex! If there is some way to still have these features while using glDrawElements,
  // then re-enable this code in place of the rest of this method.
#ifdef _USE_VERTEX_ARRAYS_
  glDrawElements(GL_TRIANGLE_FAN,m_NumberOfVertices,GL_UNSIGNED_INT,m_pVertices);
#else
  float texU,texV;
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < m_NumberOfVertices; i++)
    {
      texU = (pTerrain->m_pVertices[m_pVertices[i]].x) / (float)pTerrain->GetTextureTileWidth();
      texV = (pTerrain->m_pVertices[m_pVertices[i]].y) / (float)pTerrain->GetTextureTileHeight();
      if ((float)texU - (int)texU < EPSILON)
        texU = 1.0f;
      else
        texU = (float)texU - (int)texU;
      if ((float)texV - (int)texV < EPSILON)
        texV = 1.0f;
      else
        texV = (float)texV - (int)texV;
      if (fabs(1.0f - texU) <= EPSILON && fabs(minX - pTerrain->m_pVertices[m_pVertices[i]].x) <= EPSILON)
        texU = 0.0f;
      if (fabs(1.0f - texV) <= EPSILON && fabs(minY - pTerrain->m_pVertices[m_pVertices[i]].y) <= EPSILON)
        texV = 0.0f;
      if (!pTerrain->m_bMultiTextureSupported)
        glTexCoord2f(texU,texV);
      else
        {
          glMultiTexCoord2fARB_ptr(GL_TEXTURE0_ARB,texU,texV);
          glMultiTexCoord2fARB_ptr(GL_TEXTURE1_ARB,texU * pTerrain->GetCommonTextureRepeats(),texV * pTerrain->GetCommonTextureRepeats());
        }
      glVertex3f(pTerrain->m_pVertices[m_pVertices[i]].x + pTerrain->m_OffsetX,pTerrain->m_pVertices[m_pVertices[i]].y + pTerrain->m_OffsetY,pTerrain->m_pVertices[m_pVertices[i]].z);
    }
  glEnd();
#endif
}

inline int Terrain::GetTextureTileWidth()
{
  return m_TextureTileWidth;
}

inline int Terrain::GetTextureTileHeight()
{
  return m_TextureTileHeight;
}

inline int Terrain::GetNumberOfTextureTilesWidth()
{
  return m_NumberOfTextureTilesWidth;
}

inline int Terrain::GetNumberOfTextureTilesHeight()
{
  return m_NumberOfTextureTilesHeight;
}

inline GLuint Terrain::GetTerrainTile(int index)
{
  Texture* pTexture = m_Textures[index];
  GLuint val = pTexture->UploadTexture();
  return val;
}

inline void Terrain::UnloadTerrainTile(int index)
{
  Texture* pTexture = m_Textures[index];
  pTexture->UnloadTexture();
}

inline void Terrain::ExtractFrustum()
{
  float proj[16];
  float modl[16];
  float clip[16];
  float t;
  /* Get the current PROJECTION matrix from OpenGL */
  glGetFloatv(GL_PROJECTION_MATRIX, proj);
  /* Get the current MODELVIEW matrix from OpenGL */
  glGetFloatv(GL_MODELVIEW_MATRIX, modl);
  /* Combine the two matrices (multiply projection by modelview) */
  clip[ 0] = modl[ 0] * proj[ 0] + modl[ 1] * proj[ 4] + modl[ 2] * proj[ 8] + modl[ 3] * proj[12];
  clip[ 1] = modl[ 0] * proj[ 1] + modl[ 1] * proj[ 5] + modl[ 2] * proj[ 9] + modl[ 3] * proj[13];
  clip[ 2] = modl[ 0] * proj[ 2] + modl[ 1] * proj[ 6] + modl[ 2] * proj[10] + modl[ 3] * proj[14];
  clip[ 3] = modl[ 0] * proj[ 3] + modl[ 1] * proj[ 7] + modl[ 2] * proj[11] + modl[ 3] * proj[15];
  clip[ 4] = modl[ 4] * proj[ 0] + modl[ 5] * proj[ 4] + modl[ 6] * proj[ 8] + modl[ 7] * proj[12];
  clip[ 5] = modl[ 4] * proj[ 1] + modl[ 5] * proj[ 5] + modl[ 6] * proj[ 9] + modl[ 7] * proj[13];
  clip[ 6] = modl[ 4] * proj[ 2] + modl[ 5] * proj[ 6] + modl[ 6] * proj[10] + modl[ 7] * proj[14];
  clip[ 7] = modl[ 4] * proj[ 3] + modl[ 5] * proj[ 7] + modl[ 6] * proj[11] + modl[ 7] * proj[15];
  clip[ 8] = modl[ 8] * proj[ 0] + modl[ 9] * proj[ 4] + modl[10] * proj[ 8] + modl[11] * proj[12];
  clip[ 9] = modl[ 8] * proj[ 1] + modl[ 9] * proj[ 5] + modl[10] * proj[ 9] + modl[11] * proj[13];
  clip[10] = modl[ 8] * proj[ 2] + modl[ 9] * proj[ 6] + modl[10] * proj[10] + modl[11] * proj[14];
  clip[11] = modl[ 8] * proj[ 3] + modl[ 9] * proj[ 7] + modl[10] * proj[11] + modl[11] * proj[15];
  clip[12] = modl[12] * proj[ 0] + modl[13] * proj[ 4] + modl[14] * proj[ 8] + modl[15] * proj[12];
  clip[13] = modl[12] * proj[ 1] + modl[13] * proj[ 5] + modl[14] * proj[ 9] + modl[15] * proj[13];
  clip[14] = modl[12] * proj[ 2] + modl[13] * proj[ 6] + modl[14] * proj[10] + modl[15] * proj[14];
  clip[15] = modl[12] * proj[ 3] + modl[13] * proj[ 7] + modl[14] * proj[11] + modl[15] * proj[15];
  /* Extract the numbers for the RIGHT plane */
  m_Frustum[0][0] = clip[ 3] - clip[ 0];
  m_Frustum[0][1] = clip[ 7] - clip[ 4];
  m_Frustum[0][2] = clip[11] - clip[ 8];
  m_Frustum[0][3] = clip[15] - clip[12];
  /* Normalize the result */
  t = sqrt(m_Frustum[0][0] * m_Frustum[0][0] + m_Frustum[0][1] * m_Frustum[0][1] + m_Frustum[0][2] * m_Frustum[0][2]);
  m_Frustum[0][0] /= t;
  m_Frustum[0][1] /= t;
  m_Frustum[0][2] /= t;
  m_Frustum[0][3] /= t;
  /* Extract the numbers for the LEFT plane */
  m_Frustum[1][0] = clip[ 3] + clip[ 0];
  m_Frustum[1][1] = clip[ 7] + clip[ 4];
  m_Frustum[1][2] = clip[11] + clip[ 8];
  m_Frustum[1][3] = clip[15] + clip[12];
  /* Normalize the result */
  t = sqrt(m_Frustum[1][0] * m_Frustum[1][0] + m_Frustum[1][1] * m_Frustum[1][1] + m_Frustum[1][2] * m_Frustum[1][2]);
  m_Frustum[1][0] /= t;
  m_Frustum[1][1] /= t;
  m_Frustum[1][2] /= t;
  m_Frustum[1][3] /= t;
  /* Extract the BOTTOM plane */
  m_Frustum[2][0] = clip[ 3] + clip[ 1];
  m_Frustum[2][1] = clip[ 7] + clip[ 5];
  m_Frustum[2][2] = clip[11] + clip[ 9];
  m_Frustum[2][3] = clip[15] + clip[13];
  /* Normalize the result */
  t = sqrt(m_Frustum[2][0] * m_Frustum[2][0] + m_Frustum[2][1] * m_Frustum[2][1] + m_Frustum[2][2] * m_Frustum[2][2]);
  m_Frustum[2][0] /= t;
  m_Frustum[2][1] /= t;
  m_Frustum[2][2] /= t;
  m_Frustum[2][3] /= t;
  /* Extract the TOP plane */
  m_Frustum[3][0] = clip[ 3] - clip[ 1];
  m_Frustum[3][1] = clip[ 7] - clip[ 5];
  m_Frustum[3][2] = clip[11] - clip[ 9];
  m_Frustum[3][3] = clip[15] - clip[13];
  /* Normalize the result */
  t = sqrt(m_Frustum[3][0] * m_Frustum[3][0] + m_Frustum[3][1] * m_Frustum[3][1] + m_Frustum[3][2] * m_Frustum[3][2]);
  m_Frustum[3][0] /= t;
  m_Frustum[3][1] /= t;
  m_Frustum[3][2] /= t;
  m_Frustum[3][3] /= t;
  /* Extract the FAR plane */
  m_Frustum[4][0] = clip[ 3] - clip[ 2];
  m_Frustum[4][1] = clip[ 7] - clip[ 6];
  m_Frustum[4][2] = clip[11] - clip[10];
  m_Frustum[4][3] = clip[15] - clip[14];
  /* Normalize the result */
  t = sqrt(m_Frustum[4][0] * m_Frustum[4][0] + m_Frustum[4][1] * m_Frustum[4][1] + m_Frustum[4][2] * m_Frustum[4][2]);
  m_Frustum[4][0] /= t;
  m_Frustum[4][1] /= t;
  m_Frustum[4][2] /= t;
  m_Frustum[4][3] /= t;
  /* Extract the NEAR plane */
  m_Frustum[5][0] = clip[ 3] + clip[ 2];
  m_Frustum[5][1] = clip[ 7] + clip[ 6];
  m_Frustum[5][2] = clip[11] + clip[10];
  m_Frustum[5][3] = clip[15] + clip[14];
  /* Normalize the result */
  t = sqrt(m_Frustum[5][0] * m_Frustum[5][0] + m_Frustum[5][1] * m_Frustum[5][1] + m_Frustum[5][2] * m_Frustum[5][2]);
  m_Frustum[5][0] /= t;
  m_Frustum[5][1] /= t;
  m_Frustum[5][2] /= t;
  m_Frustum[5][3] /= t;
}

inline bool Terrain::CubeInFrustum(float x, float y, float z, float size)
{
  for(int p = 0; p < 6; p++)
    {
      if(m_Frustum[p][0] * (x - size) + m_Frustum[p][1] * (y - size) + m_Frustum[p][2] * (z - size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x + size) + m_Frustum[p][1] * (y - size) + m_Frustum[p][2] * (z - size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x - size) + m_Frustum[p][1] * (y + size) + m_Frustum[p][2] * (z - size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x + size) + m_Frustum[p][1] * (y + size) + m_Frustum[p][2] * (z - size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x - size) + m_Frustum[p][1] * (y - size) + m_Frustum[p][2] * (z + size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x + size) + m_Frustum[p][1] * (y - size) + m_Frustum[p][2] * (z + size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x - size) + m_Frustum[p][1] * (y + size) + m_Frustum[p][2] * (z + size) + m_Frustum[p][3] > 0)
        continue;
      if(m_Frustum[p][0] * (x + size) + m_Frustum[p][1] * (y + size) + m_Frustum[p][2] * (z + size) + m_Frustum[p][3] > 0)
        continue;
      return false;
    }
  return true;
}

#ifdef _USE_RAYTRACING_SUPPORT_
float Terrain::IntersectRay(float startX,float startY,float startZ,float dirX,float dirY,float dirZ,float& intersectX,float& intersectY,float& intersectZ)
{
  Ray ray;
  float distance = INFINITY;
  Vector point;
  point.x = point.y = point.z = -1.0f;

  ray.m_Origin.x = startX;
  ray.m_Origin.y = startY;
  ray.m_Origin.z = startZ;
  ray.m_Direction.x = dirX;
  ray.m_Direction.y = dirY;
  ray.m_Direction.z = dirZ;
  ray.m_Direction.Normalize();
  m_pRootBlock->IntersectRay(ray,point,distance,this);
  intersectX = point.x;
  intersectY = point.y;
  intersectZ = point.z;
  return distance;
}
#endif

inline void Terrain::SetLatticePosition(int x,int y)
{
  m_LatticePositionX = x;
  m_LatticePositionY = y;
}

inline void Terrain::GetLatticePosition(int& x,int& y)
{
  x = m_LatticePositionX;
  y = m_LatticePositionY;
}

#ifdef _USE_RAYTRACING_SUPPORT_
void TerrainBlock::IntersectRay(Ray& ray,Vector& intersectionPoint,float& lowestDistance,Terrain* pTerrain)
{
  // First test ray against this block's bounding box.
  if (RayBoxIntersect(&ray,&m_BoundingBox,NULL,NULL))
    {
      if (2 < m_Stride)
        {
          m_pChildren[0]->IntersectRay(ray,intersectionPoint,lowestDistance,pTerrain);
          m_pChildren[1]->IntersectRay(ray,intersectionPoint,lowestDistance,pTerrain);
          m_pChildren[2]->IntersectRay(ray,intersectionPoint,lowestDistance,pTerrain);
          m_pChildren[3]->IntersectRay(ray,intersectionPoint,lowestDistance,pTerrain);
        }
      else
        {
          float distance;
          Vector point;
          for (int i = 0; i < 8; i++)
            {
              if (RayPlaneIntersect(&ray,m_pTriangles[i].GetPlane(),&point,&distance) == 1)
                {
                  if (i == 0 || i == 2 || i == 4 || i == 6)
                    {
                      if (m_pTriangles[i].GetVertex(0)->x <= point.x &&
                          m_pTriangles[i].GetVertex(0)->y <= point.y &&
                          point.x <= m_pTriangles[i].GetVertex(2)->x &&
                          point.y <= m_pTriangles[i].GetVertex(1)->y &&
                          ((fmod(point.y,pTerrain->m_VertexSpacing) + fmod(point.x,pTerrain->m_VertexSpacing)) <= pTerrain->m_VertexSpacing))
                        {
                          if (distance < lowestDistance)
                            {
                              lowestDistance = distance;
                              intersectionPoint.x = point.x;
                              intersectionPoint.y = point.y;
                              intersectionPoint.z = point.z;
                            }
                        }
                    }
                  else
                    {
                      if (m_pTriangles[i].GetVertex(1)->x <= point.x &&
                          m_pTriangles[i].GetVertex(0)->y <= point.y &&
                          point.x <= m_pTriangles[i].GetVertex(2)->x &&
                          point.y <= m_pTriangles[i].GetVertex(1)->y &&
                          ((fmod(point.y,pTerrain->m_VertexSpacing) + fmod(point.x,pTerrain->m_VertexSpacing)) >= pTerrain->m_VertexSpacing))
                        {
                          if (distance < lowestDistance)
                            {
                              lowestDistance = distance;
                              intersectionPoint.x = point.x;
                              intersectionPoint.y = point.y;
                              intersectionPoint.z = point.z;
                            }
                        }
                    }
                }
            }
        }
    }
}

int RayPlaneIntersect(Ray *ray,Plane *plane,Vector* point,float *distance)
{
  float			vd,vo,PnDOTRo,t;

  vd = plane->a * ray->m_Direction.x + plane->b * ray->m_Direction.y + plane->c * ray->m_Direction.z;
  if (vd == 0.0)
    // The plane is parallel to the ray. I've never seen this happen but someday it will . . .
    return -1;
  if (vd > 0.0)
    {
      // The plane is facing away from the ray so no intersection occurs.
      return -2;
    }
  PnDOTRo = plane->a * ray->m_Origin.x + plane->b * ray->m_Origin.y + plane->c * ray->m_Origin.z;
  vo = -1.0f * (PnDOTRo + plane->d);
  t = vo / vd;
  if (t < 0.0f)
    // The intersection occurs behind the ray's origin.
    return -3;
  point->x = ray->m_Origin.x + ray->m_Direction.x * t;
  point->y = ray->m_Origin.y + ray->m_Direction.y * t;
  point->z = ray->m_Origin.z + ray->m_Direction.z * t;
  if (distance != NULL)
    *distance = t;
  return 1;
}

int RayBoxIntersect(Ray *ray,Box *box,Vector *point,float *distance)
{
  float		tnear,tfar,t1,t2;

  tnear = -INFINITY;
  tfar = INFINITY;

  // Find intersection with x-aligned planes of box.
  // If the ray is parallel to the box and not within the planes of the box it misses.
  if (ray->m_Direction.x == 0.0)
    if ((ray->m_Origin.x < box->m_Min.x) && (ray->m_Origin.x > box->m_Max.x))
      return 0;
  // Calculate intersection distance with the box's planes.
  t1 = (box->m_Min.x - ray->m_Origin.x) / ray->m_Direction.x;
  t2 = (box->m_Max.x - ray->m_Origin.x) / ray->m_Direction.x;
  if (t1 > t2)
    {
      float tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
  if (t1 > tnear)
    tnear = t1;
  if (t2 < tfar)
    tfar = t2;
  if (tnear > tfar)
    return 0;
  if (tfar < 0.0)
    return 0;
  // Find intersection with y-aligned planes of box.
  // If the ray is parallel to the box and not within the planes of the box it misses.
  if (ray->m_Direction.y == 0.0)
    if ((ray->m_Origin.y < box->m_Min.y) && (ray->m_Origin.y > box->m_Max.y))
      return 0;
  // Calculate intersection distance with the box's planes.
  t1 = (box->m_Min.y - ray->m_Origin.y) / ray->m_Direction.y;
  t2 = (box->m_Max.y - ray->m_Origin.y) / ray->m_Direction.y;
  if (t1 > t2)
    {
      float tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
  if (t1 > tnear)
    tnear = t1;
  if (t2 < tfar)
    tfar = t2;
  if (tnear > tfar)
    return 0;
  if (tfar < 0.0)
    return 0;
  // Find intersection with z-aligned planes of box.
  // If the ray is parallel to the box and not within the planes of the box it misses.
  if (ray->m_Direction.z == 0.0)
    if ((ray->m_Origin.z < box->m_Min.z) && (ray->m_Origin.z > box->m_Max.z))
      return 0;
  // Calculate intersection distance with the box's planes.
  t1 = (box->m_Min.z - ray->m_Origin.z) / ray->m_Direction.z;
  t2 = (box->m_Max.z - ray->m_Origin.z) / ray->m_Direction.z;
  if (t1 > t2)
    {
      float tmp = t1;
      t1 = t2;
      t2 = tmp;
    }
  if (t1 > tnear)
    tnear = t1;
  if (t2 < tfar)
    tfar = t2;
  if (tnear > tfar)
    return 0;
  if (tfar < 0.0)
    return 0;
  // If we survived all of the tests, the box is hit.
  if (point != NULL)
    {
      point->x = ray->m_Origin.x + tnear * ray->m_Direction.x;
      point->y = ray->m_Origin.y + tnear * ray->m_Direction.y;
      point->z = ray->m_Origin.z + tnear * ray->m_Direction.z;
      *distance = tnear;
    }
  return 1;
}
#endif

void LoadImage(char* szFilename,int* pWidth,int* pHeight,Uint8** pBuffer,bool bColorKey)
{
  osg::ref_ptr<osg::Image> pImage = osgDB::Registry::instance()->readImage(szFilename,osgDB::Registry::CACHE_NONE).getImage();
  if (pImage != NULL)
    {
      *pWidth = pImage->s();
      *pHeight = pImage->t();
      int pitch = 3 * pImage->s(); // why3?? !!!

      Uint8* pBufferTemp;
      if (bColorKey)
        pBufferTemp = new Uint8[*pWidth * *pHeight * 4];
      else
        pBufferTemp = new Uint8[*pWidth * *pHeight * 3];
      int i,j;
      Uint8* pImagePixels = (Uint8*)pImage->data();
      int bytesPerPixel = pImage->getPixelSizeInBits() / 8;
      for (i = pImage->t() * pitch - pitch,j = 0; i >= 0; i -= pitch)
        {
          Uint8* pImageRow = pImagePixels + i;
          for (Uint8* pImagePixel = pImageRow; pImagePixel < pImageRow + pImage->s() * bytesPerPixel; pImagePixel += bytesPerPixel)
            {
              Uint8 red,green,blue,alpha;
              Uint32* pCurrentPixel = (Uint32*)pImagePixel;
              if (bColorKey) {
                red   = (*pCurrentPixel) & 0xff; // !!!! check this is ok for Intel endian
                green = ((*pCurrentPixel) & 0xff00) >> 8; // (we should really use dataType intelligently!)
                blue  = ((*pCurrentPixel) & 0xff0000) >> 16;
                alpha = ((*pCurrentPixel) & 0xff000000) >> 24;
                //		SDL_GetRGBA(*pCurrentPixel,pImage->format,&red,&green,&blue,&alpha);
              }
              else {
                red   = (*pCurrentPixel) & 0xff; // !!!! check this is ok for Intel endian
                green = ((*pCurrentPixel) & 0xff00) >> 8; // (we should really use dataType intelligently!)
                blue  = ((*pCurrentPixel) & 0xff0000) >> 16;
                alpha = ((*pCurrentPixel) & 0xff000000) >> 24;
                //		SDL_GetRGBA(*pCurrentPixel,pImage->format,&red,&green,&blue,&alpha);
              }
              pBufferTemp[j++] = red;
              pBufferTemp[j++] = green;
              pBufferTemp[j++] = blue;
              if (bColorKey)
                {
                  pBufferTemp[j++] = alpha;
                }
            }
        }
      *pBuffer = pBufferTemp;
      //      SDL_FreeSurface(pImage);
    }
  else
    {
      *pWidth = 0;
      *pHeight = 0;
      *pBuffer = NULL;
    }
}

GLuint CreateTexture(Uint8* pTexels,int width,int height,int rowLength,int border,int internalFormat,bool bClamp,bool bColorKey)
{
  GLuint texId;
  glGenTextures(1,&texId);
  glBindTexture(GL_TEXTURE_2D,texId);
  if (bClamp)
    {
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
    }
  else
    {
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
      glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
    }
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
  glPixelStorei(GL_UNPACK_ROW_LENGTH,rowLength);
  glPixelStorei(GL_UNPACK_ALIGNMENT,1);
  if (bColorKey)
    {
      glTexImage2D(GL_TEXTURE_2D,0,internalFormat,width + 2 * border,height + 2 * border,border,GL_RGBA,GL_UNSIGNED_BYTE,pTexels);
      gluBuild2DMipmaps(GL_TEXTURE_2D,GL_RGBA,width,height,GL_RGBA,GL_UNSIGNED_BYTE,pTexels);
    }
  else
    {
      glTexImage2D(GL_TEXTURE_2D,0,internalFormat,width + 2 * border,height + 2 * border,border,GL_RGB,GL_UNSIGNED_BYTE,pTexels);
      gluBuild2DMipmaps(GL_TEXTURE_2D,GL_RGB,width,height,GL_RGB,GL_UNSIGNED_BYTE,pTexels);
    }
  glPixelStorei(GL_UNPACK_ROW_LENGTH,0);
  return texId;
}

inline bool IsPowerOf2(double number)
{
  const int MAX_POWER = 1024;
  bool isPowerOf2 = false;
  for(int i = 0; i < MAX_POWER && !isPowerOf2; i++)
    {
      if (pow(2.0,i) == number)
        isPowerOf2 = true;
    }
  return isPowerOf2;
}

Settings::Settings()
{
  m_szMediaPath = NULL;
  m_bVerbose = false;
  m_bIsCompilerOnly = false;
  m_IsHeadless = false;
  m_UseDynamicTextures = false;
}

Settings::~Settings()
{
  if (m_szMediaPath != NULL)
    delete[] m_szMediaPath;
}

Settings* Settings::GetInstance()
{
  if (pSettingsInstance == NULL)
    pSettingsInstance = new Settings();
  return pSettingsInstance;
}

bool Settings::SetProperty(char* szProperty,char* szValue)
{
  bool bSuccess = false;

  if (strcmp(szProperty,"verbose") == 0)
    {
      SetVerbose(strcmp(szValue,"true") == 0);
      bSuccess = true;
    }

  return bSuccess;
}

bool Settings::GetProperty(char* szProperty,char* szValue)
{
  bool bSuccess = false;

  if (strcmp(szProperty,"verbose") == 0)
    {
      sprintf(szValue,m_bVerbose ? "true" : "false");
      bSuccess = true;
    }
  else if (strcmp(szProperty,"glinfo") == 0)
    {
      sprintf(szValue,"OpenGL Vendor: %s; OpenGL Extensions Supported: %s",glGetString(GL_VENDOR),glGetString(GL_EXTENSIONS));
      bSuccess = true;
    }

  return bSuccess;
}

inline void Settings::SetUseDynamicTextures(bool use)
{
  m_UseDynamicTextures = use;
}

inline bool Settings::UseDynamicTextures()
{
  return m_UseDynamicTextures;
}

void Settings::SetMediaPath(char* szPath)
{
  char separator = '/';

  m_bCompressTextures = false;
  if (m_szMediaPath != NULL)
    delete[] m_szMediaPath;
  if (szPath[strlen(szPath) - 1] == separator)
    {
      m_szMediaPath = new char[strlen(szPath) + 1];
      sprintf(m_szMediaPath,szPath);
    }
  else
    {
      m_szMediaPath = new char[strlen(szPath) + 2];
      sprintf(m_szMediaPath,"%s%c",szPath,separator);
    }
}

void Settings::GetMediaPath(char** szPath)
{
  char* szOutput = new char[strlen(m_szMediaPath) + 1];
  sprintf(szOutput,m_szMediaPath);
  *szPath = szOutput;
}

void Settings::PrependMediaPath(char* szFilename,char** pszFullFilename)
{
  char* szFull = new char[strlen(szFilename) + strlen(m_szMediaPath) + 2];
  sprintf(szFull,"%s%s",m_szMediaPath,szFilename);
  *pszFullFilename = szFull;
}

void Settings::SetHeadless(bool isHeadless)
{
  m_IsHeadless = isHeadless;
}

bool Settings::IsHeadless()
{
  return m_IsHeadless;
}

void Settings::SetVerbose(bool bVerbose)
{
  m_bVerbose = bVerbose;
}

bool Settings::IsVerbose()
{
  return m_bVerbose;
}

inline int Settings::GetScreenWidth()
{
  return m_ScreenWidth;
}

inline void Settings::SetScreenWidth(int width)
{
  m_ScreenWidth = width;
}

inline int Settings::GetScreenHeight()
{
  return m_ScreenHeight;
}

inline void Settings::SetScreenHeight(int height)
{
  m_ScreenHeight = height;
}

inline bool Settings::IsCompilerOnly()
{
  return m_bIsCompilerOnly;
}

void Settings::SetCompilerOnly(bool bIsCompilerOnly)
{
  m_bIsCompilerOnly = bIsCompilerOnly;
}

void Settings::SetTextureCompression(bool bCompress)
{
  m_bCompressTextures = bCompress;
}

inline bool Settings::IsTextureCompression()
{
  return m_bCompressTextures;
}

Vector& Vector::operator = (const Vector& v)
{
  x = v.x;
  y = v.y;
  z = v.z;
  return *this;
}

inline float Vector::Normalize(float tolerance)
{
  float length = GetLength();

  if (length > tolerance)
    {
      float invLength = 1.0f / length;
      x *= invLength;
      y *= invLength;
      z *= invLength;
    }
  else
    {
      length = 0.0;
    }

  return length;
}

inline float Vector::GetLength()
{
  return sqrt(x*x + y*y + z*z);
}

Plane::Plane(Vector& p1,Vector& p2,Vector& p3)
{
  defineFromPoints(p1,p2,p3);
}

inline void Plane::defineFromPoints(Vector& p1,Vector& p2,Vector& p3)
{
  Vector v1,v2,normal;
  // Find the normal of the polygon defined by the three points(cross product of 2 vertex vectors)
  v1.x = p2.x - p1.x;
  v1.y = p2.y - p1.y;
  v1.z = p2.z - p1.z;

  v2.x = p3.x - p1.x;
  v2.y = p3.y - p1.y;
  v2.z = p3.z - p1.z;

  v1.Normalize();
  v2.Normalize();
  // Find surface normal based on cross product.
  normal.x = v1.y * v2.z - v2.y * v1.z;
  normal.y = v2.x * v1.z - v1.x * v2.z;
  normal.z = v1.x * v2.y - v2.x * v1.y;
  normal.Normalize();
  // This surface normal represents the a,b,c components of the plane equation.
  a = normal.x;
  b = normal.y;
  c = normal.z;
  // The d component is calculated from Ax + By + Cz + D = 0
  d = -(a * p1.x + b * p1.y + c * p1.z);
}

Triangle::Triangle()
{
}

Triangle::~Triangle()
{
}

void Triangle::DefineFromPoints(Vector& p1,Vector& p2,Vector& p3)
{
  m_pVertices[0].x = p1.x;
  m_pVertices[0].y = p1.y;
  m_pVertices[0].z = p1.z;
  m_pVertices[1].x = p2.x;
  m_pVertices[1].y = p2.y;
  m_pVertices[1].z = p2.z;
  m_pVertices[2].x = p3.x;
  m_pVertices[2].y = p3.y;
  m_pVertices[2].z = p3.z;
#ifdef _USE_RAYTRACING_SUPPORT_
  m_Plane.defineFromPoints(p3,p2,p1);
#endif
}

inline Vector* Triangle::GetVertex(int index)
{
  return &m_pVertices[index];
}

#ifdef _USE_RAYTRACING_SUPPORT_
Plane* Triangle::GetPlane()
{
  return &m_Plane;
}
#endif


TerrainLattice::TerrainLattice(int widthTerrains,int heightTerrains,float terrainWidth,float terrainHeight)
{
  m_WidthTerrains = widthTerrains;
  m_HeightTerrains = heightTerrains;
  m_TerrainWidth = terrainWidth;
  m_TerrainHeight = terrainHeight;
  m_pTerrains = new Terrain*[widthTerrains * heightTerrains];
  for (int i = 0; i < widthTerrains * heightTerrains; i++)
    m_pTerrains[i] = NULL;
  m_WidthActiveTerrains = m_HeightActiveTerrains = 1;
}

TerrainLattice::~TerrainLattice()
{
  delete[] m_pTerrains;
}

void TerrainLattice::AddTerrain(Terrain* pTerrain,int positionX,int positionY)
{
  m_pTerrains[positionY * m_WidthTerrains + positionX] = pTerrain;
  pTerrain->SetLatticePosition(positionX,positionY);
}

Terrain* TerrainLattice::GetTerrain(int positionX,int positionY)
{
  return m_pTerrains[positionY * m_WidthTerrains + positionX];
}

Terrain* TerrainLattice::GetTerrainRelative(Terrain* pTerrain,Terrain::DIRECTION direction)
{
  int offsetX,offsetY;
  switch (direction)
    {
    case Terrain::DIR_NORTH:
      offsetX = 0;
      offsetY = 1;
      break;
    case Terrain::DIR_NORTHEAST:
      offsetX = 1;
      offsetY = 1;
      break;
    case Terrain::DIR_EAST:
      offsetX = 1;
      offsetY = 0;
      break;
    case Terrain::DIR_SOUTHEAST:
      offsetX = 1;
      offsetY = -1;
      break;
    case Terrain::DIR_SOUTH:
      offsetX = 0;
      offsetY = -1;
      break;
    case Terrain::DIR_SOUTHWEST:
      offsetX = -1;
      offsetY = -1;
      break;
    case Terrain::DIR_WEST:
      offsetX = -1;
      offsetY = 0;
      break;
    case Terrain::DIR_NORTHWEST:
      offsetX = -1;
      offsetY = 1;
      break;
    default:
      offsetX=offsetY=0;
    }
  return GetTerrainRelative(pTerrain,offsetX,offsetY);
}


Terrain* TerrainLattice::GetTerrainRelative(Terrain* pTerrain,int positionX,int positionY)
{
  Terrain* pRequestedTerrain = NULL;
  if (-1 <= positionX && positionX <= 1 && -1 <= positionY && positionY <= 1)
    {
      int x,y;
      pTerrain->GetLatticePosition(x,y);
      x += positionX;
      y += positionY;

      Terrain* pTerrainCenter = m_pTerrains[m_CurrentTerrainIndex[Terrain::DIR_CENTER]];
      int centerX,centerY;
      pTerrainCenter->GetLatticePosition(centerX,centerY);
      if (abs(x - centerX) <= m_WidthActiveTerrains && abs(y - centerY) <= m_HeightActiveTerrains)
        {
          if (0 <= x && x < m_WidthTerrains && 0 <= y && y < m_HeightTerrains)
            pRequestedTerrain = GetTerrain(x,y);
        }
    }
  return pRequestedTerrain;
}

Terrain* TerrainLattice::GetTerrainAtPoint(float x,float y)
{
  int indexX = (int)x / (int)m_TerrainWidth;
  int indexY = (int)y / (int)m_TerrainHeight;
  if (0 <= indexX && indexX < m_WidthTerrains && 0 <= indexY && indexY < m_HeightTerrains)
    return GetTerrain(indexX,indexY);
  else
    return NULL;
}

void TerrainLattice::SetCameraPosition(float x,float y,float z)
{
  int indexX = (int)x / (int)m_TerrainWidth;
  int indexY = (int)y / (int)m_TerrainHeight;
  m_CurrentTerrainIndex[Terrain::DIR_CENTER] = indexY * m_WidthTerrains + indexX;
  m_CurrentTerrainIndex[Terrain::DIR_SOUTH] = 0 < indexY ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] - m_WidthTerrains : -1;
  m_CurrentTerrainIndex[Terrain::DIR_SOUTHEAST] = (0 < indexY && indexX < m_WidthTerrains - 1) ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] - m_WidthTerrains + 1 : -1;
  m_CurrentTerrainIndex[Terrain::DIR_SOUTHWEST] = 0 < indexX && 0 < indexY ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] - m_WidthTerrains - 1 : -1;
  m_CurrentTerrainIndex[Terrain::DIR_EAST] = indexX < m_WidthTerrains - 1 ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] + 1 : -1;
  m_CurrentTerrainIndex[Terrain::DIR_WEST] = 0 < indexX ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] - 1 : -1;
  m_CurrentTerrainIndex[Terrain::DIR_NORTH] = indexY < m_HeightTerrains - 1 ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] + m_WidthTerrains : -1;
  m_CurrentTerrainIndex[Terrain::DIR_NORTHEAST] = (indexX < m_WidthTerrains - 1 && indexY < m_HeightTerrains - 1) ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] + m_WidthTerrains + 1 : -1;
  m_CurrentTerrainIndex[Terrain::DIR_NORTHWEST] = (indexY < m_HeightTerrains - 1 && 0 < indexX) ? m_CurrentTerrainIndex[Terrain::DIR_CENTER] + m_WidthTerrains - 1 : -1;

  for (int i = 0; i < m_WidthTerrains * m_HeightTerrains; i++)
    {
      bool active = false;
      for (int dir = 0; dir < 9 && !active; dir++)
        active = (i == m_CurrentTerrainIndex[dir]);
      if (!active && m_pTerrains[i] != NULL)
        {
          vector<TerrainLoadListener*>::iterator iter = m_TerrainLoadListeners.begin();
          while (iter != m_TerrainLoadListeners.end())
            {
              TerrainLoadListener* pListener = *iter;
              pListener->TerrainUnloading(m_pTerrains[i]);
              iter++;
            }
          delete m_pTerrains[i];
          m_pTerrains[i] = NULL;
        }
      else if (active && m_pTerrains[i] == NULL)
        {
          LoadTerrain(i);
          vector<TerrainLoadListener*>::iterator iter = m_TerrainLoadListeners.begin();
          while (iter != m_TerrainLoadListeners.end())
            {
              TerrainLoadListener* pListener = *iter;
              pListener->TerrainLoaded(m_pTerrains[i]);
              iter++;
            }
        }
    }
}

void TerrainLattice::LoadTerrain(int index)
{
  int indexX = index % m_WidthTerrains;
  int indexY = index / m_WidthTerrains;
  char szMapName[1024];
  sprintf(szMapName,"%d-%d-%s",indexX,indexY,m_BaseName.c_str());

  Terrain* pTerrain = new Terrain(szMapName,m_MaxNumTriangles,m_bUseBorders,(float)indexX * m_TerrainWidth,(float)indexY * m_TerrainHeight);
  if (m_TerrainWidth == 0.0f)
    {
      m_TerrainWidth = pTerrain->GetWidth();
      m_TerrainHeight = pTerrain->GetHeight();
    }
  pTerrain->SetMaximumVisibleBlockSize(m_MaxBlockSize);
  pTerrain->SetCommonTextureRepeats(m_CommonRepeats);
  AddTerrain(pTerrain,indexX,indexY);
}

void TerrainLattice::SetDetailThreshold(float threshold)
{
  for (int i = 0; i < 9; i++)
    {
      if (0 <= m_CurrentTerrainIndex[i])
        m_pTerrains[m_CurrentTerrainIndex[i]]->SetDetailThreshold(threshold);
    }
}

inline float TerrainLattice::GetElevation(float x,float y)
{
  Terrain* pTerrain = GetTerrainAtPoint(x,y);
  if (pTerrain != NULL)
    return pTerrain->GetElevation(x,y);
  else
    return 0.0f;
}

inline Terrain::DIRECTION TerrainLattice::GetOppositeDirection(Terrain::DIRECTION direction)
{
  Terrain::DIRECTION oppositeDirection;
  switch (direction)
    {
    case Terrain::DIR_NORTH:
      oppositeDirection = Terrain::DIR_SOUTH;
      break;
    case Terrain::DIR_NORTHEAST:
      oppositeDirection = Terrain::DIR_SOUTHWEST;
      break;
    case Terrain::DIR_EAST:
      oppositeDirection = Terrain::DIR_WEST;
      break;
    case Terrain::DIR_SOUTHEAST:
      oppositeDirection = Terrain::DIR_NORTHWEST;
      break;
    case Terrain::DIR_SOUTH:
      oppositeDirection = Terrain::DIR_NORTH;
      break;
    case Terrain::DIR_SOUTHWEST:
      oppositeDirection = Terrain::DIR_NORTHEAST;
      break;
    case Terrain::DIR_WEST:
      oppositeDirection = Terrain::DIR_EAST;
      break;
    case Terrain::DIR_NORTHWEST:
      oppositeDirection = Terrain::DIR_SOUTHEAST;
      break;
    case Terrain::DIR_CENTER:
      oppositeDirection = Terrain::DIR_CENTER;
      break;
    default:
      oppositeDirection = Terrain::DIR_CENTER;
    }
  return oppositeDirection;
}

void TerrainLattice::Tessellate()
{
  int i;
  for (i = 0; i < 9; i++)
    {
      if (m_CurrentTerrainIndex[i] != -1)
        m_pTerrains[m_CurrentTerrainIndex[i]]->Tessellate();
    }

  for (i = 0; i < 9; i++)
    {
      if (m_CurrentTerrainIndex[i] != -1)
        {
          for (int direction = 0; direction < 8; direction++)
            {
              if (direction != Terrain::DIR_CENTER)
                {
                  Terrain* pTerrain = GetTerrainRelative(m_pTerrains[m_CurrentTerrainIndex[i]],(Terrain::DIRECTION)direction);
                  if (pTerrain != NULL)
                    {
                      m_pTerrains[m_CurrentTerrainIndex[i]]->UpdateNeighbor(pTerrain,(Terrain::DIRECTION)direction);
                      pTerrain->UpdateNeighbor(m_pTerrains[m_CurrentTerrainIndex[i]],GetOppositeDirection((Terrain::DIRECTION)direction));
                    }
                }
            }
        }
    }

  for (i = 0; i < 9; i++)
    {
      if (m_CurrentTerrainIndex[i] != -1)
        m_pTerrains[m_CurrentTerrainIndex[i]]->m_pRootBlock->RepairCracks(m_pTerrains[m_CurrentTerrainIndex[i]],m_pTerrains[m_CurrentTerrainIndex[i]]->m_pTriangleStrips,m_pTerrains[m_CurrentTerrainIndex[i]]->m_pTriangleFans,&m_pTerrains[m_CurrentTerrainIndex[i]]->m_CountStrips,&m_pTerrains[m_CurrentTerrainIndex[i]]->m_CountFans);
    }

}

void TerrainLattice::Render()
{
  for (int i = 0; i < 9; i++)
    {
      if (m_CurrentTerrainIndex[i] != -1)
        m_pTerrains[m_CurrentTerrainIndex[i]]->Render();
    }
}

void TerrainLattice::Load(char* szBaseName,int maxNumTriangles,int maxBlockSize,float commonRepeats,bool bUseBorders)
{
  int i,j;
  bool horizontalExists,verticalExists,anyExist;

  i = j = 0;
  m_TerrainWidth = m_TerrainHeight = 0.0f;
  m_WidthTerrains = m_HeightTerrains = 0;
  verticalExists = true;
  anyExist = false;

  m_BaseName = szBaseName;
  m_MaxNumTriangles = maxNumTriangles;
  m_MaxBlockSize = maxBlockSize;
  m_CommonRepeats = commonRepeats;
  m_bUseBorders = bUseBorders;

  while(verticalExists)
    {
      verticalExists = false;
      horizontalExists = true;
      while (horizontalExists)
        {
          char szMapName[1024];
          sprintf(szMapName,"%d-%d-%s",i,j,szBaseName);
          char* szFullFilename;
          Settings::GetInstance()->PrependMediaPath(szMapName,&szFullFilename);
          FILE* file = fopen(szFullFilename,"rb");
          horizontalExists = (file != NULL);
          if (horizontalExists)
            {
              fclose(file);
              verticalExists = true;
              anyExist = true;
              if (i == 0 && j == 0)
                {
                  Terrain* pTerrain = new Terrain(szMapName,maxNumTriangles,bUseBorders,(float)i * m_TerrainWidth,(float)j * m_TerrainHeight);
                  m_TerrainWidth = pTerrain->GetWidth();
                  m_TerrainHeight = pTerrain->GetHeight();
                  delete pTerrain;
                }
              i++;
              if (j == 0)
                m_WidthTerrains++;
            }
          else
            i = 0;
        }
      if (verticalExists)
        {
          m_HeightTerrains++;
          j++;
        }
    }
  if (!anyExist)
    {
      char* szMediaPath;
      Settings::GetInstance()->GetMediaPath(&szMediaPath);
      string msg = "Compiled map files for ";
      msg += szBaseName;
      msg += " could not be found at ";
      msg += szMediaPath;
      delete[] szMediaPath;
      throw new std::invalid_argument(Exception(msg));
    }
}

void TerrainLattice::AddTerrainLoadListener(TerrainLoadListener& listener)
{
  m_TerrainLoadListeners.push_back(&listener);
}

void TerrainLattice::RemoveTerrainLoadListener(TerrainLoadListener& listener)
{
  bool found = false;
  vector<TerrainLoadListener*>::iterator iter = m_TerrainLoadListeners.begin();
  while (iter != m_TerrainLoadListeners.end() && !found)
    {
      TerrainLoadListener* pListener = *iter;
      if (pListener == &listener)
        {
          m_TerrainLoadListeners.erase(iter);
          found = true;
        }
      else
        iter++;
    }
}

inline float TerrainLattice::GetWidth()
{
  return m_WidthTerrains * m_TerrainWidth;
}

inline float TerrainLattice::GetHeight()
{
  return m_HeightTerrains * m_TerrainHeight;
}

Texture::Texture(Uint8* pBuffer,int width,int height,int rowLength,int borderSize,bool bClamp,bool useCompression)
{
  m_pBuffer = new Uint8[height * width * 3];
  int k = 0;
  for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width * 3; j++,k++)
        {
          m_pBuffer[k] = pBuffer[i * rowLength * 3 + j];
        }
    }
  m_Width = width;
  m_Height = height;
  m_UseCompression = useCompression;
  m_BorderSize = borderSize;
  m_TextureID = 0;
  m_bClamp = bClamp;
}

Texture::~Texture()
{
  if (m_TextureID != 0)
    {
      glDeleteTextures(1,&m_TextureID);
      m_TextureID = 0;
    }
  if (m_pBuffer != NULL)
    {
      delete[] m_pBuffer;
      m_pBuffer = NULL;
    }
}

GLuint Texture::UploadTexture()
{
  if (m_TextureID == 0)
    {
      int textureFormat;
      textureFormat = m_UseCompression ? COMPRESSED_RGB_S3TC_DXT1_EXT : GL_RGB4;
      m_TextureID = CreateTexture(m_pBuffer,m_Width,m_Height,m_Width,0,textureFormat,m_bClamp);
    }
  return m_TextureID;
}

void Texture::UnloadTexture()
{
  if (m_TextureID != 0)
    {
      glDeleteTextures(1,&m_TextureID);
      m_TextureID = 0;
    }
}
