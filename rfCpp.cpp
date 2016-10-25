// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "rfCpp.h"

#include <float.h>
#include <math.h>
#include <RF/rfmath.h>

#include <RF/rfdefs.h>
#include <RF/rfgraph.h>

#include "rfStruct.h"

#define T_EPSILON 0.f//1e-6

//#define DEBUG_OUTPUT

// Macros from path.h /////////////////////////////////////////////////////////

#define RF_ADDRBITS 32

#if RF_ADDRBITS == 16
 #define RF_SECTOR(x) ((const rfSector16 * RF_RESTRICT)x)
 #define RF_NODE(x) ((const rfNode16 * RF_RESTRICT)x)
 #define RF_LINKSHIFT (RF_GRAPH_LINK16_SHIFT)
 #define RF_TRILIST RF_SECTOR16_TRILIST
 #define RF_TRILIST_TYPE int16_t
#elif RF_ADDRBITS == 32
 #define RF_SECTOR(x) ((const rfSector32 * RF_RESTRICT)x)
 #define RF_NODE(x) ((const rfNode32 * RF_RESTRICT)x)
 #define RF_LINKSHIFT (RF_GRAPH_LINK32_SHIFT)
 #define RF_TRILIST RF_SECTOR32_TRILIST
 #define RF_TRILIST_TYPE int32_t
#else
 #define RF_SECTOR(x) ((const rfSector64 * RF_RESTRICT)x)
 #define RF_NODE(x) ((const rfNode64 * RF_RESTRICT)x)
 #define RF_LINKSHIFT (RF_GRAPH_LINK64_SHIFT)
 #define RF_TRILIST RF_SECTOR64_TRILIST
 #define RF_TRILIST_TYPE int64_t
#endif

#define RF_ELEM_PREPARE_AXIS(axis) \
  edgeindex[axis] = ( axis << RF_EDGE_AXIS_SHIFT ) | RF_EDGE_MIN; \
  vectinv[axis] = -RFF_MAX; \
  if( rffabs( ray.dir[axis] ) > (rff)0.0 ) \
  { \
    vectinv[axis] = (rff)1.0 / ray.dir[axis]; \
    if( ray.dir[axis] >= (rff)0.0 ) \
      edgeindex[axis] = ( axis << RF_EDGE_AXIS_SHIFT ) | RF_EDGE_MAX; \
  }

///////////////////////////////////////////////////////////////////////////////

#include "rfgraph/resolve.h"

void cppTraceRay(void *graph, RTCRay &ray)
{
  rff hitdist = 0.f;
  rfTri *trihit = nullptr;

  void *root = resolve(graph, ray.org);

  rff src[3] = {ray.org[0] + ray.tnear * ray.dir[0],
                ray.org[1] + ray.tnear * ray.dir[1],
                ray.org[2] + ray.tnear * ray.dir[2]};

  rff RF_ALIGN16 vectinv[4];
  int edgeindex[3];
  RF_ELEM_PREPARE_AXIS(0);
  RF_ELEM_PREPARE_AXIS(1);
  RF_ELEM_PREPARE_AXIS(2);

  for( ; ; )
  {
#ifdef DEBUG_OUTPUT
    fprintf(stderr, "traversing sector\n");
#endif
    /* Sector traversal */

    // [info]
    // Ray box intersection to determine endpoint of the line segment to
    // intersect with triangles. (faster than raw ray intersection, we think)
    int nredge = edgeindex[0];
    rff mindist = (RF_SECTOR(root)->edge[edgeindex[0]] - src[0]) * vectinv[0];
    rff dist = (RF_SECTOR(root)->edge[edgeindex[1]] - src[1]) * vectinv[1];
    if( dist < mindist )
    {
      nredge = edgeindex[1];
      mindist = dist;
    }
    dist = (RF_SECTOR(root)->edge[edgeindex[2]] - src[2]) * vectinv[2];
    if( dist < mindist )
    {
      nredge = edgeindex[2];
      mindist = dist;
    }
    // [/info]

    int tricount = RF_SECTOR_GET_PRIMCOUNT(RF_SECTOR(root));
    if( tricount )
    {
      RF_TRILIST_TYPE *trilist;// addressing type

      // [info]
      // Calculating the endpoint using the entry (resolved origin)
      // and ray info
      rff dst[3] = {src[0] + (mindist * ray.dir[0]),
                    src[1] + (mindist * ray.dir[1]),
                    src[2] + (mindist * ray.dir[2])};
      // [/info]

      // [info]
      // Triangle intersection!
      trilist = RF_TRILIST( RF_SECTOR(root) );
      do
      {
        rff dstdist, srcdist, uv[2], f, vray[3];
        rfTri *tri;

        tri = (rfTri *)RF_ADDRESS(root, (rfssize)(*trilist++) << RF_LINKSHIFT);

        // Intersect the triangle, with no back/front face culling
        dstdist = rfMathPlanePoint( tri->plane, dst );
        srcdist = rfMathPlanePoint( tri->plane, src );
        if( dstdist * srcdist > (rff)0.0 )
          continue;
        f = srcdist / ( srcdist - dstdist );
        vray[0] = src[0] + f * ( dst[0] - src[0] );
        vray[1] = src[1] + f * ( dst[1] - src[1] );
        vray[2] = src[2] + f * ( dst[2] - src[2] );
        uv[0] = rfMathVectorDotProduct( &tri->edpu[0], vray ) + tri->edpu[3];
        if( !( uv[0] >= (rff)0.0 ) || ( uv[0] > (rff)1.0 ) )
          continue;
        uv[1] = rfMathVectorDotProduct( &tri->edpv[0], vray ) + tri->edpv[3];
        if( ( uv[1] < (rff)0.0 ) || ( ( uv[0] + uv[1] ) > (rff)1.0 ) )
          continue;
        dst[0] = vray[0];
        dst[1] = vray[1];
        dst[2] = vray[2];

        trihit = tri;
      } while( --tricount );

      int axisindex = nredge >> 1;
      hitdist = (dst[axisindex] - ray.org[axisindex]) * vectinv[axisindex];

#ifdef DEBUG_OUTPUT
      fprintf(stderr, "hit triangle: %p\n", trihit);
#endif

      if(trihit && hitdist > T_EPSILON)
        break;
    }
#ifdef DEBUG_OUTPUT
    else
        fprintf(stderr, "no triangles...\n");
#endif
    // [/info]

    // [info]
    // If the neiboring node is a sector, just go straight to it and move on
    /* Traverse through the sector's edge */
    if(RF_SECTOR(root)->flags &
       ((RF_LINK_SECTOR<<RF_SECTOR_LINKFLAGS_SHIFT) << nredge))
    {
#ifdef DEBUG_OUTPUT
      fprintf(stderr, "neighbor is a sector\n");
#endif
      /* Neighbor is sector */
      rfssize slink = (rfssize)RF_SECTOR(root)->link[nredge];
      if(!(slink))
      {
#ifdef DEBUG_OUTPUT
        fprintf(stderr, "goto tracevoid\n");
#endif
        break;//CHECKME:-->may be incorrect? was 'goto tracevoid;'
      }
      root = RF_ADDRESS(root, slink << RF_LINKSHIFT);
      continue;
    }
    // [/info]

    // [info]
    // We have to do some kind of disambiguation of the neighbors to figure
    // out what sector we need to traverse to next.
    /* Neighbor is node */
    src[0] += mindist * ray.dir[0];
    src[1] += mindist * ray.dir[1];
    src[2] += mindist * ray.dir[2];
    root = RF_ADDRESS(root,
                      (rfssize)RF_SECTOR(root)->link[nredge] << RF_LINKSHIFT);
    // [/info]

    /* Node traversal */
    for( ; ; )
    {
#ifdef DEBUG_OUTPUT
      fprintf(stderr, "node traversal\n");
#endif

      int linkflags;
      linkflags = RF_NODE(root)->flags;//--> pull out the current traversal
                                       //    node
                                       //...links to other nodes or a sector
      // [info]
      // Figure out which half-space we are traversing through the
      // diambiguation plane.
      if( src[RF_NODE_GET_AXIS(linkflags)] < RF_NODE(root)->plane )
      {
        root = RF_ADDRESS(root,
                          (rfssize)RF_NODE(root)
                          ->link[RF_NODE_LESS] << RF_LINKSHIFT);
        if( linkflags & ((RF_LINK_SECTOR<<RF_NODE_LINKFLAGS_SHIFT)
                         << RF_NODE_LESS))
          break;
      }
      else
      {
        root = RF_ADDRESS(root,
                          (rfssize)RF_NODE(root)->link[RF_NODE_MORE]
                          << RF_LINKSHIFT);
        if( linkflags & ((RF_LINK_SECTOR<<RF_NODE_LINKFLAGS_SHIFT)
                         << RF_NODE_MORE))
          break;
      }
      // [/info]
    }
  }

  // We have a triangle intersection, go ahead and populate the ray in the
  // RayPacket accordingly
  if(trihit)
  {
    rfTriangleData* data =
            (rfTriangleData*)(RF_ADDRESS(trihit, sizeof(rfTri)));

    ray.tfar = hitdist - T_EPSILON;
    ray.Ng[0] = trihit->plane[0];
    ray.Ng[1] = trihit->plane[1];
    ray.Ng[2] = trihit->plane[2];

    ray.geomID = data->geomID;
    ray.instID = data->geomID;
    ray.primID = data->primID;
  }
}
