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

// ospray
#include "common/Ray.ih"
#include "common/Model.ih"
#include "geometry/Geometry.ih"
#include "math/LinearSpace.ih"

#include "common/safe_gather.ih"

struct RayforceGraph
{
  uniform Geometry  geometry; //!< inherited geometry fields
  uniform int32     numTriangles; //!< number of triangles
  uniform int32     idxSize; //!< stride of triangle indices, in int32 elements
  uniform int32     vtxSize; //!< stride of vertex positions, in float32 elements
  uniform int32     norSize; //!< stride of normals, in float32 elements
  uniform int      *index;  //!< mesh's triangle index array
  uniform float    *vertex; //!< mesh's vertex position array
  uniform float    *normal; //!< mesh's vertex normal array
  uniform vec4f    *color;  //!< mesh's vertex color array
  uniform vec2f    *texcoord; //!< mesh's texture coordinate array
  uniform uint32   *prim_materialID;     // per-primitive material ID
  uniform Material *uniform *materialList;  // list of materials, if multiple materials are assigned to this mesh.
  uniform int32     geom_materialID;     // per-object material ID
};

static void RayforceGraph_postIntersect(uniform Geometry *uniform _self,
                                        uniform Model *uniform model,
                                        varying DifferentialGeometry &dg,
                                        const varying Ray &ray,
                                        uniform int64 flags)
{
  uniform RayforceGraph *uniform self = (uniform RayforceGraph *uniform)_self;
  dg.Ng = dg.Ns = ray.Ng;

#if 0
  const varying int indexBase = self->idxSize * ray.primID;
  const varying vec3i index = make_vec3i(self->index[indexBase+0],
                                         self->index[indexBase+1],
                                         self->index[indexBase+2]);
  const uniform float *uniform normal = self->normal;
  const uniform int32 norSize = self->norSize;
  if ((flags & DG_NS) && normal) {
    const uniform vec3f *const varying n0ptr = (const uniform vec3f *)&normal[index.x * norSize];
    const uniform vec3f *const varying n1ptr = (const uniform vec3f *)&normal[index.y * norSize];
    const uniform vec3f *const varying n2ptr = (const uniform vec3f *)&normal[index.z * norSize];
    dg.Ns
      = (1.f-ray.u-ray.v) * *n0ptr
      + ray.u * *n1ptr
      + ray.v * *n2ptr;
  }

  if ((flags & DG_COLOR)) {
    uniform vec4f *uniform color = self->color;
    if (color) {
      dg.color
        = (1.f-ray.u-ray.v) * (color[index.x])
        + ray.u * (color[index.y])
        + ray.v * (color[index.z]);
    }
  }

  if (flags & DG_TEXCOORD && self->texcoord) {
    //calculate texture coordinate using barycentric coordinates
    const uniform vec2f *uniform texcoord = self->texcoord;
    dg.st
      = (1.f-ray.u-ray.v) * (texcoord[index.x])
      + ray.u * (texcoord[index.y])
      + ray.v * (texcoord[index.z]);
  } else {
    dg.st = make_vec2f(0.0f, 0.0f);
  }

  if (flags & DG_TANGENTS) {
    uniform bool fallback = true;
    if (self->texcoord) {
      const uniform vec2f *uniform texcoord = self->texcoord;
      const vec2f dst02 = texcoord[index.x] - texcoord[index.z];
      const vec2f dst12 = texcoord[index.y] - texcoord[index.z];
      const float det = dst02.x * dst12.y - dst02.y * dst12.x;

      if (det != 0.f) {
        const float invDet = rcp(det);
        const uniform float *uniform vertex = self->vertex;
        const uniform int32 vtxSize = self->vtxSize;
        const vec3f *const varying v0ptr = (const uniform vec3f *)&vertex[index.x * vtxSize];
        const vec3f *const varying v1ptr = (const uniform vec3f *)&vertex[index.y * vtxSize];
        const vec3f *const varying v2ptr = (const uniform vec3f *)&vertex[index.z * vtxSize];
        const vec3f dp02 = *v0ptr - *v2ptr;
        const vec3f dp12 = *v1ptr - *v2ptr;
        dg.dPds = (dst12.y * dp02 - dst02.y * dp12) * invDet;
        dg.dPdt = (dst02.x * dp12 - dst12.x * dp02) * invDet;
        fallback = false;
      }
    }
    if (fallback) {
      linear3f f = frame(dg.Ng);
      dg.dPds = f.vx;
      dg.dPdt = f.vy;
    }
  }

  if (flags & DG_MATERIALID) {
    if (self->prim_materialID) {
      dg.materialID = self->prim_materialID[ray.primID];
    }
    // TODO: gather material ID from index array if indexSize==4
    else {
      dg.materialID = self->geom_materialID;
    }
  }
#else
  const uniform bool huge_mesh = false;
  const vec3i index = gather_vec3i(huge_mesh, self->index, self->idxSize, ray.primID);
  vec3f bary = make_vec3f(1.0f - ray.u - ray.v, ray.u, ray.v);

  if (flags & DG_NS && self->normal) {
    const uniform float *uniform normal = self->normal;
    const uniform int32 norSize = self->norSize;
    const vec3f a = gather_vec3f(huge_mesh, normal, norSize, index.x);
    const vec3f b = gather_vec3f(huge_mesh, normal, norSize, index.y);
    const vec3f c = gather_vec3f(huge_mesh, normal, norSize, index.z);
    dg.Ns = interpolate(bary, a, b, c);
  }

  if (flags & DG_COLOR && self->color) {
    const uniform vec4f *uniform color = self->color;
    const vec4f a = gather_vec4f(huge_mesh, color, index.x);
    const vec4f b = gather_vec4f(huge_mesh, color, index.y);
    const vec4f c = gather_vec4f(huge_mesh, color, index.z);
    dg.color = interpolate(bary, a, b, c);
#if 0
    if (!self->has_alpha)
      dg.color.w = 1.f;
#endif
  }

  if (flags & DG_TEXCOORD && self->texcoord) {
    const uniform vec2f *uniform texcoord = self->texcoord;
    const vec2f a = gather_vec2f(huge_mesh, texcoord, index.x);
    const vec2f b = gather_vec2f(huge_mesh, texcoord, index.y);
    const vec2f c = gather_vec2f(huge_mesh, texcoord, index.z);
    dg.st = interpolate(bary, a, b, c);
  } else
    dg.st = make_vec2f(0.0f, 0.0f);

  if (flags & DG_TANGENTS) {
    uniform bool fallback = true;
    if (self->texcoord) {
      const uniform vec2f *uniform texcoord = self->texcoord;
      const vec2f a = gather_vec2f(huge_mesh, texcoord, index.x);
      const vec2f b = gather_vec2f(huge_mesh, texcoord, index.y);
      const vec2f c = gather_vec2f(huge_mesh, texcoord, index.z);
      const vec2f dst02 = a - c;
      const vec2f dst12 = b - c;
      const float det = dst02.x * dst12.y - dst02.y * dst12.x;

      if (det != 0.f) {
        const float invDet = rcp(det);
        const uniform float *uniform vertex = self->vertex;
        const uniform int32 vtxSize = self->vtxSize;
        const vec3f a = gather_vec3f(huge_mesh, vertex, vtxSize, index.x);
        const vec3f b = gather_vec3f(huge_mesh, vertex, vtxSize, index.y);
        const vec3f c = gather_vec3f(huge_mesh, vertex, vtxSize, index.z);
        const vec3f dp02 = a - c;
        const vec3f dp12 = b - c;
        dg.dPds = (dst12.y * dp02 - dst02.y * dp12) * invDet;
        dg.dPdt = (dst02.x * dp12 - dst12.x * dp02) * invDet;
        fallback = false;
      }
    }
    if (fallback) {
      linear3f f = frame(dg.Ng);
      dg.dPds = f.vx;
      dg.dPdt = f.vy;
    }
  }

  if (flags & DG_MATERIALID) {
    if (self->prim_materialID) {
      dg.materialID = self->prim_materialID[ray.primID];
    }
    // TODO: gather material ID from index array if indexSize==4
    else {
      dg.materialID = self->geom_materialID;
    }
  }
#endif
}

SampleAreaRes RayforceGraph_sampleArea(
    const Geometry *uniform const _self
    , const int32 primID
    , const uniform affine3f &xfm
    , const uniform affine3f &
    , const vec2f& s
    )
{
  const RayforceGraph *const uniform self = (const RayforceGraph *uniform)_self;
  SampleAreaRes res;

  // gather vertices
  const uniform bool huge_mesh = false;//self->huge_mesh;
  const vec3i index = gather_vec3i(huge_mesh, self->index, self->idxSize, primID);
  const uniform float *uniform vertex = self->vertex;
  const uniform int32 vtxSize = self->vtxSize;
  const vec3f a = gather_vec3f(huge_mesh, vertex, vtxSize, index.x);
  const vec3f b = gather_vec3f(huge_mesh, vertex, vtxSize, index.y);
  const vec3f c = gather_vec3f(huge_mesh, vertex, vtxSize, index.z);

  const vec3f localPos = uniformSampleTriangle(a, b, c, s);
  res.pos = xfmPoint(xfm, localPos);

  const vec3f e1 = a-c;
  const vec3f e2 = b-c;
  res.normal = normalize(cross(xfmVector(xfm, e1), xfmVector(xfm, e2)));

  return res;
}

int32 RayforceGraph_getMaterialID(
    const Geometry *uniform const _self
    , const int32 primID
    )
{
  const RayforceGraph *const uniform self = (const RayforceGraph *uniform)_self;
  return self->prim_materialID ? self->prim_materialID[primID] : self->geom_materialID;
}

void RayforceGraph_getAreas(
    const Geometry *const uniform _self
    , const int32 *const uniform primIDs
    , const uniform int32 numPrims
    , const uniform affine3f &xfm
    , float *const uniform areas
    )
{
  const RayforceGraph *const uniform self = (const RayforceGraph *uniform)_self;

  const uniform int32 idxSize = self->idxSize;
  const float *const uniform vertex = self->vertex;
  const uniform int64 vtxSize = self->vtxSize;
  // TODO vectorize this loop, with foreach or ProgramCount & ProgramIndex
  for (uniform int32 i = 0; i < numPrims; i++) {
    int *uniform index = self->index + primIDs[i] * idxSize;
    const uniform vec3f a = *((vec3f*)(vertex + vtxSize * *index));
    const uniform vec3f b = *((vec3f*)(vertex + vtxSize * *(index+1)));
    const uniform vec3f c = *((vec3f*)(vertex + vtxSize * *(index+2)));
    const uniform vec3f e1 = xfmVector(xfm, a-c);
    const uniform vec3f e2 = xfmVector(xfm, b-c);
    areas[i] = 0.5f * length(cross(e1, e2));
  }
}


void RayforceGraph_Constructor(uniform RayforceGraph *uniform mesh,
                               void *uniform cppEquivalent,
                               uniform Model *uniform model,
                               uniform int32  geomID,
                               uniform int32  numTriangles,
                               uniform int32  idxSize,
                               uniform int32  vtxSize,
                               uniform int32  norSize,
                               uniform int    *uniform index,
                               uniform float  *uniform vertex,
                               uniform float  *uniform normal,
                               uniform vec4f  *uniform color,
                               uniform vec2f  *uniform texcoord,
                               uniform int32   geom_materialID,
                               uniform Material *uniform *uniform materialList,
                               uniform uint32 *uniform prim_materialID)
{
  Geometry_Constructor(&mesh->geometry,cppEquivalent,
                       RayforceGraph_postIntersect,
                       model,geomID,
                       materialList);
  mesh->geometry.getMaterialID = RayforceGraph_getMaterialID;
  mesh->geometry.getAreas = RayforceGraph_getAreas;
  mesh->geometry.sampleArea = RayforceGraph_sampleArea;
  mesh->geometry.numPrimitives = numTriangles;
  mesh->numTriangles = numTriangles;
  mesh->index        = index;
  mesh->vertex       = vertex;
  mesh->normal       = normal;
  mesh->color        = color;
  mesh->texcoord     = texcoord;
  mesh->idxSize      = idxSize;
  mesh->vtxSize      = vtxSize;
  mesh->norSize      = norSize;
  mesh->prim_materialID = prim_materialID;
  mesh->geom_materialID = geom_materialID;
}

export void *uniform RayforceGraph_create(void *uniform cppEquivalent)
{
  uniform RayforceGraph *uniform geom = uniform new uniform RayforceGraph;
  RayforceGraph_Constructor(geom, cppEquivalent, NULL, 0, 0, 0, 0, 0, NULL,
                            NULL, NULL, NULL, NULL, -1, NULL, NULL);
  return geom;
}

export void *uniform RayforceGraph_set(void *uniform _mesh,
                                       void *uniform _model,
                                       uniform int32  geomID,
                                       uniform int32  numTriangles,
                                       uniform int32  idxSize,
                                       uniform int32  vtxSize,
                                       uniform int32  norSize,
                                       uniform int    *uniform index,
                                       uniform float  *uniform vertex,
                                       uniform float  *uniform normal,
                                       uniform vec4f  *uniform color,
                                       uniform vec2f  *uniform texcoord,
                                       uniform int32   geom_materialID,
                                       void *uniform _materialList,
                                       uniform uint32 *uniform prim_materialID)
{
  uniform RayforceGraph *uniform mesh = (uniform RayforceGraph *uniform)_mesh;
  uniform Model *uniform model = (uniform Model *uniform)_model;
  uniform Material *uniform *uniform materialList =
      (uniform Material *uniform *uniform)_materialList;
  RayforceGraph_Constructor(mesh,
                            mesh->geometry.cppEquivalent,
                            model,
                            geomID,
                            numTriangles,
                            idxSize, vtxSize, norSize,
                            index,
                            vertex,
                            normal,
                            color,
                            texcoord,
                            geom_materialID,
                            (Material*uniform*uniform)materialList,
                            prim_materialID);
}
