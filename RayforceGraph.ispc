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

extern void rayforceIntersectFunc(void *uniform  graphs,
                                  void *uniform  ray,
                                  uniform uint64 item);

static void RayforceGraph_postIntersect(uniform Geometry *uniform _self,
                                        uniform Model *uniform model,
                                        varying DifferentialGeometry &dg,
                                        const varying Ray &ray,
                                        uniform int64 flags)
{
  uniform RayforceGraph *uniform self = (uniform RayforceGraph *uniform)_self;
  dg.Ng = dg.Ns = ray.Ng;

  const varying int indexBase = self->idxSize * ray.primID;
  const varying vec3i index = make_vec3i(self->index[indexBase+0],
                                         self->index[indexBase+1],
                                         self->index[indexBase+2]);
  const uniform float *uniform normal = self->normal;
  const uniform int32 norSize = self->norSize;
  if ((flags & DG_NS) && normal) {
    const uniform vec3f *const varying n0ptr =
        (const uniform vec3f *)&normal[index.x * norSize];
    const uniform vec3f *const varying n1ptr =
        (const uniform vec3f *)&normal[index.y * norSize];
    const uniform vec3f *const varying n2ptr =
        (const uniform vec3f *)&normal[index.z * norSize];
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
        const vec3f *const varying v0ptr =
            (const uniform vec3f *)&vertex[index.x * vtxSize];
        const vec3f *const varying v1ptr =
            (const uniform vec3f *)&vertex[index.y * vtxSize];
        const vec3f *const varying v2ptr =
            (const uniform vec3f *)&vertex[index.z * vtxSize];
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

    if( self->materialList) {
      dg.material = self->materialList[dg.materialID < 0 ? 0 : dg.materialID];
    }
  }
}

unmasked void RfGraph_bounds(uniform RayforceGraph *uniform self,
                             uniform size_t primID,
                             uniform box3fa &bbox)
{
#if 0
  uniform uint8 *uniform spherePtr = self->data
      + self->bytesPerSphere*((uniform int64)primID);
  uniform bool offr = self->offset_radius >= 0;
  uniform float radius =
      offr ? *((uniform float *uniform)(spherePtr+self->offset_radius)) :
             self->radius;
  uniform vec3f center =
      *((uniform vec3f*uniform)(spherePtr+self->offset_center));
  bbox = make_box3fa(center-make_vec3f(radius),center+make_vec3f(radius));
#endif
}

void RfGraph_intersect(uniform RayforceGraph *uniform self,
                       varying Ray &_ray,
                       uniform size_t primID)
{
  foreach_active(l) {
    uniform Ray ray;
    ray.org.x = extract(_ray.org.x, l);
    ray.org.y = extract(_ray.org.y, l);
    ray.org.z = extract(_ray.org.z, l);

    ray.dir.x = extract(_ray.dir.x, l);
    ray.dir.y = extract(_ray.dir.y, l);
    ray.dir.z = extract(_ray.dir.z, l);

    ray.t0 = extract(_ray.t0, l);
    ray.t  = extract(_ray.t, l);

    ray.time = extract(_ray.time, l);
    ray.mask = extract(_ray.mask, l);

    rayforceIntersectFunc(self[primID].geometry.cppEquivalent, &ray, primID);

    if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
      _ray.Ng.x = insert(_ray.Ng.x, l, ray.Ng.x);
      _ray.Ng.y = insert(_ray.Ng.y, l, ray.Ng.y);
      _ray.Ng.z = insert(_ray.Ng.z, l, ray.Ng.z);

      _ray.t = insert(_ray.t, l, ray.t);

      _ray.u = insert(_ray.u, l, ray.u);
      _ray.v = insert(_ray.v, l, ray.v);

      _ray.geomID = insert(_ray.geomID, l, ray.geomID);
      _ray.primID = insert(_ray.primID, l, ray.primID);
      _ray.instID = insert(_ray.instID, l, ray.instID);
    }
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
                               uniform Material *uniform material,
                               uniform Material *uniform *uniform materialList,
                               uniform uint32 *uniform prim_materialID)
{
  Geometry_Constructor(&mesh->geometry,cppEquivalent,
                       RayforceGraph_postIntersect,
                       model,geomID,
                       material);
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
  mesh->materialList = materialList;
  mesh->geom_materialID = geom_materialID;
}

export void *uniform RayforceGraph_create(void *uniform cppEquivalent)
{
  uniform RayforceGraph *uniform geom = uniform new uniform RayforceGraph;
  RayforceGraph_Constructor(geom, cppEquivalent, NULL, 0, 0, 0, 0, 0, NULL,
                            NULL, NULL, NULL, NULL, -1, NULL, NULL, NULL);
  return geom;
}

export void RayforceGraph_set(void *uniform _mesh,
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
                              void *uniform material,
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
                            (Material*uniform)material,
                            (Material*uniform*uniform)materialList,
                            prim_materialID);

  rtcSetUserData(model->embreeSceneHandle,geomID,mesh);
  rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
                       (uniform RTCBoundsFunc)&RfGraph_bounds);
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&RfGraph_intersect);
  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCOccludedFuncVarying)&RfGraph_intersect);
}
