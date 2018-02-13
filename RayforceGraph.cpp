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
#include "RayforceGraph.h"
#include "ospray/common/Model.h"
#include "../include/ospray/ospray.h"
// embree
#include "embree2/rtcore.h"
#include "embree2/rtcore_scene.h"
#include "embree2/rtcore_geometry.h"
#include "embree2/rtcore_ray.h"

#include "RayforceGraph_ispc.h"
#include "rfStruct.h"

extern rfPipeline rfRays;

namespace ospray {

template<typename T>
void getRay(const T& rays, RTCRay &ray, int i)
{
  ray.org[0] = rays.orgx[i];
  ray.org[1] = rays.orgy[i];
  ray.org[2] = rays.orgz[i];

  ray.dir[0] = rays.dirx[i];
  ray.dir[1] = rays.diry[i];
  ray.dir[2] = rays.dirz[i];

  ray.tnear  = rays.tnear[i];
  ray.tfar   = rays.tfar[i];

  ray.time   = rays.time[i];
  ray.mask   = rays.mask[i];

  ray.primID = rays.primID[i];
  ray.geomID = rays.geomID[i];
  ray.instID = rays.instID[i];
}

template<typename T>
void setRay(const RTCRay& ray, T &rays, int i)
{
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) {
    rays.Ngx[i] = ray.Ng[0];
    rays.Ngy[i] = ray.Ng[1];
    rays.Ngz[i] = ray.Ng[2];

    rays.primID[i] = ray.primID;
    rays.geomID[i] = ray.geomID;
    rays.instID[i] = ray.instID;
    rays.tfar[i]   = ray.tfar;
    rays.u[i]      = ray.u;
    rays.v[i]      = ray.v;
  }
}

static void rayforceBoundsFunc(const RayforceGraph* graphs,
                               size_t               item,
                               RTCBounds*           bounds_o)
{
  const RayforceGraph& graph = graphs[item];
  bounds_o->lower_x = graph.bounds.lower.x;
  bounds_o->lower_y = graph.bounds.lower.y;
  bounds_o->lower_z = graph.bounds.lower.z;
  bounds_o->upper_x = graph.bounds.upper.x;
  bounds_o->upper_y = graph.bounds.upper.y;
  bounds_o->upper_z = graph.bounds.upper.z;
}

static void traceRay(const RayforceGraph &graph, RTCRay &_ray)
{
  // Setup ray
  rfRaySingle ray;
  rfRaySingleInit(&ray);

#if 0
  ray.origin[0] = _ray.org[0];
  ray.origin[1] = _ray.org[1];
  ray.origin[2] = _ray.org[2];
#else
  ray.origin[0] = _ray.org[0] + _ray.dir[0] * _ray.tnear;
  ray.origin[1] = _ray.org[1] + _ray.dir[1] * _ray.tnear;
  ray.origin[2] = _ray.org[2] + _ray.dir[2] * _ray.tnear;
#endif

  ray.vector[0] = _ray.dir[0];
  ray.vector[1] = _ray.dir[1];
  ray.vector[2] = _ray.dir[2];

  ray.root = graph.rf_scene->resolve(ray.origin);

  ray.clipdist = _ray.tfar;
#if 0
  ray.skipdist = _ray.tnear;
#endif

  rfRayResults rayData;
  rayData.hit     = 0;
  rayData.hitDist = _ray.tfar;

  // Trace ray
  (*graph.rf_traceFcn)(ray, rayData);

  // Save hit data
  if (rayData.hit)
  {
    _ray.tfar   = rayData.hitDist;
    _ray.Ng[0]  = rayData.Ng[0];
    _ray.Ng[1]  = rayData.Ng[1];
    _ray.Ng[2]  = rayData.Ng[2];
    _ray.geomID = rayData.geomID;
    _ray.primID = rayData.primID;
    _ray.u      = rayData.u;
    _ray.v      = rayData.v;
  }
}

static void rayforceIntersectFunc(const RayforceGraph* graphs,
                                  RTCRay&              ray,
                                  size_t               item)
{
  const RayforceGraph& graph = graphs[item];
  traceRay(graph, ray);
}

static void rayforceIntersectFuncN(const int*                 valid,
                                   const RayforceGraph*       graphs,
                                   const RTCIntersectContext* context,
                                   RTCRayNp*                  rays,
                                   size_t                     N,
                                   size_t                     item)
{
  UNUSED(context);
  const RayforceGraph& graph = graphs[item];

  for (size_t i = 0; i < N; ++i) {
    if (valid[i]) {
      RTCRay ray;
      getRay(*rays, ray, i);
      traceRay(graph, ray);
      setRay(ray, *rays, i);
    }
  }
}

static void rayforceIntersectFunc1Mp(const RayforceGraph*       graphs,
                                     const RTCIntersectContext* context,
                                     RTCRay**                   rays,
                                     size_t                     M,
                                     size_t                     item)
{
  UNUSED(context);
  const RayforceGraph& graph = graphs[item];

  for (size_t i = 0; i < M; ++i){
    traceRay(graph, *rays[i]);
  }
}

template<int SIZE>
static void rayforceIntersectFuncNt(const int*           mask,
                                    const RayforceGraph* graphs,
                                    RTCRayNt<SIZE>&      _rays,
                                    size_t               item)
{
  RTCIntersectContext ctx;
  RTCRayNp rays {_rays.orgx, _rays.orgy, _rays.orgz, _rays.dirx, _rays.diry,
                 _rays.dirz, _rays.tnear, _rays.tfar, _rays.time, _rays.mask,
                 _rays.Ngx, _rays.Ngy, _rays.Ngz, _rays.u, _rays.v,
                 _rays.geomID, _rays.primID, _rays.instID};
  rayforceIntersectFuncN(mask, graphs, &ctx, &rays, SIZE, item);
}

// RayforceGraph definitions //////////////////////////////////////////////////

RayforceGraph::RayforceGraph()
  : eMesh(RTC_INVALID_GEOMETRY_ID)
{
  this->ispcEquivalent = ispc::RayforceGraph_create(this);

  rf_context = new rfut::Context;
  rf_device  = new rfut::Device<Target::System>(*rf_context);
  rf_scene   = new rfut::Scene<Target::System>(*rf_context, *rf_device);
  rf_object  = new rfut::Object(*rf_scene, CullMode::None);
  rf_model   = new rfut::Model(*rf_scene, ModelType::Triangles);
}

RayforceGraph::~RayforceGraph()
{
  delete rf_device;
  delete rf_model;
  delete rf_object;
  delete rf_scene;
  delete rf_context;
}

std::string RayforceGraph::toString() const
{
  return "ospray::RayforceGraph";
}

void RayforceGraph::finalize(Model *model)
{
  Geometry::finalize(model);

  RTCScene embreeSceneHandle = model->embreeSceneHandle;

  vertexData = getParamData("vertex",getParamData("position"));
  normalData = getParamData("vertex.normal",getParamData("normal"));
  colorData  = getParamData("vertex.color",getParamData("color"));
  texcoordData = getParamData("vertex.texcoord",getParamData("texcoord"));
  indexData  = getParamData("index",getParamData("triangle"));
  prim_materialIDData = getParamData("prim.materialID");
  geom_materialID = getParam1i("geom.materialID",-1);

  std::string saveGraphFile = getParamString("saveGraphFile", "");
  std::string loadGraphFile = getParamString("loadGraphFile", "");


  if (!vertexData)
    throw std::runtime_error("rayforce graph must have 'vertex' array");
  if (!indexData)
    throw std::runtime_error("rayforce graph must have 'index' array");
  if (colorData && colorData->type != OSP_FLOAT4 && colorData->type != OSP_FLOAT3A)
    throw std::runtime_error("vertex.color must have data type OSP_FLOAT4 or OSP_FLOAT3A");

  this->index = (int*)indexData->data;
  this->vertex = (float*)vertexData->data;
  this->normal = normalData ? (float*)normalData->data : nullptr;
  this->color  = colorData ? (vec4f*)colorData->data : nullptr;
  this->texcoord = texcoordData ? (vec2f*)texcoordData->data : nullptr;
  this->prim_materialID =
      prim_materialIDData ? (uint32*)prim_materialIDData->data : nullptr;

  size_t numVerts = -1;
  switch (indexData->type) {
  case OSP_INT:
  case OSP_UINT:  numTris = indexData->size() / 3; idxSize = 3; break;
  case OSP_INT3:
  case OSP_UINT3: numTris = indexData->size(); idxSize = 3; break;
  case OSP_UINT4:
  case OSP_INT4:  numTris = indexData->size(); idxSize = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.index data type");
  }

  switch (vertexData->type) {
  case OSP_FLOAT:   numVerts = vertexData->size() / 4; vtxSize = 4; break;
  case OSP_FLOAT3:  numVerts = vertexData->size(); vtxSize = 3; break;
  case OSP_FLOAT3A: numVerts = vertexData->size(); vtxSize = 4; break;
  case OSP_FLOAT4 : numVerts = vertexData->size(); vtxSize = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.vertex data type");
  }
  if (normalData) switch (normalData->type) {
  case OSP_FLOAT3:  norSize = 3; break;
  case OSP_FLOAT:
  case OSP_FLOAT3A: norSize = 4; break;
  default:
    throw std::runtime_error("unsupported vertex.normal data type");
  }

  eMesh = rtcNewUserGeometry(embreeSceneHandle, 1);

  if (loadGraphFile.empty()) {
    float *vertices = new float[numVerts*3];
    for (uint i = 0; i < numVerts; ++i) {
      auto *v = &vertex[vtxSize*i];
      vertices[3*i+0] = v[0];
      vertices[3*i+1] = v[1];
      vertices[3*i+2] = v[2];
    }

    uint *indices = new uint[numTris*3];
    for (uint i = 0; i < numTris; ++i) {
      auto *t = &index[idxSize*i];
      indices[3*i+0] = static_cast<uint>(t[0]);
      indices[3*i+1] = static_cast<uint>(t[1]);
      indices[3*i+2] = static_cast<uint>(t[2]);
    }

    rfTriangleData* tridata = new rfTriangleData[numTris];
    for (uint i = 0; i < numTris; ++i) {
      tridata[i].geomID = eMesh;
      tridata[i].primID = i;
    }

    rf_model->setData(numTris,
                      vertices,
                      indices,
                      sizeof(rfTriangleData),
                      sizeof(rfTriangleData),
                      tridata);

    // Cleanup
    delete [] vertices;
    delete [] indices;
    delete [] tridata;
  } else {
    rf_model->loadCacheFile(loadGraphFile);
  }

  rf_object->attach(*rf_model);
  rf_scene->acquire();
  rf_traceFcn = new rfut::TraceFcn<Target::System>(*rf_scene, rfRays);

  // Save graph cache
  if (!saveGraphFile.empty() && loadGraphFile.empty())
    rf_model->saveCacheFile(saveGraphFile);

  rtcSetUserData(embreeSceneHandle, eMesh, this);

  rtcSetBoundsFunction(embreeSceneHandle,
                       eMesh,
                       (RTCBoundsFunc)&rayforceBoundsFunc);


#ifdef OSPRAY_USE_EMBREE_STREAMS
  rtcSetIntersectFunction1Mp(embreeSceneHandle,
                             eMesh,
                             (RTCIntersectFunc1Mp)&rayforceIntersectFunc1Mp);

  rtcSetIntersectFunctionN(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFuncN)&rayforceIntersectFuncN);

  rtcSetOccludedFunction1Mp(embreeSceneHandle,
                            eMesh,
                            (RTCOccludedFunc1Mp)&rayforceIntersectFunc1Mp);

  rtcSetOccludedFunctionN(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFuncN)&rayforceIntersectFuncN);
#else
  rtcSetIntersectFunction(embreeSceneHandle,
                          eMesh,
                          (RTCIntersectFunc)&rayforceIntersectFunc);

  rtcSetIntersectFunction4(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc4)&rayforceIntersectFuncNt<4>);

  rtcSetIntersectFunction8(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc8)&rayforceIntersectFuncNt<8>);

  rtcSetIntersectFunction16(embreeSceneHandle,
                            eMesh,
                            (RTCIntersectFunc16)&rayforceIntersectFuncNt<16>);

  rtcSetOccludedFunction(embreeSceneHandle,
                         eMesh,
                         (RTCOccludedFunc)&rayforceIntersectFunc);

  rtcSetOccludedFunction4(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc4)&rayforceIntersectFuncNt<4>);

  rtcSetOccludedFunction8(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc8)&rayforceIntersectFuncNt<8>);

  rtcSetOccludedFunction16(embreeSceneHandle,
                           eMesh,
                           (RTCOccludedFunc16)&rayforceIntersectFuncNt<16>);
#endif

  bounds = empty;

  for (size_t i = 0; i < numVerts*vtxSize; i += vtxSize)
    bounds.extend(*(vec3f*)(vertex + i));

  ispc::RayforceGraph_set(getIE(),model->getIE(), eMesh,
                          numTris,
                          idxSize,
                          vtxSize,
                          norSize,
                          (int*)index,
                          (float*)vertex,
                          (float*)normal,
                          (ispc::vec4f*)color,
                          (ispc::vec2f*)texcoord,
                          geom_materialID,
                          materialList ? ispcMaterialPtrs.data() : nullptr,
                          (uint32_t*)prim_materialID);
}

OSP_REGISTER_GEOMETRY(RayforceGraph, rfgraph);
OSP_REGISTER_GEOMETRY(RayforceGraph, rayforce);

extern "C" void ospray_init_module_rayforce()
{
  printf("Loaded plugin 'rayforce' ...\n");
}

} // ::ospray
