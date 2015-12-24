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
    rays.tnear[i]  = ray.tnear;
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
  rayData.hit = 0;

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
    _ray.instID = rayData.geomID;
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

static void rayforceIntersectFunc4(const void*          _mask,
                                   const RayforceGraph* graphs,
                                   RTCRay4&             rays,
                                   size_t               item)
{
  const RayforceGraph& graph = graphs[item];
  const int *mask = (int*)_mask;

  for (int i = 0; i < 4; ++i) {
    if (mask[i]) {
      RTCRay ray;
      getRay(rays, ray, i);
      traceRay(graph, ray);
      setRay(ray, rays, i);
    }
  }
}

static void rayforceIntersectFunc8(const void*          _mask,
                                   const RayforceGraph* graphs,
                                   RTCRay8&             rays,
                                   size_t               item)
{
  const RayforceGraph& graph = graphs[item];
  const int *mask = (int*)_mask;

  for (int i = 0; i < 8; ++i) {
    if (mask[i]) {
      RTCRay ray;
      getRay(rays, ray, i);
      traceRay(graph, ray);
      setRay(ray, rays, i);
    }
  }
}

static void rayforceIntersectFunc16(const void*          _mask,
                                    const RayforceGraph* graphs,
                                    RTCRay16&            rays,
                                    size_t               item)
{
  const RayforceGraph& graph = graphs[item];
  const int *mask = (int*)_mask;

  for (int i = 0; i < 16; ++i) {
    if (mask[i]) {
      RTCRay ray;
      getRay(rays, ray, i);
      traceRay(graph, ray);
      setRay(ray, rays, i);
    }
  }
}

RayforceGraph::RayforceGraph()
  : eMesh(RTC_INVALID_GEOMETRY_ID)
{
  this->ispcMaterialPtrs = NULL;
  this->ispcEquivalent = ispc::RayforceGraph_create(this);

  rf_context = new rfut::Context;
  rf_device  = new rfut::Device<Target::System>(*rf_context);
  rf_scene   = new rfut::Scene<Target::System>(*rf_context, *rf_device);
  rf_object  = new rfut::Object(*rf_scene, CullMode::None);
  rf_model   = new rfut::Model(*rf_scene, ModelType::Triangles, 224, 224);
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
  return "ospray::TriangleMesh";
}

void RayforceGraph::finalize(Model *model)
{
  Assert(model && "invalid model pointer");

  RTCScene embreeSceneHandle = model->embreeSceneHandle;

  vertexData = getParamData("vertex",getParamData("position"));
  normalData = getParamData("vertex.normal",getParamData("normal"));
  colorData  = getParamData("vertex.color",getParamData("color"));
  texcoordData = getParamData("vertex.texcoord",getParamData("texcoord"));
  indexData  = getParamData("index",getParamData("triangle"));
  prim_materialIDData = getParamData("prim.materialID");
  materialListData = getParamData("materialList");
  geom_materialID = getParam1i("geom.materialID",-1);

  Assert2(vertexData != NULL,
          "triangle mesh geometry does not have either 'position'"
          " or 'vertex' array");
  Assert2(indexData != NULL,
          "triangle mesh geometry does not have either 'index'"
          " or 'triangle' array");

  this->index = (int*)indexData->data;
  this->vertex = (float*)vertexData->data;
  this->normal = normalData ? (float*)normalData->data : NULL;
  this->color  = colorData ? (vec4f*)colorData->data : NULL;
  this->texcoord = texcoordData ? (vec2f*)texcoordData->data : NULL;
  this->prim_materialID  = prim_materialIDData ? (uint32*)prim_materialIDData->data : NULL;
  this->materialList  = materialListData ? (ospray::Material**)materialListData->data : NULL;

  if (materialList && !ispcMaterialPtrs) {
    const int num_materials = materialListData->numItems;
    ispcMaterialPtrs = new void*[num_materials];
    for (int i = 0; i < num_materials; i++) {
      assert(this->materialList[i] != NULL &&
             "Materials in list should never be NULL");
      this->ispcMaterialPtrs[i] = this->materialList[i]->getIE();
    }
  }

  size_t numTris  = -1;
  size_t numVerts = -1;

  size_t numCompsInTri = 0;
  size_t numCompsInVtx = 0;
  size_t numCompsInNor = 0;
  switch (indexData->type) {
  case OSP_INT:
  case OSP_UINT:  numTris = indexData->size() / 3; numCompsInTri = 3; break;
  case OSP_INT3:
  case OSP_UINT3: numTris = indexData->size(); numCompsInTri = 3; break;
  case OSP_UINT4:
  case OSP_INT4:  numTris = indexData->size(); numCompsInTri = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.index data type");
  }

  switch (vertexData->type) {
  case OSP_FLOAT:   numVerts = vertexData->size() / 4; numCompsInVtx = 4; break;
  case OSP_FLOAT3:  numVerts = vertexData->size(); numCompsInVtx = 3; break;
  case OSP_FLOAT3A: numVerts = vertexData->size(); numCompsInVtx = 4; break;
  case OSP_FLOAT4 : numVerts = vertexData->size(); numCompsInVtx = 4; break;
  default:
    throw std::runtime_error("unsupported trianglemesh.vertex data type");
  }
  if (normalData) switch (normalData->type) {
  case OSP_FLOAT3:  numCompsInNor = 3; break;
  case OSP_FLOAT:
  case OSP_FLOAT3A: numCompsInNor = 4; break;
  default:
    throw std::runtime_error("unsupported vertex.normal data type");
  }

  eMesh = rtcNewUserGeometry(embreeSceneHandle, 1);

  float *vertices = new float[numVerts*3];
  for (uint i = 0; i < numVerts; ++i) {
    auto *v = &vertex[numCompsInVtx*i];
    vertices[3*i+0] = v[0];
    vertices[3*i+1] = v[1];
    vertices[3*i+2] = v[2];
  }

  uint *indices = new uint[numTris*3];
  for (uint i = 0; i < numTris; ++i) {
    auto *t = &index[numCompsInTri*i];
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

  rf_object->attach(*rf_model);
  rf_scene->acquire();
  rf_traceFcn = new rfut::TraceFcn<Target::System>(*rf_scene, rfRays);

  // Cleanup
  delete [] vertices;
  delete [] indices;
  delete [] tridata;

  rtcSetUserData(embreeSceneHandle, eMesh, this);

  rtcSetBoundsFunction(embreeSceneHandle,
                       eMesh,
                       (RTCBoundsFunc)&rayforceBoundsFunc);

  rtcSetIntersectFunction(embreeSceneHandle,
                          eMesh,
                          (RTCIntersectFunc)&rayforceIntersectFunc);

  rtcSetIntersectFunction4(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc4)&rayforceIntersectFunc4);

  rtcSetIntersectFunction8(embreeSceneHandle,
                           eMesh,
                           (RTCIntersectFunc8)&rayforceIntersectFunc8);

  rtcSetIntersectFunction16(embreeSceneHandle,
                            eMesh,
                            (RTCIntersectFunc16)&rayforceIntersectFunc16);

  rtcSetOccludedFunction(embreeSceneHandle,
                         eMesh,
                         (RTCOccludedFunc)&rayforceIntersectFunc);

  rtcSetOccludedFunction4(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc4)&rayforceIntersectFunc4);

  rtcSetOccludedFunction8(embreeSceneHandle,
                          eMesh,
                          (RTCOccludedFunc8)&rayforceIntersectFunc8);

  rtcSetOccludedFunction16(embreeSceneHandle,
                           eMesh,
                           (RTCOccludedFunc16)&rayforceIntersectFunc16);

  bounds = embree::empty;

  for (int i = 0; i < numVerts * numCompsInVtx; i += numCompsInVtx) {
    bounds.extend(*(vec3f*)(vertex + i));
  }

  ispc::RayforceGraph_set(getIE(),model->getIE(),
                          eMesh,
                          numTris,
                          numCompsInTri,
                          numCompsInVtx,
                          numCompsInNor,
                          (int*)index,
                          (float*)vertex,
                          (float*)normal,
                          (ispc::vec4f*)color,
                          (ispc::vec2f*)texcoord,
                          geom_materialID,
                          getMaterial()?getMaterial()->getIE():NULL,
                          ispcMaterialPtrs,
                          (uint32*)prim_materialID);
}

OSP_REGISTER_GEOMETRY(RayforceGraph,rfgraph);
OSP_REGISTER_GEOMETRY(RayforceGraph,rayforce);

extern "C" void ospray_init_module_rayforce()
{
  printf("Loaded plugin 'rayforce' ...\n");
}

} // ::ospray
