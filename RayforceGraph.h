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

#pragma once

#include "Geometry.h"
#include "ospray/common/Data.h"

#include <rfut/CullMode.h>
#include <rfut/ModelType.h>
#include <rfut/Target.h>

#include <rfut/Buffer.t>
#include <rfut/Context.h>
#include <rfut/Device.t>
#include <rfut/Model.h>
#include <rfut/Object.h>
#include <rfut/Scene.t>
#include <rfut/TraceFcn.t>

namespace ospray {

  struct RayforceGraph : public Geometry
  {

    RayforceGraph();
    ~RayforceGraph();
    std::string toString() const override;
    void finalize(Model *model) override;

    const int    *index;  //!< mesh's triangle index array
    const float  *vertex; //!< mesh's vertex array
    const float  *normal; //!< mesh's vertex normal array
    const vec4f  *color;  //!< mesh's vertex color array
    const vec2f  *texcoord; //!< mesh's vertex texcoord array
    const uint32 *prim_materialID; //!< per-primitive material ID
    Material **materialList; //!< per-primitive material list
    int geom_materialID;

    Ref<Data> indexData;  /*!< triangle indices (A,B,C,materialID) */
    Ref<Data> vertexData; /*!< vertex position (vec3fa) */
    Ref<Data> normalData; /*!< vertex normal array (vec3fa) */
    Ref<Data> colorData;  /*!< vertex color array (vec3fa) */
    Ref<Data> texcoordData; /*!< vertex texcoord array (vec2f) */
    Ref<Data> prim_materialIDData;  /*!< data array for per-prim material ID (uint32) */
    Ref<Data> materialListData; /*!< data array for per-prim materials */
    uint32    eMesh;   /*!< embree triangle mesh handle */

    //! Target-independent data
    rfut::Context* rf_context;
    rfut::Object*  rf_object;
    rfut::Model*   rf_model;

    //! Target-dependent data
    rfut::Scene<Target::System>*    rf_scene;
    rfut::Device<Target::System>*   rf_device;
    rfut::TraceFcn<Target::System>* rf_traceFcn;

    void** ispcMaterialPtrs; /*!< pointers to ISPC equivalent materials */
  };

} // ::ospray
