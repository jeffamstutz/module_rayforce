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

#include "widgets/OSPGlutViewer.h"
#include "commandline/Utility.h"
#include "commandline/SceneParser/trianglemesh/TriangleMeshSceneParser.h"
#include "ospray/ospray.h"

class RayforceSceneParser : public TriangleMeshSceneParser
{
public:
  RayforceSceneParser(ospray::cpp::Renderer renderer,
                      std::string geometryType = "rayforce") :
    TriangleMeshSceneParser(renderer, geometryType) {}
};

int main(int ac, const char **av)
{
  ospInit(&ac,av);
  ospray::glut3D::initGLUT(&ac,av);

  ospLoadModule("rayforce");
  auto ospObjs = parseCommandLine<DefaultRendererParser,
                                  DefaultCameraParser,
                                  RayforceSceneParser,
                                  DefaultLightsParser>(ac, av);

  ospcommon::box3f      bbox;
  ospray::cpp::Model    model;
  ospray::cpp::Renderer renderer;
  ospray::cpp::Camera   camera;

  std::tie(bbox, model, renderer, camera) = ospObjs;

  ospray::OSPGlutViewer window(bbox, model, renderer, camera);
  window.create("ospRayforceViewer: OSPRay Rayforce Viewer");

  ospray::glut3D::runGLUT();
}
