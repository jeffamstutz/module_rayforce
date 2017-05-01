## ======================================================================== ##
## Copyright 2009-2015 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

option(OSPRAY_MODULE_RAYFORCE "Build 'RAYF' (rayforce model viewer)" ON)

if (OSPRAY_MODULE_RAYFORCE)

  if (DEFINED OSPRAY_MODULE_CPP)
    if (OSPRAY_MODULE_CPP)
      add_definitions(-DUSE_CPP_INTERFACE)
      add_definitions(-DSTREAM_SIZE=${OSPRAY_MODULE_CPP_STREAM_SIZE})
    endif()
  endif()

  find_package(Rayforce REQUIRED)

  include_directories(${RAYFORCE_INCLUDE_DIR})

  include_directories_ispc(${CMAKE_CURRENT_SOURCE_DIR})

  ospray_create_library(ospray_module_rayforce
    RayforceGraph.cpp
    RayforceGraph.h
    RayforceGraph.ispc
    rfPipeline.c
    rfStruct.h
    LINK
    ospray
    ${RAYFORCE_LIBRARIES}
  )

  set_source_files_properties(rfPipeline.c
    PROPERTIES COMPILE_FLAGS "-msse4.1 -std=gnu99")

  ospray_create_application(ospRayforceViewer
    rayforceViewer.cpp
    LINK
    ospray
    ospray_commandline
    ospray_minisg
    ospray_imgui3d
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
  )

endif()
