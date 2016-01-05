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

  if (NOT OSPRAY_APPS_MODELVIEWER)
    message(WARNING "You must enable OSPRAY_APPS_MODELVIEWER to build the Rayforce module!")
  endif()

  find_package(Rayforce REQUIRED)

  configure_ospray()

  # -------------------------------------------------------

  include_directories(
    ${PROJECT_SOURCE_DIR}/ospray
    ${PROJECT_SOURCE_DIR}/ospray/geometry
    ${PROJECT_SOURCE_DIR}/ospray/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RAYFORCE_INCLUDE_DIR}
  )

  include_directories_ispc(
    ${PROJECT_SOURCE_DIR}/ospray
    ${PROJECT_SOURCE_DIR}/ospray/geometry
    ${PROJECT_SOURCE_DIR}/ospray/include
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  ospray_add_library(ospray_module_rayforce${OSPRAY_LIB_SUFFIX} SHARED
    RayforceGraph.cpp
    RayforceGraph.h
    RayforceGraph.ispc
    rfPipeline.c
    rfStruct.h
  )

  set_source_files_properties(rfPipeline.c
    PROPERTIES COMPILE_FLAGS "-msse4.1 -std=gnu99")

  target_link_libraries(ospray_module_rayforce${OSPRAY_LIB_SUFFIX}
    ospray${OSPRAY_LIB_SUFFIX}
    ${RAYFORCE_LIBRARIES}
  )

  if (NOT THIS_IS_MIC)
    include_directories(
      ${PROJECT_SOURCE_DIR}/apps/modelViewer
    )

    add_executable(ospRayforceViewer
      rayforceViewer.cpp
    )

    target_link_libraries(ospRayforceViewer
      ospray
      ospray_minisg
      ospray_glut3d
      ${OPENGL_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
  endif()
endif()
