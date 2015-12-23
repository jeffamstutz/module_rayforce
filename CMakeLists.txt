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

OPTION(OSPRAY_MODULE_RAYFORCE "Build 'RAYF' (rayforce model viewer)" ON)

IF (OSPRAY_MODULE_RAYFORCE)

  find_package(Rayforce REQUIRED)

  CONFIGURE_OSPRAY()

  # -------------------------------------------------------

  INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/ospray
    ${PROJECT_SOURCE_DIR}/ospray/geometry
    ${PROJECT_SOURCE_DIR}/ospray/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${RAYFORCE_INCLUDE_DIR}
  )

  INCLUDE_DIRECTORIES_ISPC(
    ${PROJECT_SOURCE_DIR}/ospray
    ${PROJECT_SOURCE_DIR}/ospray/geometry
    ${PROJECT_SOURCE_DIR}/ospray/include
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

  OSPRAY_ADD_LIBRARY(ospray_module_rayforce${OSPRAY_LIB_SUFFIX} SHARED
    RayforceGraph.cpp
    RayforceGraph.h
    RayforceGraph.ispc
    rfPipeline.c
    rfStruct.h
  )

  SET_SOURCE_FILES_PROPERTIES(rfPipeline.c
    PROPERTIES COMPILE_FLAGS "-msse4.1 -std=gnu99")

  TARGET_LINK_LIBRARIES(ospray_module_rayforce${OSPRAY_LIB_SUFFIX}
    ospray${OSPRAY_LIB_SUFFIX}
    ${RAYFORCE_LIBRARIES}
  )

  IF (NOT THIS_IS_MIC)
    INCLUDE_DIRECTORIES(
      ${PROJECT_SOURCE_DIR}/apps/modelViewer
    )

    ADD_EXECUTABLE(ospRayforce
      rayforceViewer.cpp
    )

    TARGET_LINK_LIBRARIES(ospRayforce
      ospray
      ospray_minisg
      ospray_glut3d
      ${OPENGL_LIBRARIES}
      ${GLUT_LIBRARIES}
    )
  ENDIF()
ENDIF()
