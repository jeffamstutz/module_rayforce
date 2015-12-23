
#pragma once

#include <RF/rf.h>

#define RF_TERMINATE 0
#define RF_CONTINUE  1

typedef unsigned int  uint;
typedef unsigned char uchar;

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  // Per-triangle data ////////////////////////////////////////////////////////

  typedef struct
  {
    int geomID;
    int primID;
  } rfTriangleData;

  // Per-ray data /////////////////////////////////////////////////////////////

  typedef struct
  {
    int   hit;
    float hitDist;
    int   geomID;
    int   primID;
    float Ng[3];
    float u;
    float v;
  } rfRayResults;

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus
