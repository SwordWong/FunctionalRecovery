#ifndef MESHALIGNBYMARKER_H
#define MESHALIGNBYMARKER_H
#include "ObjMesh.h"
#include "RigidEstimation.h"
#include <vector>
#include "definations.h"
void meshAlignByMarker(const ObjMesh& dst, ObjMesh & src, std::vector<int> &MarkerIdx, bool scale = false);
Tbx::Transfo getTransfoByMarker(const ObjMesh& dst, ObjMesh & src, std::vector<int> &MarkerIdx, bool scale = false);
#endif // !MESHALIGNBYMARKER_H
