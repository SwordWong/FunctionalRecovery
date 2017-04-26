#pragma once
#include <vector>
#include <list>
#include "PointTree.h"
class ObjMesh;
class Oversegement {
public:
	Oversegement();
	~Oversegement();

	int segement(ObjMesh& mesh, std::vector<int>& vertexPatchId);
public:
	typedef float real;
	typedef ldp::kdtree::PointTree<real> KDTree;
	typedef std::vector<int> IDVec;
	typedef std::vector<KDTree::Point> pointVec;
	float threshold_distance;
	float threshold_cosine;

	std::vector<KDTree> kdtreeList;
	std::vector<IDVec> patchVertexIdxList;
	std::vector<pointVec> patchPointsList;

};
