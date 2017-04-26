#include "Oversegement.h"

#include <limits>
#include <math.h>
#include "ObjMesh.h"

using namespace std;
using namespace ldp;
const Oversegement::real PI  = 3.14159265358979323846;

Oversegement::Oversegement()
{
	threshold_distance = 0.01;
	threshold_cosine = cos(15/180*PI);
}
Oversegement::~Oversegement()
{
}

int Oversegement::segement(ObjMesh & mesh, std::vector<int>& vertexPatchId)
{
	kdtreeList.clear();
	patchVertexIdxList.clear();
	patchPointsList.clear();

	int num_vertex = mesh.vertex_list.size();
	if (num_vertex != mesh.vertex_normal_list.size())
		mesh.updateNormals();
	vertexPatchId = vector<int>(num_vertex, -1);
	list<int> unlabel_vertex_idx(num_vertex);
	list<int>::iterator it = unlabel_vertex_idx.begin();
	for (int i = 0; it != unlabel_vertex_idx.end(); i++, it++)*it = i;
	while (unlabel_vertex_idx.size() > 0) {
		int idx_vertex = *unlabel_vertex_idx.begin();
		Float3 v = mesh.vertex_list[idx_vertex];
		KDTree::Point p(ldp_basic_vec3<real>(v.x(), v.y(), v.z()), idx_vertex);

		real min_dis = numeric_limits<double>::max();
		int idx_closest_patch = -1;
		for (int idx_kdtree = 0; idx_kdtree < kdtreeList.size(); idx_kdtree++) {
			
			float dist;
			KDTree::Point np = kdtreeList[idx_kdtree].nearestPoint(p, dist);
			Float3 nv = mesh.vertex_list[np.idx];
			real cosine = v.dot(nv) / (v.length()*nv.length());
			if (dist < threshold_distance && cosine < threshold_cosine && dist < min_dis)
			{
				idx_closest_patch = idx_kdtree;
				min_dis = dist;
			}
		}
		if (idx_closest_patch > -1)
		{
			patchPointsList[idx_closest_patch].push_back(p);
			kdtreeList[idx_closest_patch].build(patchPointsList[idx_closest_patch]);
			patchVertexIdxList[idx_closest_patch].push_back(idx_vertex);
			vertexPatchId[idx_vertex] = idx_closest_patch;
		}
		else
		{
			KDTree tree;
			IDVec patchVertexIdx;
			pointVec patchPoints;
			kdtreeList.push_back(tree);

			patchVertexIdx.push_back(idx_vertex);
			patchPoints.push_back(p);
			kdtreeList[kdtreeList.size() - 1].build(patchPoints);

			patchPointsList.push_back(patchPoints);
			patchVertexIdxList.push_back(patchVertexIdx);
			
		}
		unlabel_vertex_idx.erase(unlabel_vertex_idx.begin());
	}
	return patchVertexIdxList.size();
}
