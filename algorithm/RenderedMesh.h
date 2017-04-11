#ifndef RENDEREDMESH_H
#define RENDEREDMESH_H
#include "ObjMesh.h"
#include "definations.h"
#include <set>
#include "kdtree.h"
#include "kdtree\Primitive.h"
#include "MicrosoftKinect.h"
#include <hash_set>
typedef std::vector<std::set<int>> SetList;
class RenderedMesh
{
public:
	RenderedMesh(){};
	~RenderedMesh(){};
	void init(ObjMesh mesh, dfusion::Intr intr,
		Tbx::Transfo transform, int height, int width);
	//void getRenderedIdx(std::set<int> &vertexIdx, std::set<int> &faceIdx,std::vector<Float3> &debug_vlist,int recursive_dep	
		void getRenderedIdx(std::set<int> &vertexIdx, std::set<int> &faceIdx, std::set<int> *vertexIdxHand = NULL, const std::set<PixelPos> *handpixel = NULL);
		void getRenderedHandCoveredIdx(std::set<int> *vertexIdxHand = NULL, const std::set<PixelPos> *handpixel = NULL);
		void getHandCoveredIdx(std::set<int> *vertexIdxHand = NULL, const std::set<PixelPos> *handpixel = NULL);
private:
	void buildVertexFaceMap();
	std::set<int> faceNeighbourToFace(int i_face);
	std::set<int> vertexNeighbourToVertex(int i_vertex);

	void insertNearVertexes(int i_vertex, std::set<int>& VertexSet, int recursive_depth);
	void insertNearFaces(int i_face, std::set<int> &FaceSet, int recursive_depth);
private:
	ObjMesh mesh;
	dfusion::Intr intr;
	Tbx::Transfo transform;
	int height;
	int width;
	SetList vertexContainedFace;
	ldp::kdtree::SAHKDTree kdtree;
	std::vector<ldp::kdtree::Primitive> kdata;
	 
};


#endif