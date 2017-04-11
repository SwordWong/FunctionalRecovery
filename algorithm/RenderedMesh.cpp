#include "RenderedMesh.h"

#define  DEBUG_OUTPUT 0

void transformVertexList(std::vector<Float3> &vertexList, Tbx::Transfo tr)
{
	Tbx::Point3 dst, src;
	
	for (int i = 0; i < vertexList.size(); i++)
	{
		src = Tbx::Point3(vertexList[i].x(), vertexList[i].y(), vertexList[i].z());
		dst = Tbx::Point3(tr.get_mat3()*src + tr.get_translation());
		vertexList[i] = Float3(dst.x, dst.y, dst.z);
	}
}
ldp::kdtree::Ray getRayFromUV(dfusion::Intr intr, int u, int v)
{
	Float3 src = Float3(0, 0, 0);
	float3 dst_f3 = intr.uvd2xyz((float)u, (float)v, 1.0f);
	Float3 dst = Float3(dst_f3.x, dst_f3.y, dst_f3.z);
	Float3 rayDir = dst - src;
	rayDir.normalize();
	return ldp::kdtree::Ray(src,rayDir);
}
void insertFaceVertexIntoSet(const ObjMesh::obj_face& f, std::set<int> &VertexSet)
{
	for (int i_vertex_in_face = 0; i_vertex_in_face < f.vertex_count; i_vertex_in_face++)
	{
		int vertex_idx = f.vertex_index[i_vertex_in_face];
		VertexSet.insert(vertex_idx);
	}
}
void insertContainedFaceIntoSet(const SetList &vcf, int i_vertex, std::set<int> &FaceSet)
{
	std::set<int>::iterator iter;
	//printf("vcf size = %d\n", vcf.size());
	for (iter = vcf[i_vertex].begin(); iter != vcf[i_vertex].end(); iter++)
	{
		//printf("%d\n", *iter);
		FaceSet.insert(*iter);
	}
}
void insertLindedFaceIntoSet(const SetList &vcf, const ObjMesh::obj_face& f, std::set<int> &FaceSet)
{
	for (int i_vertex_in_face = 0; i_vertex_in_face < f.vertex_count; i_vertex_in_face++)
	{
		int vertex_idx = f.vertex_index[i_vertex_in_face];
		std::set<int>::iterator iter;
		for (iter = vcf[vertex_idx].begin(); iter != vcf[vertex_idx].end(); iter++)
		{
			FaceSet.insert(*iter);
		}
	}
}
void insertLinkedVertexIntoSet(const SetList &vcf, const std::vector<ObjMesh::obj_face> faceList,int i_vertex, std::set<int> &VertexSet)
{
	std::set<int>::iterator iter;
	for (iter = vcf[i_vertex].begin(); iter != vcf[i_vertex].end(); iter++)
	{
		int i_face = *iter;
		const ObjMesh::obj_face& f = faceList[i_face];
		insertFaceVertexIntoSet(f, VertexSet);
	}
}



std::set<int> RenderedMesh::faceNeighbourToFace(int i_face)
{
	std::set<int> FaceSet;
	const ObjMesh::obj_face& f = mesh.face_list[i_face];
	for (int i_vertex_in_face = 0; i_vertex_in_face < f.vertex_count; i_vertex_in_face++)
	{
		int vertex_idx = f.vertex_index[i_vertex_in_face];
		insertContainedFaceIntoSet(vertexContainedFace, vertex_idx, FaceSet);
	}
	FaceSet.erase(FaceSet.find(i_face));
	return FaceSet;
}
std::set<int> RenderedMesh ::vertexNeighbourToVertex(int i_vertex)
{
	std::set<int> VertexSet;
	insertLinkedVertexIntoSet(vertexContainedFace, mesh.face_list, i_vertex, VertexSet);
	std::set<int>::iterator iter;
	
	/*for (iter = VertexSet.begin(); iter != VertexSet.end(); iter++)
	{
		VertexSet.insert(*iter);
	}*/
	VertexSet.erase(VertexSet.find(i_vertex));
	return VertexSet;
}
void RenderedMesh::insertNearVertexes(int i_vertex, std::set<int>& VertexSet, int recursive_depth)
{
#if DEBUG_OUTPUT
	printf("i_vertex = %d, recursive_depth = %d\n", i_vertex, recursive_depth);
#endif
	VertexSet.insert(i_vertex);
	if (recursive_depth <= 0)
		return;

	std::set<int> neigVertex = vertexNeighbourToVertex(i_vertex);
	std::set<int>::iterator iter;
	for (iter = neigVertex.begin(); iter != neigVertex.end(); iter++)
	{
		std::set<int>::iterator it_find = VertexSet.find(*iter);
		if (it_find == VertexSet.end())
			insertNearVertexes(*iter, VertexSet, recursive_depth - 1);
	}
}
void RenderedMesh::insertNearFaces(int i_face, std::set<int> &FaceSet, int recursive_depth)
{
#if DEBUG_OUTPUT
	//printf("i_face = %d, recursive_depth = %d\n", i_face, recursive_depth);
#endif
	FaceSet.insert(i_face);
	if (recursive_depth <= 0)
		return;
	std::set<int> neigFace = faceNeighbourToFace(i_face);
	std::set<int>::iterator iter;
	for (iter = neigFace.begin(); iter != neigFace.end(); iter++)
	{
		std::set<int>::iterator it_find = FaceSet.find(*iter);
		if (it_find == FaceSet.end())
			insertNearFaces(*iter, FaceSet, recursive_depth - 1);
	}
}
void RenderedMesh::init(ObjMesh mesh, dfusion::Intr intr,
	Tbx::Transfo transform, int height, int width)
{
	this->mesh = mesh;
	this->intr = intr;
	this->transform = transform;
	this->height = height;
	this->width = width;

	buildVertexFaceMap();
}
void RenderedMesh::buildVertexFaceMap()
{
	this->vertexContainedFace.resize(mesh.vertex_list.size());
	for (int i_face = 0; i_face < mesh.face_list.size(); i_face++)
	{
		const ObjMesh::obj_face& f = mesh.face_list[i_face];
		for (int i_vertex_in_face = 0; i_vertex_in_face < f.vertex_count; i_vertex_in_face++)
		{
			
			int vertex_idx = f.vertex_index[i_vertex_in_face];
			vertexContainedFace[vertex_idx].insert(i_face);
			///printf("i_face = %d, vertex_idx =%d\n", i_face, vertex_idx);
		}
	}
	printf("buildVertexFaceMap:VertexFaceMap built\n");
	printf("buildVertexFaceMap:VertexFaceMap size = %d\n", vertexContainedFace.size());
}
void addRayVertex(ldp::kdtree::Ray ray, float dist, int num, std::vector<Float3> &debug_vlist)
{
	Float3 src = Float3(ray.o[0], ray.o[1], ray.o[2]);
	Float3 dir = Float3(ray.d[0], ray.d[1], ray.d[2]);
	Float3 v;
	for (int i = 1; i <= num; i++)
	{
		v = src + dir*dist*i / (float)num;
		//printf("v = %f, %f, %f\n", v.x(), v.y(), v.z());
		debug_vlist.push_back(v);
	}
}
bool inFace(int i_vertex, const ObjMesh::obj_face& f)
{
	for (int i = 0; i <= f.vertex_count; i++)
	{
		if (f.vertex_index[i] == i_vertex)
			return true;
	}
	return false;
}
//void RenderedMesh::getRenderedIdx(std::set<int> &vertexIdx, std::set<int> &faceIdx,std::vector<Float3> &debug_vlist, int recursive_depth)
void RenderedMesh::getRenderedIdx(std::set<int> &vertexIdx, std::set<int> &faceIdx,
	std::set<int> *vertexIdxHand, const std::set<PixelPos> *handpixel)
{
	printf("getRenderedIdx:VertexFaceMap size = %d\n", vertexContainedFace.size());
	const static ldp::Int3 od[2] = { ldp::Int3(0, 1, 2), ldp::Int3(0, 2, 3) };
	kdata.clear();
	kdata.reserve(mesh.face_list.size() * 6);
	SetList vertexNeign;
	transformVertexList(mesh.vertex_list, transform);
	vertexIdx.clear();
	faceIdx.clear();
	for (int i = 0; i < mesh.face_list.size(); i++)
	{
		
		const ObjMesh::obj_face& f = mesh.face_list[i];
		for (int j = 0; j <= f.vertex_count - 3; j++)
		{
			ldp::Float3* v[3];
			for (int k = 0; k < 3; k++)
				v[k] = &mesh.vertex_list[f.vertex_index[od[j][k]]];
			ldp::kdtree::Primitive pm((int)ldp::kdtree::Primitive::TRIANGLE, v[0], v[1], v[2]);
			pm.index = i;
			kdata.push_back(pm);
		}
	}
	kdtree.build(kdata);
	printf("kdtree built\n");
	ldp::kdtree::Ray ray;
	ldp::kdtree::Primitive* pm;
	int face_id;
	PixelPos ppos;
	if (vertexIdxHand)
		vertexIdxHand->clear();
	for (int i_vertex = 0; i_vertex < mesh.vertex_list.size(); i_vertex++)
	{
		//printf("i_vertex = %d/%d\n", i_vertex, mesh.vertex_list.size());
		Float3 src = Float3(0, 0, 0);
		Float3 dst = mesh.vertex_list[i_vertex];
		Float3 dir = dst - src;
		ldp::kdtree::real dist = std::numeric_limits<ldp::kdtree::real>::max();;
		ray = ldp::kdtree::Ray(src, dir);
		//printf("1\n");
		dir.normalize();
		int sect = kdtree.intersect(ray, dist, pm, 0);
		const ObjMesh::obj_face& f = mesh.face_list[pm->index];
		//printf("2\n");
		if (sect != HIT || inFace(i_vertex, f))
		{
			if (vertexIdxHand && handpixel)
			{
				float3 uvd;
				uvd = intr.xyz2uvd(dst.x(),dst.y(),dst.z());
				/*ppos.x = uvd.x;
				ppos.y = uvd.y;*/
				ppos = (int)uvd.y * width + (int)uvd.x;

				//printf("(x,y) = (%d,%d), ppos = %d\n", (int)uvd.x, (int)uvd.y, ppos);
				std::hash_set<PixelPos>::iterator iter;
				if (handpixel->find(ppos) != handpixel->end())
					vertexIdxHand->insert(i_vertex);
				
			}
			vertexIdx.insert(i_vertex);
			insertLinkedVertexIntoSet(vertexContainedFace, mesh.face_list, i_vertex, vertexIdx);
			insertContainedFaceIntoSet(vertexContainedFace, i_vertex, faceIdx);
			//insertNearVertexes(i_vertex, vertexIdx, recursive_depth);
		}
	}
//	for (int i = 0; i < width; i++)
//	{
//		for (int j = 0; j < height; j++)
//		{
//			ray = getRayFromUV(intr, i, j);
//
//			ldp::kdtree::real dist = std::numeric_limits<ldp::kdtree::real>::max();
//			int sect = kdtree.intersect(ray, dist, pm, 0);
//			
//			if (sect == HIT)
//			{
//				face_id = pm->index;
//#if DEBUG_OUTPUT
//				printf("i = %d, j = %d, face_id = %d\n", i, j, face_id);
//#endif
//				//addRayVertex(ray, dist, 5, debug_vlist);
//				if (faceIdx.find(face_id) == faceIdx.end())
//				{
//					insertNearFaces(face_id, faceIdx, recursive_depth);
//#if DEBUG_OUTPUT
//					printf("face inserted\n");
//#endif
//					const ObjMesh::obj_face& f = mesh.face_list[face_id];
//					for (int i_vertex_in_face = 0; i_vertex_in_face < f.vertex_count; i_vertex_in_face++)
//					{
//						int vertex_id = f.vertex_index[i_vertex_in_face];
//						insertNearVertexes(vertex_id, vertexIdx, recursive_depth);
//					}
//#if DEBUG_OUTPUT
//					printf("vertex inserted\n");
//#endif
//				}
//			}
//			else
//			{
//				//addRayVertex(ray, 0.8, 5, debug_vlist);
//			}
//		}
//	}
//	mesh.saveObj("ttt.obj");
}
void RenderedMesh::getRenderedHandCoveredIdx(std::set<int>* vertexIdxHand, const std::set<PixelPos>* handpixel)
{
	const static ldp::Int3 od[2] = { ldp::Int3(0, 1, 2), ldp::Int3(0, 2, 3) };
	kdata.clear();
	kdata.reserve(mesh.face_list.size() * 6);
	SetList vertexNeign;
	transformVertexList(mesh.vertex_list, transform);
	for (int i = 0; i < mesh.face_list.size(); i++)
	{
		const ObjMesh::obj_face& f = mesh.face_list[i];
		for (int j = 0; j <= f.vertex_count - 3; j++)
		{
			ldp::Float3* v[3];
			for (int k = 0; k < 3; k++)
				v[k] = &mesh.vertex_list[f.vertex_index[od[j][k]]];
			ldp::kdtree::Primitive pm((int)ldp::kdtree::Primitive::TRIANGLE, v[0], v[1], v[2]);
			pm.index = i;
			kdata.push_back(pm);
		}
	}
	kdtree.build(kdata);
	printf("getHandCoveredIdx:kdtree built\n");
	ldp::kdtree::Ray ray;
	ldp::kdtree::Primitive* pm;
	int face_id;
	PixelPos ppos;
	if (vertexIdxHand)
		vertexIdxHand->clear();
	for (int i_vertex = 0; i_vertex < mesh.vertex_list.size(); i_vertex++)
	{
		//printf("i_vertex = %d/%d\n", i_vertex, mesh.vertex_list.size());
		Float3 src = Float3(0, 0, 0);
		Float3 dst = mesh.vertex_list[i_vertex];
		Float3 dir = dst - src;
		ldp::kdtree::real dist = std::numeric_limits<ldp::kdtree::real>::max();;
		ray = ldp::kdtree::Ray(src, dir);
		dir.normalize();
		int sect = kdtree.intersect(ray, dist, pm, 0);
		const ObjMesh::obj_face& f = mesh.face_list[pm->index];
		if (sect != HIT || inFace(i_vertex, f))
		{
			if (vertexIdxHand && handpixel)
			{
				float3 uvd;
				uvd = intr.xyz2uvd(dst.x(), dst.y(), dst.z());
				ppos = (int)uvd.y * width + (int)uvd.x;

				std::hash_set<PixelPos>::iterator iter;
				if (handpixel->find(ppos) != handpixel->end())
					vertexIdxHand->insert(i_vertex);

			}
		}
	}
}

void RenderedMesh::getHandCoveredIdx(std::set<int>* vertexIdxHand, const std::set<PixelPos>* handpixel)
{
	int face_id;
	PixelPos ppos;
	if (vertexIdxHand)
		vertexIdxHand->clear();
	for (int i_vertex = 0; i_vertex < mesh.vertex_list.size(); i_vertex++)
	{
		Float3 src = Float3(0, 0, 0);
		Float3 dst = mesh.vertex_list[i_vertex];
		if (vertexIdxHand && handpixel)
		{
			float3 uvd;
			uvd = intr.xyz2uvd(dst.x(), dst.y(), dst.z());
			ppos = (int)uvd.y * width + (int)uvd.x;

			std::hash_set<PixelPos>::iterator iter;
			if (handpixel->find(ppos) != handpixel->end())
				vertexIdxHand->insert(i_vertex);

		}
	}
}
