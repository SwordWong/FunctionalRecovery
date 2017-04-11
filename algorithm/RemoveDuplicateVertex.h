#ifndef REMOVEDUPLICATEVERTEX_H
#define REMOVEDUPLICATEVERTEX_H
#include <ObjMesh.h>
#include <vector>
#include <map>
#include <vector>
#include <PointTree.h>
#define EPSILON 1e-10
//struct cmp_key
//{
//	bool operator()(const Float3 &k1, const Float3 &k2)const
//	{
//		if (k1.x() != k2.x())
//		{
//			return k2.x() - k1.x() > EPSILON;
//		}
//		if (k1.y() != k2.y())
//		{
//			return k2.y() < k1.y() > EPSILON;
//		}
//		if (k1.z() != k2.z())
//		{
//			return k2.z() >k1.z() > EPSILON;
//		}
//		return false;
//	}
//};
//inline void removeDuplicateVertex(ObjMesh & mesh, std::vector<int> &vertexMap)
//{
//	std::vector<Float3> vertex_list_new;
//	std::vector<Float3> normal_list_new;
//	std::vector<int> num_list;
//	Float3 vertex, normal;
//	int index_new;
//	if (mesh.vertex_list.size() != vertexMap.size())
//	{
//		printf("Build vertexMap\n");
//		std::map<Float3, int, cmp_key> vmap;
//		std::map<Float3, int>::iterator iter;
//		
//		index_new = 0;
//		vertexMap.resize(mesh.vertex_list.size());
//		for (int i = 0; i < mesh.vertex_list.size(); i++)
//		{
//			vertex = mesh.vertex_list[i];
//			normal = mesh.vertex_normal_list[i];
//			iter = vmap.find(vertex);
//			//printf("removeDuplicateVertex:%d/%d\n", i, mesh.vertex_list.size());
//			//printf("vertex: %f, %f, %f\n", vertex.x(), vertex.y(), vertex.z());
//			//printf("normal: %f, %f, %f\n", normal.x(), normal.y(), normal.z());
//			if (iter == vmap.end())
//			{
//				vertexMap[i] = index_new;
//				//printf("push to vertexMap\n");
//				vmap.insert(std::pair<Float3, int>(vertex, index_new));
//				//printf("insert to vmap\n");
//				vertex_list_new.push_back(vertex);
//				//printf("push to vertex_list_new\n");
//				normal_list_new.push_back(normal);
//				//printf("push to normal_list_new\n");
//				num_list.push_back(1);
//				//printf("push to num_list\n");
//				index_new++;
//			}
//			else
//			{
//				int index = iter->second;
//				vertexMap[i] = index;
//				normal_list_new[index] += normal;
//				num_list[index]++;
//			}
//		}
//		//printf("new vertex list builded\n");
//		mesh.vertex_list.resize(vertex_list_new.size());
//		mesh.vertex_normal_list.resize(vertex_list_new.size());
//		for (int i = 0; i < mesh.vertex_list.size(); i++)
//		{	
//			mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
//			mesh.vertex_list[i] = vertex_list_new[i];
//		}
//		//printf("new vertex_normal_list builded\n");
//		for (int i = 0; i < mesh.face_list.size(); i++)
//		{
//			ObjMesh::obj_face &f = mesh.face_list[i];
//			f.material_index = -1;
//			f.vertex_count = 3;
//			for (int index_v = 0; index_v < 3; index_v++)
//			{
//				index_new = vertexMap[f.vertex_index[index_v]];
//				f.vertex_index[index_v] = index_new;
//				f.normal_index[index_v] = index_new;
//			}
//		}
//		
//	}
//	else
//	{
//		printf("Map by vertexMap\m");
//		int index;
//		vertex_list_new.resize(vertexMap.size());
//		
//		for (int i = 0; i < mesh.vertex_list.size(); i++)
//		{
//			normal_list_new.push_back(Float3(0, 0, 0));
//			num_list.push_back(0);
//		}
//			
//		for (int i = 0; i < mesh.vertex_list.size(); i++)
//		{
//			index = vertexMap[i];
//			vertex = mesh.vertex_list[i];
//			normal = mesh.vertex_normal_list[i];
//			vertex_list_new[index] = vertex;
//			normal_list_new[index] += normal;
//			num_list[index]++;
//		}
//		mesh.vertex_list.resize(vertex_list_new.size());
//		mesh.vertex_normal_list.resize(vertex_list_new.size());
//		for (int i = 0; i < mesh.vertex_list.size(); i++)
//		{
//			mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
//			mesh.vertex_list[i] = vertex_list_new[i];
//		}
//		for (int i = 0; i < mesh.face_list.size(); i++)
//		{
//			ObjMesh::obj_face &f = mesh.face_list[i];
//			f.material_index = -1;
//			f.vertex_count = 3;
//			for (int index_v = 0; index_v < 3; index_v++)
//			{
//				index_new = vertexMap[f.vertex_index[index_v]];
//				f.vertex_index[index_v] = index_new;
//				f.normal_index[index_v] = index_new;
//			}
//		}
//	}
//
//}
//inline void removeDuplicateVertexByVertexMap(ObjMesh & mesh, std::vector<int> &vertexMap, int size_new)
//{
//	std::vector<Float3> vertex_list_new;
//	std::vector<Float3> normal_list_new;
//	std::vector<int> num_list;
//	Float3 vertex, normal;
//	int index_new;
//	
//	printf("Map by vertexMap\m");
//	int index;
//	vertex_list_new.resize(size_new);
//
//	for (int i = 0; i < size_new; i++)
//	{
//		normal_list_new.push_back(Float3(0, 0, 0));
//		num_list.push_back(0);
//	}
//
//	for (int i = 0; i < mesh.vertex_list.size(); i++)
//	{
//		index = vertexMap[i];
//		vertex = mesh.vertex_list[i];
//		normal = mesh.vertex_normal_list[i];
//		vertex_list_new[index] = vertex;
//		normal_list_new[index] += normal;
//		num_list[index]++;
//	}
//	mesh.vertex_list.resize(size_new);
//	mesh.vertex_normal_list.resize(size_new);
//	for (int i = 0; i < mesh.vertex_list.size(); i++)
//	{
//		mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
//		mesh.vertex_list[i] = vertex_list_new[i];
//	}
//	for (int i = 0; i < mesh.face_list.size(); i++)
//	{
//		ObjMesh::obj_face &f = mesh.face_list[i];
//		f.material_index = -1;
//		f.vertex_count = 3;
//		for (int index_v = 0; index_v < 3; index_v++)
//		{
//			index_new = vertexMap[f.vertex_index[index_v]];
//			f.vertex_index[index_v] = index_new;
//			f.normal_index[index_v] = index_new;
//		}
//	}
//
//}
//inline int getVertexMap(const std::vector<Float3> &vertex_list, std::vector<int> &vertexMap, float min_dist)
//{
//	Float3 vertex;
//	ldp::kdtree::PointTree<float> pointTree;
//	std::vector<ldp::kdtree::PointTree<float>::Point> pointList;
//	ldp::kdtree::PointTree<float>::Point point, nearest;
//	vertexMap.resize(vertex_list.size());
//	int index_new = 0;
//	float dist;
//	for (int i = 0; i < vertex_list.size(); i++)
//	{
//		vertex = vertex = vertex_list[i];
//		point.p.x() = vertex.x();
//		point.p.y() = vertex.y();
//		point.p.z() = vertex.z();
//		point.idx = i;
//		if (pointList.empty())
//		{
//			pointList.push_back(point);
//			vertexMap[i] = index_new++;	
//		}
//		else
//		{
//			pointTree.build(pointList);
//			nearest = pointTree.nearestPoint(point, dist);
//			if (dist < min_dist)
//			{
//				vertexMap[i] = vertexMap[nearest.idx];
//			}
//			else
//			{
//				pointList.push_back(point);
//				vertexMap[i] = index_new++;
//			}
//		}
//	}
//	return index_new;
//}
inline int getVertexMap(const std::vector<Float3> &vertex_list, std::vector<int> &vertexMap, float min_dist, std::map<int,int> *inverseVertexMap = NULL)
{
	Float3 vertex;
	ldp::kdtree::PointTree<float> pointTree;
	std::vector<ldp::kdtree::PointTree<float>::Point> pointList;
	ldp::kdtree::PointTree<float>::Point point, nearest;
	vertexMap.resize(vertex_list.size());
	int index_new = 0;
	std::vector<float> dist;
	std::vector<int> idx;
	std::vector<std::pair<size_t, float>> indices_dists;
	
	for (int i = 0; i < vertex_list.size(); i++)
	{
		vertex = vertex = vertex_list[i];
		point.p.x() = vertex.x();
		point.p.y() = vertex.y();
		point.p.z() = vertex.z();
		point.idx = i;
		pointList.push_back(point);
		vertexMap[i] = -1;
	}
	pointTree.build(pointList);
	if (inverseVertexMap)
		inverseVertexMap->clear();
	for (int i = 0; i < pointList.size(); i++)
	{
		point = pointList[i];
		dist.clear();
		idx.clear();
		indices_dists.clear();
		
		//pointTree.kNearestPoints(point, idx.data(), dist.data(), k);
		///*if (vertexMap[i] != -1)
		//	continue;*/
		//for (int j = 0; j < k; j++)
		//{
		//	if (dist[j] < min_dist)
		//	{
		//		nearest = pointList[idx[j]];
		//		if (nearest.idx == point.idx)
		//			continue;
		//		if (vertexMap[nearest.idx] != -1)
		//		{
		//			vertexMap[i] = vertexMap[nearest.idx];
		//			break;
		//		}
		//	}
		//	else
		//	{
		//		break;
		//	}
		//}
		
		pointTree.pointInSphere(point, min_dist, indices_dists);
		if (indices_dists.size() == 0)
		{
			if (inverseVertexMap)
			{
				printf("index_new = %d\n", index_new);
				inverseVertexMap->insert(std::pair<int, int>(index_new, i));
				//(*inverseVertexMap)[index_new] = i;
			}
				
			vertexMap[i] = index_new++;
			
		}
		else
		{
			for (int j = 0; j < indices_dists.size(); j++)
			{
				if (indices_dists[j].first == i)
					continue;
				if (vertexMap[indices_dists[j].first] != -1)
				{
					vertexMap[i] = vertexMap[indices_dists[j].first];
					break;
				}
			}
			if (vertexMap[i] == -1)
			{
				
				if (inverseVertexMap)
				{
					//printf("index_new = %d\n", index_new);
					inverseVertexMap->insert(std::pair<int, int>(index_new, i));
					//(*inverseVertexMap)[index_new] = i;
				}
					
				vertexMap[i] = index_new++;
				
			}
		}
	}
	//printf("getVertexMap:inverseVertexMap size = %d\n", inverseVertexMap->size());
	return index_new;
}
inline void removeDuplicateVertexByVertexMap(ObjMesh & mesh, std::vector<int> &vertexMap, int size_new)
{
	std::vector<Float3> vertex_list_new;
	std::vector<Float3> vertex_color_list_new;
	std::vector<Float3> normal_list_new;
	std::vector<int> num_list;
	Float3 vertex, normal, color;
	int index_new;
	bool withColor = mesh.vertex_color_list.size() == mesh.vertex_list.size();
	//printf("Map by vertexMap\m");
	int index;
	vertex_list_new.resize(size_new);
	if(withColor)
		vertex_color_list_new.resize(size_new);
	for (int i = 0; i < size_new; i++)
	{
		normal_list_new.push_back(Float3(0, 0, 0));
		num_list.push_back(0);
	}
	printf("removeDuplicateVertexByVertexMap:	init finished\n");
	printf("removeDuplicateVertexByVertexMap:	vertexMap size = %d\n", vertexMap.size());
	for (int i = 0; i < mesh.vertex_list.size(); i++)
	{
		index = vertexMap[i];
		normal = mesh.vertex_normal_list[i];
		normal_list_new[index] += normal;
		num_list[index]++;
		if (num_list[index] > 1)
			continue;
		vertex = mesh.vertex_list[i];
		vertex_list_new[index] = vertex;
		if (withColor)
		{	
			//if (vertex_color_list_new[index].sqrDist(Float3(1.0f, 0.0f, 0.0f)) < 1e-5)
			//{
			//	//printf("r\n");
			//	continue;
			//}
				
			color = mesh.vertex_color_list[i];
			vertex_color_list_new[index] = color;
		}
	}
	printf("removeDuplicateVertexByVertexMap:	new list build\n");
	mesh.vertex_list.resize(size_new);
	mesh.vertex_normal_list.resize(size_new);
	if (withColor)
		mesh.vertex_color_list.resize(size_new);
	for (int i = 0; i < mesh.vertex_list.size(); i++)
	{
		mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
		mesh.vertex_list[i] = vertex_list_new[i];
		if (withColor)
			mesh.vertex_color_list[i] = vertex_color_list_new[i];
	}
	printf("removeDuplicateVertexByVertexMap:	new list assigned\n");
	for (int i = 0; i < mesh.face_list.size(); i++)
	{
		ObjMesh::obj_face &f = mesh.face_list[i];
		f.material_index = -1;
		f.vertex_count = 3;
		for (int index_v = 0; index_v < 3; index_v++)
		{
			index_new = vertexMap[f.vertex_index[index_v]];
			f.vertex_index[index_v] = index_new;
			f.normal_index[index_v] = index_new;
		}
	}
	printf("removeDuplicateVertexByVertexMap:	face updated\n");
}
inline void removeDuplicateVertex(ObjMesh & mesh, std::vector<int> &vertexMap)
{
	std::vector<Float3> vertex_list_new;
	std::vector<Float3> normal_list_new;
	std::vector<int> num_list;
	Float3 vertex, normal;
	int index_new;


	int num = getVertexMap(mesh.vertex_list, vertexMap, 0.5*1e-13);
	//printf("num = %d\n", num);
	removeDuplicateVertexByVertexMap(mesh, vertexMap, num);

	//ldp::kdtree::PointTree<float> pointTree;
	//if (mesh.vertex_list.size() != vertexMap.size())
	//{
	//	printf("Build vertexMap\n");
	//	
	//	std::map<Float3, int>::iterator iter;

	//	index_new = 0;
	//	vertexMap.resize(mesh.vertex_list.size());
	//	
	//	std::vector<ldp::kdtree::PointTree<float>::Point> pointList;
	//	ldp::kdtree::PointTree<float>::Point point;
	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		vertex = vertex = mesh.vertex_list[i];
	//		point.p.x() = vertex.x();
	//		point.p.y() = vertex.y();
	//		point.p.z() = vertex.z();
	//		pointList.push_back(point);
	//	}
	//	pointTree.build(pointList);

	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		vertex = mesh.vertex_list[i];
	//		normal = mesh.vertex_normal_list[i];
	//		//iter = vmap.find(vertex);
	//		//printf("removeDuplicateVertex:%d/%d\n", i, mesh.vertex_list.size());
	//		//printf("vertex: %f, %f, %f\n", vertex.x(), vertex.y(), vertex.z());
	//		//printf("normal: %f, %f, %f\n", normal.x(), normal.y(), normal.z());
	//		if (iter == vmap.end())
	//		{
	//			vertexMap[i] = index_new;
	//			//printf("push to vertexMap\n");
	//			vmap.insert(std::pair<Float3, int>(vertex, index_new));
	//			//printf("insert to vmap\n");
	//			vertex_list_new.push_back(vertex);
	//			//printf("push to vertex_list_new\n");
	//			normal_list_new.push_back(normal);
	//			//printf("push to normal_list_new\n");
	//			num_list.push_back(1);
	//			//printf("push to num_list\n");
	//			index_new++;
	//		}
	//		else
	//		{
	//			int index = iter->second;
	//			vertexMap[i] = index;
	//			normal_list_new[index] += normal;
	//			num_list[index]++;
	//		}
	//	}
	//	//printf("new vertex list builded\n");
	//	mesh.vertex_list.resize(vertex_list_new.size());
	//	mesh.vertex_normal_list.resize(vertex_list_new.size());
	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
	//		mesh.vertex_list[i] = vertex_list_new[i];
	//	}
	//	//printf("new vertex_normal_list builded\n");
	//	for (int i = 0; i < mesh.face_list.size(); i++)
	//	{
	//		ObjMesh::obj_face &f = mesh.face_list[i];
	//		f.material_index = -1;
	//		f.vertex_count = 3;
	//		for (int index_v = 0; index_v < 3; index_v++)
	//		{
	//			index_new = vertexMap[f.vertex_index[index_v]];
	//			f.vertex_index[index_v] = index_new;
	//			f.normal_index[index_v] = index_new;
	//		}
	//	}

	//}
	//else
	//{
	//	printf("Map by vertexMap\m");
	//	int index;
	//	vertex_list_new.resize(vertexMap.size());

	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		normal_list_new.push_back(Float3(0, 0, 0));
	//		num_list.push_back(0);
	//	}

	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		index = vertexMap[i];
	//		vertex = mesh.vertex_list[i];
	//		normal = mesh.vertex_normal_list[i];
	//		vertex_list_new[index] = vertex;
	//		normal_list_new[index] += normal;
	//		num_list[index]++;
	//	}
	//	mesh.vertex_list.resize(vertex_list_new.size());
	//	mesh.vertex_normal_list.resize(vertex_list_new.size());
	//	for (int i = 0; i < mesh.vertex_list.size(); i++)
	//	{
	//		mesh.vertex_normal_list[i] = normal_list_new[i] / num_list[i];
	//		mesh.vertex_list[i] = vertex_list_new[i];
	//	}
	//	for (int i = 0; i < mesh.face_list.size(); i++)
	//	{
	//		ObjMesh::obj_face &f = mesh.face_list[i];
	//		f.material_index = -1;
	//		f.vertex_count = 3;
	//		for (int index_v = 0; index_v < 3; index_v++)
	//		{
	//			index_new = vertexMap[f.vertex_index[index_v]];
	//			f.vertex_index[index_v] = index_new;
	//			f.normal_index[index_v] = index_new;
	//		}
	//	}
	//}

}

#endif // !REMOVEDUPLICATEVERTEX_H
