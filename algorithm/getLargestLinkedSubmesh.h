#ifndef GETLARGERESTLINKEDSUBMESH_H
#define GETLARGERESTLINKEDSUBMESH_H
#include <ObjMesh.h>
#include <set>
#include <vector>
inline void getLargestLinkedSubmesh(ObjMesh &mesh, std::vector<int> &submeshIdx)
{
	
	std::vector<int> vertexGroupIdx;
	std::vector<std::vector<int>> groupVertexIdx;
	for (int i = 0; i < mesh.vertex_list.size(); i++)
		vertexGroupIdx.push_back(-1);
	int num_v_in_groups = 0;
	int groupIdx = 0;
	int largest_group_size = 0;
	int largest_group_index;
	mesh.selectNone();
	while (num_v_in_groups < mesh.vertex_list.size())
	{
		//printf("%d/%d\n", num_v_in_groups, mesh.vertex_list.size());
		for (int i = 0; i < mesh.vertex_list.size(); i++)
		{
			if (vertexGroupIdx[i] == -1)
			{
				mesh.selectLinkedVertices(i, ObjMesh::Select_OnlyGiven);
				std::vector<int> group_new;
				
				for (int j = 0; j < mesh.vertex_list.size(); j++)
				{
					if (mesh.isVertexSelected(j))
					{
						group_new.push_back(j);
						vertexGroupIdx[j] = groupIdx;
						num_v_in_groups++;
					}
				}
				//printf("group_new size = %d\n", group_new.size());
				//printf("largest_group_size = %d\n", largest_group_size);
				if (group_new.size() > largest_group_size)
				{
					largest_group_size = group_new.size();
					largest_group_index = groupIdx;
					//printf("largest_group_size updated\n", largest_group_size);
				}
				
				groupVertexIdx.push_back(group_new);
				mesh.selectNone();
				groupIdx++;
				break;
			}
		}
	}
	
	submeshIdx = groupVertexIdx[largest_group_index];
}
#endif