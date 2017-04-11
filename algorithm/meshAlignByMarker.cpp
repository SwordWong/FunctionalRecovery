#include "meshAlignByMarker.h"
#include <iostream>
using namespace rigid_estimation;
void transformVertexList(std::vector<Float3> &vertexList, const Mat3 &R, const Point &T)
{
	Point dst, src;
	for (int i = 0; i < vertexList.size(); i++)
	{
		src = Point(vertexList[i].x(), vertexList[i].y(), vertexList[i].z());
		dst = R*src + T;
		vertexList[i] = Float3(dst.x(), dst.y(), dst.z());
	}
}
void transformVertexList(std::vector<Float3> &vertexList, const Mat3 &R, const Point &T, const Point &S)
{
	Point dst, src;
	for (int i = 0; i < vertexList.size(); i++)
	{
		src = Point(vertexList[i].x(), vertexList[i].y(), vertexList[i].z());
		dst = R*S.cwiseProduct(src) + T;
		vertexList[i] = Float3(dst.x(), dst.y(), dst.z());
	}
}
void meshAlignByMarker(const ObjMesh& dst, ObjMesh & src, std::vector<int> &MarkerIdx, bool scale)
{

	std::vector<Point> dstMarker, srcMarker;
	Point pointDst, pointSrc;
	Float3 point;
	for (int i = 0; i < MarkerIdx.size(); i++)
	{
		point = dst.vertex_list[MarkerIdx[i]];
		pointDst = Point(point.x(), point.y(), point.z());
		dstMarker.push_back(pointDst);
		point = src.vertex_list[MarkerIdx[i]];
		pointSrc = Point(point.x(), point.y(), point.z());
		srcMarker.push_back(pointSrc);
	}
	Mat3 R;
	Point T;
	if (scale)
	{
		Point S;
		estimate_rigid_with_scale(MarkerIdx.size(), srcMarker.data(), dstMarker.data(), R, T, S);
		transformVertexList(src.vertex_list, R, T, S);
	}
	else
	{
		estimate_rigid(MarkerIdx.size(), srcMarker.data(), dstMarker.data(), R, T);
		transformVertexList(src.vertex_list, R, T);
	}
	src.updateNormals();
}
Tbx::Transfo getTransfoByMarker(const ObjMesh & dst, ObjMesh & src, std::vector<int>& MarkerIdx, bool scale)
{
	std::vector<Point> dstMarker, srcMarker;
	Point pointDst, pointSrc;
	Float3 point;
	for (int i = 0; i < MarkerIdx.size(); i++)
	{
		point = dst.vertex_list[MarkerIdx[i]];
		pointDst = Point(point.x(), point.y(), point.z());
		dstMarker.push_back(pointDst);
		point = src.vertex_list[MarkerIdx[i]];
		pointSrc = Point(point.x(), point.y(), point.z());
		srcMarker.push_back(pointSrc);
	}
	Mat3 R;
	Point T;
	if (scale)
	{
		Point S;
		estimate_rigid_with_scale(MarkerIdx.size(), srcMarker.data(), dstMarker.data(), R, T, S);
	}
	else
	{
		estimate_rigid(MarkerIdx.size(), srcMarker.data(), dstMarker.data(), R, T);
	}
	
	/*std::cout << "R:" << std::endl << R << std::endl;
	std::cout << "T:" << std::endl << T << std::endl;*/
	float a = R(0, 0), b = R(0, 1), c = R(0, 2),
		d = R(1, 0), e = R(1, 1), f = R(1, 2),
		g = R(2, 0), h = R(2, 1), i = R(2, 2);
	Tbx::Mat3 R_tbx = Tbx::Mat3(a, b, c,
		d, e, f,
		g, h, i);

	Tbx::Vec3 T_tbx = Tbx::Vec3(T.x(), T.y(), T.z());


	return Tbx::Transfo(R_tbx, T_tbx);
}
