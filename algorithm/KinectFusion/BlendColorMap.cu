#include "stdio.h"
#include "BlendColorMap.h"
#include "device_utils.h"
#include <helper_math.h>
#include "device_launch_parameters.h"

__global__ void BlendColorMapKernal(PtrStepSz<uchar4> rgba1, PtrStepSz<uchar4> rgba2, PtrStepSz<uchar4> rgbaDst)
{
	int u = threadIdx.x + blockIdx.x * blockDim.x;
	int v = threadIdx.y + blockIdx.y * blockDim.y;

	if (u < rgba1.cols && v < rgba1.rows)
	{
		uchar4 color;
		int a1, a2, a3;
		a1 = rgba1.ptr(v)[u].w;
		a2 = rgba2.ptr(v)[u].w;
		if (a1 == 0 && a2 == 0)
		{
			color.x = color.y = color.z = color.w = 1;
			return;
		}
		//if(rgba1.ptr(v)[u].x == 0 && rgba1.ptr(v)[u].y == 0 && rgba1.ptr(v)[u].z == 0)
		//	a1 = 0;
		a1 *= 6;
		color.x = (rgba1.ptr(v)[u].x * a1 + rgba2.ptr(v)[u].x * a2)/(a1+a2);
		color.y = (rgba1.ptr(v)[u].y * a1 + rgba2.ptr(v)[u].y * a2)/(a1+a2);
		color.z = (rgba1.ptr(v)[u].z * a1 + rgba2.ptr(v)[u].z * a2)/(a1+a2);
		rgbaDst.ptr(v)[u] = color;
	}
}
//__global__ __forceinline__ void blendColorMapKernel
void BlendColorMap(const dfusion::ColorMap & map1, const dfusion::ColorMap & map2, dfusion::ColorMap &dst)
{
	if (map1.cols() != map2.cols() || map2.rows() != map2.rows())
	{
		printf("BlendColorMap: size mismatch\n");
		return;
	}
	dst.release();
	dst.create(map1.rows(), map1.cols());

	dim3 block(32, 8);
	dim3 grid(1, 1, 1);
	grid.x = divUp(map1.cols(), block.x);
	grid.y = divUp(map1.rows(), block.y);
	BlendColorMapKernal << <grid, block >> > (map1, map2, dst);
	cudaSafeCall(cudaGetLastError(), "BlendColorMap");
}