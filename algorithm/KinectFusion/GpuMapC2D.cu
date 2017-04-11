#include "GpuMapC2D.h"
#include "device_utils.h"
#include <helper_math.h>
#include "device_launch_parameters.h"
#include <thrust/sort.h>
#include <thrust/functional.h>


struct C2DMapper
{
	//const int* handPixel;
	const NUI_COLOR_IMAGE_POINT* coordMapping;
	uchar4* color_list;
	int* handflag;

	uchar4* color_src;
	uchar4* color_dst;

	dfusion::depthtype* depth;

	int color_dis_sq;

	int rows;
	int cols;

	int color_list_size;

	int num_hand_pixel;
	__device__ __forceinline__ void operator()(int row, int col)
	{
		if (row >= 0 && row < rows && col >= 0 && col < cols)
		{
			int index = row*cols + col;
			if (depth[index] == 0)
				return;
			int colorInDepthX = coordMapping[index].x;
			int colorInDepthY = coordMapping[index].y;

			unsigned int sourceColorIndex = colorInDepthX + (colorInDepthY * cols);
			uchar4 color = color_src[sourceColorIndex];

			color_dst[index] = color;
			if (toRemove(color))
			{
				//num_hand_pixel++;
				Remove(index);
			}
		}

	}
	__device__ __forceinline__ bool Remove(int index)
	{
		//if (row >= 0 && row < rows && col >= 0 && col < cols)
		//{
			depth[index] = 0;
			//int index = row*cols + col;
			handflag[index] = 1;
		//}
	}
	__device__ __forceinline__ bool toRemove(uchar4 color)
	{

		for (int i = 0; i < color_list_size; i++)
		{
			int dis_sq = 0;
			dis_sq += (color.x - color_list[i].x)*(color.x - color_list[i].x);
			dis_sq += (color.y - color_list[i].y)*(color.y - color_list[i].y);
			dis_sq += (color.z - color_list[i].z)*(color.z - color_list[i].z);
			if (dis_sq < color_dis_sq)
				return true;
		}
		return false;
	}

};

__global__ void Color2Depth_kernal(C2DMapper mapper)
{
	
	int col = threadIdx.x + blockIdx.x * blockDim.x;
	int row = threadIdx.y + blockIdx.y * blockDim.y;

	mapper(row, col);
}
__global__ void init_index_kernal(int* index_d, int size)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	if(index < size)
		index_d[index] = index;
}
//void mapC2D_GPU(dfusion::depthtype* depth, unsigned char* dst_pBGRA,
//	const unsigned char* src_pBGRA, const std::vector<dfusion::PixelRGBA> &colorList, std::set<PixelPos> &handPixel,
//	std::vector<NUI_COLOR_IMAGE_POINT> &coordMapping, int color_dis)
//{
//	//printf("mapC2D_GPU:color_dis = %d\n", color_dis);
//	dfusion::depthtype* depth_d;
//	dfusion::PixelRGBA* src_color_d;
//	dfusion::PixelRGBA* dst_color_d;
//
//
//	NUI_COLOR_IMAGE_POINT* coordMapping_d;
//	dfusion::PixelRGBA* colorList_d;
//	bool* handflag_d;
//
//
//	cudaMalloc((void**)&depth_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype));
//	cudaMalloc((void**)&dst_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA));
//	cudaMalloc((void**)&src_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA));
//	
//
//	cudaMalloc((void**)&handflag_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(bool));
//	cudaMalloc((void**)&coordMapping_d, coordMapping.size() * sizeof(NUI_COLOR_IMAGE_POINT));
//	cudaMalloc((void**)&colorList_d, colorList.size() * sizeof(dfusion::PixelRGBA));
//
//	//printf("mapC2D_GPU:cudaMalloc\n");
//
//
//	cudaMemset(handflag_d, false, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT);
//	//cudaMemcpy(handflag_d, handPixel.data(), handPixel.size() * sizeof(PixelPos), cudaMemcpyHostToDevice);
//	cudaMemcpy(coordMapping_d, coordMapping.data(), coordMapping.size() * sizeof(NUI_COLOR_IMAGE_POINT), cudaMemcpyHostToDevice);
//	cudaMemcpy(colorList_d, colorList.data(), colorList.size() * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);
//
//
//	cudaMemcpy(depth_d, depth, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype), cudaMemcpyHostToDevice);
//	cudaMemcpy(src_color_d, src_pBGRA, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);
//	//cudaMemcpy(dst_color_d, src_pBGRA, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);
//	//printf("mapC2D_GPU:cudaMemcpy\n");
//
//	C2DMapper mapper;
//
//	mapper.depth = depth_d;
//	mapper.color_src = (uchar4*)src_color_d;
//	mapper.color_dst = (uchar4*)dst_color_d;
//	mapper.color_list = (uchar4*)colorList_d;
//	mapper.handflag = handflag_d;
//	mapper.coordMapping = coordMapping_d;
//	
//	mapper.color_list_size = colorList.size();
//	mapper.rows = dfusion::KINECT_HEIGHT;
//	mapper.cols = dfusion::KINECT_WIDTH;
//
//	mapper.color_dis_sq = color_dis*color_dis;
//	mapper.num_hand_pixel = 0;
//	dim3 block(32, 8);
//	dim3 grid(1, 1, 1);
//	grid.x = divUp(mapper.cols, block.x);
//	grid.y = divUp(mapper.rows, block.y);
//
//	//printf("mapC2D_GPU:init finished\n");
//
//	Color2Depth_kernal << <grid, block >> > (mapper);
//	cudaSafeCall(cudaGetLastError(), "mapColor2Depth_GPU");
//
//	cudaMemcpy(depth, depth_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype), cudaMemcpyDeviceToHost);
//	cudaMemcpy((dfusion::PixelRGBA*)dst_pBGRA, dst_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyDeviceToHost);
//	
//	
//	handPixel.clear();
//	//std::vector<bool> handflag;
//	bool handflag[dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT];
//	//printf("mapC2D_GPU:handflag.resize\n");
//	cudaMemcpy(handflag, handflag_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(bool), cudaMemcpyDeviceToHost);
//	//cudaSafeCall(cudaGetLastError(), "mapColor2Depth_GPU");
//	//printf("mapC2D_GPU:cudaMemcpy handflag\n");
//	for (int i = 0; i < dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT; i++)
//	{
//		//printf("mapC2D_GPU:i = %d\n",i);
//		if(handflag[i])
//			handPixel.insert(i);
//	}
//	//printf("mapC2D_GPU:handPixel size = %d\n",handPixel.size());
//
//	cudaFree(depth_d);
//	cudaFree(src_color_d);
//	cudaFree(dst_color_d);
//
//	cudaFree(colorList_d);
//	cudaFree(handflag_d);
//	cudaFree(coordMapping_d);
//}

GpuMapper::GpuMapper()
{
	printf("init GpuMapper\n");
	color_list_size = 0;

	cudaMalloc((void**)&depth_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype));
	cudaMalloc((void**)&dst_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA));
	cudaMalloc((void**)&src_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA));


	cudaMalloc((void**)&handflag_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(int));
	cudaMalloc((void**)&handflag_index_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(int));
	cudaMalloc((void**)&coordMapping_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(NUI_COLOR_IMAGE_POINT));
	//cudaMalloc((void**)&colorList_d, colorList.size() * sizeof(dfusion::PixelRGBA));
	printf("GpuMapper init\n");
}

GpuMapper::~GpuMapper()
{
	cudaFree(depth_d);
	cudaFree(src_color_d);
	cudaFree(dst_color_d);

	//cudaFree(colorList_d);
	cudaFree(handflag_d);
	cudaFree(handflag_index_d);
	cudaFree(coordMapping_d);
}

void GpuMapper::map(dfusion::depthtype * depth, unsigned char * dst_pBGRA, const unsigned char * src_pBGRA, const std::vector<dfusion::PixelRGBA>& colorList, std::set<PixelPos>* handPixel, std::vector<NUI_COLOR_IMAGE_POINT>& coordMapping, int color_dis)
{
	//printf("GpuMapper::map\n");
	cudaMalloc((void**)&colorList_d, colorList.size() * sizeof(dfusion::PixelRGBA));
	
	cudaMemset(handflag_d, 0, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT*sizeof(int));
	cudaMemset(dst_color_d, 0, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA));
	
	cudaMemcpy(coordMapping_d, coordMapping.data(), coordMapping.size() * sizeof(NUI_COLOR_IMAGE_POINT), cudaMemcpyHostToDevice);
	cudaMemcpy(colorList_d, colorList.data(), colorList.size() * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);


	cudaMemcpy(depth_d, depth, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype), cudaMemcpyHostToDevice);
	cudaMemcpy(src_color_d, src_pBGRA, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);
	//cudaMemcpy(dst_color_d, src_pBGRA, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);
	//printf("mapC2D_GPU:cudaMemcpy\n");

	C2DMapper mapper;

	mapper.depth = depth_d;
	mapper.color_src = (uchar4*)src_color_d;
	mapper.color_dst = (uchar4*)dst_color_d;
	mapper.color_list = (uchar4*)colorList_d;
	mapper.handflag = handflag_d;
	mapper.coordMapping = coordMapping_d;

	color_list_size = colorList.size();

	mapper.color_list_size = colorList.size();
	mapper.rows = dfusion::KINECT_HEIGHT;
	mapper.cols = dfusion::KINECT_WIDTH;

	mapper.num_hand_pixel = 0;

	mapper.color_dis_sq = color_dis*color_dis;

	dim3 block(32, 8);
	dim3 grid(1, 1, 1);
	grid.x = divUp(mapper.cols, block.x);
	grid.y = divUp(mapper.rows, block.y);

	//printf("mapC2D_GPU:init finished\n");

	Color2Depth_kernal << <grid, block >> > (mapper);
	//cudaSafeCall(cudaGetLastError(), "mapColor2Depth_GPU");

	cudaMemcpy(depth, depth_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::depthtype), cudaMemcpyDeviceToHost);
	cudaMemcpy((dfusion::PixelRGBA*)dst_pBGRA, dst_color_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(dfusion::PixelRGBA), cudaMemcpyDeviceToHost);

	if (handPixel)
	{
		handPixel->clear();
		//std::vector<bool> handflag;

		//printf("mapC2D_GPU:handflag.resize\n");

		if (color_list_size > 0)
		{
			int n = dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT;
			//printf("mapper.num_hand_pixel = %d\n", mapper.num_hand_pixel);

			//init_handflag_index();

			//int* handflag = new int[dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT];
			//int* handflag_index = new int[dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT];

			for (int i = 0; i < n; i++)handflag_index_h[i] = i;

			cudaMemcpy(handflag_h, handflag_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(int), cudaMemcpyDeviceToHost);
			//cudaMemcpy(handflag_index, handflag_index_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT * sizeof(int), cudaMemcpyDeviceToHost);
			//
			thrust::sort_by_key(handflag_h, handflag_h + n, handflag_index_h);
			//printf("sorted\n");
			int index = n - 1;
			while (index >= 0 && handflag_h[index])
			{
				handPixel->insert(handflag_index_h[index]);
				index--;
			}
			/*for (int i = 0; i < n; i++)
			{
			if (handflag[i]);
			handPixel.insert(i);
			}*/
			//delete[] handflag_index;
			//delete[] handflag;
		}

		//printf("mapC2D_GPU:handPixel size = %d\n", handPixel->size());
	}
	

	
	cudaFree(colorList_d);
	
}

void GpuMapper::init_handflag_index()
{
	dim3 block(256, 1);
	dim3 grid(1, 1, 1);
	grid.x = divUp(dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT, block.x);
	init_index_kernal << <grid, block >> > (handflag_index_d, dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT);
}

void GpuMapper::set_color_list(const std::vector<dfusion::PixelRGBA>& colorList)
{
	release_color_list();
	cudaMalloc((void**)&colorList_d, colorList.size() * sizeof(dfusion::PixelRGBA));
	cudaMemcpy(colorList_d, colorList.data(), colorList.size() * sizeof(dfusion::PixelRGBA), cudaMemcpyHostToDevice);

	color_list_size = colorList.size();
}

void GpuMapper::release_color_list()
{
	if(color_list_size > 0)
		cudaFree(colorList_d);
	color_list_size = 0;
}
