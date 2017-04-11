#pragma once
#include "definations.h"
#include <set>
#include <vector>
#include "MicrosoftKinect.h"
class GpuMapper
{
public:
	GpuMapper();
	~GpuMapper();
	void map(dfusion::depthtype* depth, unsigned char* dst_pBGRA,
		const unsigned char* src_pBGRA, const std::vector<dfusion::PixelRGBA> &colorList, std::set<PixelPos>* handPixel,
		std::vector<NUI_COLOR_IMAGE_POINT> &coordMapping, int color_dis = 50);
	void init_handflag_index();
	void set_color_list(const std::vector<dfusion::PixelRGBA> &colorList);
	void release_color_list();
protected:
	dfusion::depthtype* depth_d;
	dfusion::PixelRGBA* src_color_d;
	dfusion::PixelRGBA* dst_color_d;


	NUI_COLOR_IMAGE_POINT* coordMapping_d;
	dfusion::PixelRGBA* colorList_d;

	int handflag_h[dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT];
	int handflag_index_h[dfusion::KINECT_WIDTH * dfusion::KINECT_HEIGHT];

	int* handflag_d;
	int* handflag_index_d;

	int color_list_size;

	int width;
	int height;
private:
	
};
//void mapC2D_GPU(dfusion::depthtype* depth, unsigned char* dst_pBGRA,
//	const unsigned char* src_pBGRA, const std::vector<dfusion::PixelRGBA> &colorList, std::set<PixelPos> &handPixel,
//	std::vector<NUI_COLOR_IMAGE_POINT> &coordMapping, int color_dis = 50);