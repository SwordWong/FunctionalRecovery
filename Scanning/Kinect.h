#pragma once
#include <string>

#include <windows.h>
#include <NuiApi.h>
#pragma comment(lib, "Kinect10.lib")
#include "definations.h"
struct BGRA32Pixel {
	unsigned char nBlue;
	unsigned char nGreen;
	unsigned char nRed;
	unsigned char alpha;
};
class Kinect {
public:
	Kinect();
	~Kinect();
	int InitKinect();
	//	get data
	int GetDepthColorIntoBuffer(dfusion::depthtype* depth, unsigned char* pBGRA,
		 bool mirror_input = true);
	int GetDepthMap(bool mirror);
	int GetColorMap(bool mirror);
private:
	
	int FreeSpace();
	void ErrorCheck(HRESULT status, std::string str);

public:
	//	current frame id
	int frame_id;
	Kinect* ref_kinect;

	//	depth data
	int dep_width, dep_height;
	float dep_h_fov, dep_v_fov;

	//	color data
	int img_width, img_height;
	float img_h_fov, img_v_fov;
	BGRA32Pixel*		image_map;
	NUI_DEPTH_IMAGE_PIXEL*	depth_map;

	//	Kinect SDK Interface, v1
	INuiSensor* pNuiSensor;
	HANDLE		pColorStreamHandle;
	HANDLE		pDepthStreamHandle;
	HANDLE		hNextColorFrameEvent;
	HANDLE		hNextDepthFrameEvent;
};