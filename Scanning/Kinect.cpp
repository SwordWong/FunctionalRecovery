#include "Kinect.h"
#include <iostream>
using namespace  std;
Kinect::Kinect()
{
	frame_id = 0;
	ref_kinect = NULL;

	dep_width = 0;
	dep_height = 0;
	dep_h_fov = 0.0f;
	dep_v_fov = 0.0f;
	depth_map = NULL;

	img_width = 0;
	img_height = 0;
	img_h_fov = 0.0f;
	img_v_fov = 0.0f;
	image_map = NULL;
}

Kinect::~Kinect()
{
	int FreeSpace();
}

int Kinect::GetDepthColorIntoBuffer(dfusion::depthtype * depth, unsigned char * pBGRA, bool mirror_input)
{
	GetDepthMap(mirror_input);
	// get depth
	if (depth)
	{
		const int n = dep_height*dep_width;
		for (int i = 0; i < n; i++)
		{
			depth[i] = depth_map[i].depth;
		}
	}

	GetColorMap(mirror_input);
	// get color
	if (pBGRA)
	{
		memset(pBGRA, 0, dep_width*dep_height * 4 * sizeof(char));
		{
			memcpy(pBGRA, image_map, img_width*img_height * 4);
		}// end if not map2depth
		for (int i = 0; i < img_width*img_height; i++)
			pBGRA[i * 4 + 3] = 255;
	}// end if pBGRA
	return 1;
}

int Kinect::GetDepthMap(bool mirror)
{

	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = pNuiSensor->NuiImageStreamGetNextFrame(pDepthStreamHandle, 0, &imageFrame);
	if (hr == E_NUI_FRAME_NO_DATA) {	//	new frame not arrived yet, this is not an error
		return 0;
	}
	ErrorCheck(hr, "error: Microsoft_Kinect::GetDepthMap10: get next frame");
	
	//	---------------	copied from kinect explorer
	BOOL nearMode;
	INuiFrameTexture* pTexture;
	hr = pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if (FAILED(hr)) {
		pNuiSensor->NuiImageStreamReleaseFrame(pDepthStreamHandle, &imageFrame);
		return 0;
	}

	//	---------------	
	NUI_LOCKED_RECT LockedRect;
	hr = pTexture->LockRect(0, &LockedRect, NULL, 0);
	ErrorCheck(hr, "error: Microsoft_Kinect::GetDepthMap10, lock rect");

	errno_t err = memcpy_s(
		depth_map,
		dep_width*dep_height * sizeof(NUI_DEPTH_IMAGE_PIXEL),
		LockedRect.pBits,
		pTexture->BufferLen());

	for (int y = 0; y < dep_height; y++)
	{
		const NUI_DEPTH_IMAGE_PIXEL* pSrc = (const NUI_DEPTH_IMAGE_PIXEL*)LockedRect.pBits + dep_width * y;
		NUI_DEPTH_IMAGE_PIXEL* pDst = depth_map + dep_width * y;
		if (mirror)
			for (int x = 0; x < dep_width; x++)
				pDst[x] = pSrc[dep_width - 1 - x];
		else
			for (int x = 0; x < dep_width; x++)
				pDst[x] = pSrc[x];
	}// end for y

	hr = pTexture->UnlockRect(0);
	ErrorCheck(hr, "error: Microsoft_Kinect::GetDepthMap10, unlock rect");

	pTexture->Release();

	hr = pNuiSensor->NuiImageStreamReleaseFrame(pDepthStreamHandle, &imageFrame);
	ErrorCheck(hr, "error: Microsoft_Kinect::GetDepthMap10, release frame");

	return 1;
}

int Kinect::GetColorMap(bool mirror)
{
	NUI_IMAGE_FRAME imageFrame;

	HRESULT hr = pNuiSensor->NuiImageStreamGetNextFrame(pColorStreamHandle, 0, &imageFrame);
	if (hr == E_NUI_FRAME_NO_DATA) {	//	new frame not arrived yet, this is not an error
		return 0;
	}
	ErrorCheck(hr, "GetColorMap: get next frame");
	
	NUI_LOCKED_RECT LockedRect;
	hr = imageFrame.pFrameTexture->LockRect(0, &LockedRect, NULL, 0);
	ErrorCheck(hr, "GetColorMap: lock rect");

	for (int y = 0; y < dep_height; y++)
	{
		const BGRA32Pixel* pSrc = (const BGRA32Pixel*)LockedRect.pBits + dep_width * y;
		BGRA32Pixel* pDst = image_map + dep_width * y;
		if (mirror)
			for (int x = 0; x < dep_width; x++)
			{
				pDst[x] = pSrc[dep_width - 1 - x];
				std::swap(pDst[x].nRed, pDst[x].nBlue);
			}
		else
			for (int x = 0; x < dep_width; x++)
			{
				pDst[x] = pSrc[x];
				std::swap(pDst[x].nRed, pDst[x].nBlue);
			}
	}// end for y

	hr = imageFrame.pFrameTexture->UnlockRect(0);
	ErrorCheck(hr, "GetColorMap: unlock rect");

	hr = pNuiSensor->NuiImageStreamReleaseFrame(pColorStreamHandle, &imageFrame);
	ErrorCheck(hr, "GetColorMap: release frame");

	return 1;
}

int Kinect::InitKinect()
{
	//	check the number of connected sensor
	int iSensorCount = 0;
	HRESULT hr = NuiGetSensorCount(&iSensorCount);
	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: unable to count connected");

	//	create kinect by given index
	hr = NuiCreateSensorByIndex(0, &pNuiSensor);
	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: unable to connect to kinect by index");

	// Get the status of the sensor, and if connected, then we can initialize it
	hr = pNuiSensor->NuiStatus();
	if (S_OK != hr) {
		pNuiSensor->Release();
		std::cout << "error: MicrosoftKinect::InitKinect10: "
			<< "connection invalid, error code: " << hr << std::endl;
		return 0;
	}

	//	initialize parameters 
	int init_flag = NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH;
	hr = pNuiSensor->NuiInitialize(init_flag);
	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: initialization failed");

	//	create event and streams
	///<	\todo	this part is under construction, currently we just open color and depth stream
	hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	hr = pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextDepthFrameEvent,
		&pDepthStreamHandle);
	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: open depth frame");

	hNextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	hr = pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextColorFrameEvent,
		&pColorStreamHandle);
	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: open color frame");

	// coord mapper

	ErrorCheck(hr, "MicrosoftKinect::InitKinect10: get coord mapper");

	//	---------------------------------------
	//	init class storage
	frame_id = 0;

	dep_width = 640;
	dep_height = 480;
	dep_h_fov = NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV;
	dep_v_fov = NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV;
	depth_map = new NUI_DEPTH_IMAGE_PIXEL[dep_width * dep_height];

	img_width = 640;
	img_height = 480;
	img_h_fov = NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV;
	img_v_fov = NUI_CAMERA_COLOR_NOMINAL_VERTICAL_FOV;
	image_map = new BGRA32Pixel[img_width * img_height];

	//	check alloc space
	if (!depth_map || !image_map) {
		cout << "error: MicrosoftKinect::InitKinect10, alloc kinect storage space failed" << endl;
		exit(0);
	}

	return 1;
}

int Kinect::FreeSpace()
{
	if (NULL != pNuiSensor) {
		pNuiSensor->NuiShutdown();
		pNuiSensor->Release();
	}

	CloseHandle(hNextDepthFrameEvent);
	CloseHandle(hNextColorFrameEvent);
	return 1;
}

void Kinect::ErrorCheck(HRESULT status, std::string str)
{
	if (FAILED(status)) {
		std::cout << "error: MicrosoftKinect, "
			<< str << ", error code: " << std::hex << status << std::endl;
	}
}
