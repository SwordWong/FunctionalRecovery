#pragma once
#include "util.h"
#include "ObjMesh.h"
#include "mpu\VolumeData.h"
#include "MicrosoftKinect.h"
#include "kinect_util.h"
#include "TsdfVolume.h"
#include "RayCaster.h"
#include "MarchingCubes.h"
#include "GpuMesh.h"
#include "KinectFusionProcessor.h"
#include "GpuKdTree.h"
#include "BoneMesh.h"
#include <set>
#include <hash_set>

class ReconstructionDataHolder
{
public:

	enum ShowType
	{
		BlendShape = 0,
		PCA,
		RegionBlendShape,
		RegionPCA,
		MeshIK,
		ShowTypeEnd
	};

	void init();

	//inline void setUI(Ui_DFdeformClass *ui) { this->ui = ui; }
	static void saveDepth(const std::vector<dfusion::depthtype>& depth_h, std::string filename);
	static void loadDepth(std::vector<dfusion::depthtype>& depth_h, std::string filename);

	static void saveTimestamp(LARGE_INTEGER timestamp, std::string filename);
public:
	//Ui_DFdeformClass* ui;
	dfusion::KinectFusionProcessor m_processor;
	Microsoft_Kinect m_kinect;
	dfusion::Param m_dparam;
	dfusion::LightSource m_lights;

	std::vector<dfusion::depthtype> m_depth_h;
	std::vector<dfusion::PixelRGBA> m_color_h;

	dfusion::DepthMap m_depth_d;
	dfusion::ColorMap m_color_d;
	dfusion::ColorMap m_warpedview_shading;
	dfusion::ColorMap m_canoview_shading;
	dfusion::ColorMap m_errorMap_shading;
	dfusion::ColorMap m_warpedview_shading_with_colorImage;

	std::set<PixelPos> handPixel;
	// the following is used for debugging/visualizing loaded volumes.
	//dfusion::RayCaster m_rayCaster;
	//dfusion::MarchingCubes m_marchCube;
	//dfusion::TsdfVolume m_volume;
	//dfusion::GpuMesh m_mesh;

	//dfusion::ColorMap m_warpedview_shading;
private:
	mutable ldp::TimeStamp m_timeStamp;
};

extern ReconstructionDataHolder reconstruction_dataholder;
