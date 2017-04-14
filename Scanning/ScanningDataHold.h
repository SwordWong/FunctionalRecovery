#pragma once
#include <QImage>
#include <vector>

#include "definations.h"
#include "MicrosoftKinect.h"
#include <Eigen/Dense>
//#include "ldp_basic_vec.h"
const int color_width = 640;
const int color_height = 480;
const int depth_width = 640;
const int depth_height = 480;
class Microsoft_Kinect;
class ScanningDataHolder {
public:
	ScanningDataHolder();
	~ScanningDataHolder();
	static void saveDepth(const std::vector<dfusion::depthtype>& depth_h, std::string filename);
	static void RGBA2QImage(const std::vector<dfusion::PixelRGBA>& color, QImage &img);
	
	void init();
	void getFrame();
	void getDepthShowImage();
	void getNormalShowImage();
	void calVMAP(std::vector< dfusion::depthtype> &depth_map, std::vector<Eigen::Vector3d> &vmap);
	void calNMAP(std::vector<Eigen::Vector3d> &vmap, std::vector<Eigen::Vector3d> &nmap);
public:
	Microsoft_Kinect *m_kinect;
	dfusion::Intr intr;
	std::vector<dfusion::depthtype> m_depth;
	std::vector<dfusion::PixelRGBA> m_color;
	QImage m_color_qimage;
	QImage m_depth_show_image;
	QImage m_normal_show_image;
};
extern ScanningDataHolder scanning_data_holder;