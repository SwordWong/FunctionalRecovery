#include "ReconstructionDataHolder.h"
#include <fstream>
//#include "WarpField.h"

#include "meshAlignByMarker.h"


#define	OUTPUT_TMP_MESH 1
#define	OUTPUT_RENDERED_MESH 0
#define SET_KEYFRAME_HAND_COLOR 0
#define OUTPUT_MAPS 1
#define ALIGN_KEYFRAME_TO_BASE 0
ReconstructionDataHolder reconstruction_dataholder;
using namespace ldp;
//typedef Ui::DFdeformClass DFdeformClass;

void ReconstructionDataHolder::init()
{
	m_kinect.InitKinect();
	m_depth_h.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	m_color_h.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	m_depth_d.create(dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
	m_color_d.create(dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT);

	m_lights.pos = make_float3(0, 0, 0);
	m_lights.amb = make_float3(0.3, 0.3, 0.3);
	m_lights.diffuse = make_float3(0.8, 0.8, 0.8);
	m_lights.spec = make_float3(0, 0, 0);


	m_processor.init(m_dparam);

}
void ReconstructionDataHolder::saveDepth(const std::vector<dfusion::depthtype>& depth_h, std::string filename)
{
	if (depth_h.size() != dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT)
		throw std::exception("saveDepth: size not matched!");

	std::ofstream stm(filename, std::ios_base::binary);
	if (stm.fail())
		throw std::exception(("save failed: " + filename).c_str());

	std::vector<unsigned short> tmp(depth_h.size());
	for (size_t i = 0; i < tmp.size(); i++)
		tmp[i] = depth_h[i];

	int w = dfusion::KINECT_WIDTH;
	int h = dfusion::KINECT_HEIGHT;
	stm.write((const char*)&w, sizeof(int));
	stm.write((const char*)&h, sizeof(int));
	stm.write((const char*)tmp.data(), tmp.size() * sizeof(unsigned short));

	stm.close();
}

void ReconstructionDataHolder::loadDepth(std::vector<dfusion::depthtype>& depth_h, std::string filename)
{
	std::ifstream stm(filename, std::ios_base::binary);
	if (stm.fail())
		throw std::exception(("load failed: " + filename).c_str());

	depth_h.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	std::vector<unsigned short> tmp(depth_h.size());

	int w = 0, h = 0;
	stm.read((char*)&w, sizeof(int));
	stm.read((char*)&h, sizeof(int));
	if (w != dfusion::KINECT_WIDTH || h != dfusion::KINECT_HEIGHT)
		throw std::exception("loadDepth: size not matched!");
	stm.read((char*)tmp.data(), tmp.size() * sizeof(unsigned short));

	for (size_t i = 0; i < tmp.size(); i++)
		depth_h[i] = tmp[i];

	//debug
	//for (int y = 0; y < dfusion::KINECT_HEIGHT; y++)
	//for (int x = 0; x < dfusion::KINECT_WIDTH; x++)
	//	depth_h[y*dfusion::KINECT_WIDTH + x] = tmp[y*dfusion::KINECT_WIDTH + dfusion::KINECT_WIDTH-1-x];

	stm.close();
}