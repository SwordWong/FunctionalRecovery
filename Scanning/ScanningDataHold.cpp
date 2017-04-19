#include "ScanningDataHold.h"
#include <fstream>
#include <Eigen/Dense>
#include <QByteArray>
#include "kinect_util.h"
//#include "Depth2PointCloud.h"
#define PI 3.141592653589793f
ScanningDataHolder scanning_data_holder;
ScanningDataHolder::ScanningDataHolder()
{

}
ScanningDataHolder::~ScanningDataHolder()
{
	depth_map_d.release();
	nmap_d.release();
	vmap_d.release();
	color_map_d.release();
}
void ScanningDataHolder::saveDepth(const std::vector<dfusion::depthtype>& depth_h, std::string filename)
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

void ScanningDataHolder::RGBA2QImage(const std::vector<dfusion::PixelRGBA>& color, QImage &img)
{
	const dfusion::PixelRGBA* src = color.data();
	for (int y = 0; y < img.height(); y++)
		for (int x = 0; x < img.width(); x++)
		{
			dfusion::PixelRGBA p = src[y*img.width() + x];
			img.setPixel(x, y, qRgba(p.r, p.g, p.b, p.a));
		}
}

void ScanningDataHolder::init()
{
	
	m_kinect.InitKinect();
	
	const float	f = dfusion::KINECT_HEIGHT* 0.5f / tanf(KINECT_DEPTH_V_FOV * PI / 180.0f * 0.5f);
	intr = dfusion::Intr(f, f, (1 + dfusion::KINECT_WIDTH) / 2, dfusion::KINECT_HEIGHT / 2);

	m_depth.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	m_color.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	m_color_qimage = QImage(dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);
	m_depth_show_image = QImage(dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);
	m_normal_show_image = QImage(dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);


	depth_map_d.create(dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
	color_map_d.create(dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
}

void ScanningDataHolder::getFrame()
{
	m_kinect.GetDepthColorIntoBuffer(m_depth.data(), (uchar*)m_color.data(), true);
	RGBA2QImage(m_color, m_color_qimage);
}

void ScanningDataHolder::getDepthShowImage()
{

	
	
	int min_depth = 300;
	int max_depth = 3000;
	for (int y = 0; y < m_depth_show_image.height(); y++)
		for (int x = 0; x < m_depth_show_image.width(); x++)
		{
			int i_pixel = y*m_depth_show_image.width() + x;
			float val = (float)(m_depth[i_pixel] - min_depth) / (max_depth - min_depth);
			QRgb c = qRgba(
				min(val, 1.0) * 255,
				min(val, 1.0) * 255,
				min(val, 1.0) * 255,
				255
			);
			m_depth_show_image.setPixel(x, y, c);
		}
}

void ScanningDataHolder::getNormalShowImage()
{
	
	
	
	depth_map_d.upload(m_depth.data(), dfusion::KINECT_WIDTH * sizeof(dfusion::depthtype),
		dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
	dfusion::createVMap(intr, depth_map_d, vmap_d);
	dfusion::createNMap(vmap_d, nmap_d);

	std::vector<float4> vlist(dfusion::KINECT_HEIGHT*dfusion::KINECT_WIDTH);
	std::vector<float4> nlist(dfusion::KINECT_HEIGHT*dfusion::KINECT_WIDTH);

	std::vector<dfusion::PixelRGBA> normal_image(dfusion::KINECT_HEIGHT*dfusion::KINECT_WIDTH);

	dfusion::generateNormalMap(nmap_d, color_map_d);
	color_map_d.download(normal_image.data(), dfusion::KINECT_WIDTH * sizeof(dfusion::PixelRGBA));
	//QByteArray qba_normal_image(normal_image.data(), sizeof(normal_image));
	//m_normal_show_image.loadFromData((char*)normal_image.data());
	
	for (int y = 0; y < m_normal_show_image.height(); y++)
		for (int x = 0; x < m_normal_show_image.width(); x++)
		{
			int i_pixel = y*m_normal_show_image.width() + x;
			QRgb c;
			c = qRgba(
				normal_image[i_pixel].r,
				normal_image[i_pixel].g,
				normal_image[i_pixel].b,
				normal_image[i_pixel].a
			);
			m_normal_show_image.setPixel(x, y, c);
		}
	
	
	/*std::vector<Eigen::Vector3d> vmap;
	std::vector<Eigen::Vector3d> nmap;
	calVMAP(m_depth, vmap);
	calNMAP(vmap, nmap);
	for (int y = 0; y < m_normal_show_image.height(); y++)
		for (int x = 0; x < m_normal_show_image.width(); x++)
		{
			int i_pixel = y*m_normal_show_image.width() + x;
			QRgb c;
			if (!isnan(nmap[i_pixel].x())) {
				c = qRgba(0, 0, 0, 0);
			}
			else {
				c = qRgba(
					nmap[i_pixel].x() * 255,
					nmap[i_pixel].y() * 255,
					nmap[i_pixel].z() * 255,
					255
				);
			}
			m_normal_show_image.setPixel(x, y, c);
		}*/
}

void ScanningDataHolder::calVMAP(std::vector< dfusion::depthtype> &depth_map, std::vector<Eigen::Vector3d> &vmap)
{
	int index;
	vmap.resize(dfusion::KINECT_WIDTH* dfusion::KINECT_HEIGHT);
	for (int row = 0; row < dfusion::KINECT_HEIGHT; row++)
	{
		for (int col = 0; col < dfusion::KINECT_WIDTH; col++)
		{
			index = row*dfusion::KINECT_WIDTH + col;
			float z = depth_map[index] * 0.001f;


			if (z > KINECT_NEAREST_METER)
			{
				float3 xyz = intr.uvd2xyz(col, row, z);
				vmap[index] = Eigen::Vector3d(xyz.x, xyz.y, xyz.z);
			}
			else
			{
				vmap[index] = Eigen::Vector3d(NAN, NAN, NAN);
			}
		}
	}
}

void ScanningDataHolder::calNMAP(std::vector<Eigen::Vector3d>& vmap, std::vector<Eigen::Vector3d>& nmap)
{
	int index;
	QImage normal_image(dfusion::KINECT_WIDTH,
		dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888);
	nmap.resize(dfusion::KINECT_WIDTH* dfusion::KINECT_HEIGHT);
	for (int row = 0; row < dfusion::KINECT_HEIGHT; row++)
	{
		for (int col = 0; col < dfusion::KINECT_WIDTH; col++)
		{
			index = row*dfusion::KINECT_WIDTH + col;

			if (col == dfusion::KINECT_WIDTH - 1 || col == 0
				|| row == dfusion::KINECT_HEIGHT - 1 || row == 0)
			{
				nmap[index] = Eigen::Vector3d(NAN, NAN, NAN);
			}
			else
			{
				Eigen::Vector3d v00, v01, v10;
				Eigen::Vector3d v_neig[8];
				Eigen::Vector3d normal_acc = Eigen::Vector3d(0, 0, 0);
				v00 = vmap[row*dfusion::KINECT_WIDTH + col];
				v_neig[0] = vmap[row*dfusion::KINECT_WIDTH + col + 1];
				v_neig[1] = vmap[(row - 1)*dfusion::KINECT_WIDTH + col + 1];
				v_neig[2] = vmap[(row - 1)*dfusion::KINECT_WIDTH + col];
				v_neig[3] = vmap[(row - 1)*dfusion::KINECT_WIDTH + col - 1];
				v_neig[4] = vmap[row*dfusion::KINECT_WIDTH + col - 1];
				v_neig[5] = vmap[(row + 1)*dfusion::KINECT_WIDTH + col - 1];
				v_neig[6] = vmap[(row + 1)*dfusion::KINECT_WIDTH + col];
				v_neig[7] = vmap[(row + 1)*dfusion::KINECT_WIDTH + col + 1];
				int n_valid = 0;
				for (int i = 0; i < 8; i++)
				{
					Eigen::Vector3d v_0, v_1;
					Eigen::Vector3d g_0, g_1;

					v_0 = v_neig[i];
					v_1 = v_neig[(i + 1) % 8];

					if (!isnan(v00.x()) && !isnan(v_0.x()) && !isnan(v_1.x()))
					{
						g_0 = v_0 - v00;
						g_1 = v_1 - v00;

						Eigen::Vector3d r = g_0.cross(g_1);
						r.normalize();
						normal_acc += r;
						n_valid++;
					}
				}
				if (n_valid > 0)
				{
					normal_acc.normalize();
					nmap[index] = normal_acc;
					int r, g, b, a;
					r = max(0, min(255, int(255 * (nmap[index].x() * 0.5f + 0.5f))));
					g = max(0, min(255, int(255 * (nmap[index].y() * 0.5f + 0.5f))));
					b = max(0, min(255, int(255 * (nmap[index].z() * 0.5f + 0.5f))));
					a = 255;

					normal_image.setPixel(col, row, qRgba(r, g, b, a));
				}
				else
					nmap[index] = Eigen::Vector3d(NAN, NAN, NAN);
			}

		}
	}
}



