#include "reconstruction.h"
#include "ReconstructionDataHolder.h"
#include "WarpField.h"
#include "GpuGaussNewtonSolver.h"
#include "GpuMesh.h"
#include <qthread.h>
#include <qimage.h>
#include <qdir>
#include <qqueue.h>
#include <qstring.h>
#include <qfile.h>
#include <qtextstream.h>
#include <qcolor.h>
#include <qmutex.h>
#include "TsdfVolume.h"
#include "BlendColorMap.h"
QString defaultColorListFile = "../data/colorList.dat";
bool removeFolderContent(const QString &folderDir)
{
	QDir dir(folderDir);
	QFileInfoList fileList;
	QFileInfo curFile;
	if (!dir.exists()) { return false; }
	fileList = dir.entryInfoList(QDir::Dirs | QDir::Files
		| QDir::Readable | QDir::Writable
		| QDir::Hidden | QDir::NoDotAndDotDot
		, QDir::Name);
	while (fileList.size() > 0)
	{
		int infoNum = fileList.size();
		for (int i = infoNum - 1; i >= 0; i--)
		{
			curFile = fileList[i];
			if (curFile.isFile())
			{
				QFile fileTemp(curFile.filePath());
				fileTemp.remove();
				fileList.removeAt(i);
			}
			if (curFile.isDir())
			{
				QDir dirTemp(curFile.filePath());
				QFileInfoList fileList1 = dirTemp.entryInfoList(QDir::Dirs | QDir::Files
					| QDir::Readable | QDir::Writable
					| QDir::Hidden | QDir::NoDotAndDotDot
					, QDir::Name);
				if (fileList1.size() == 0)
				{
					dirTemp.rmdir(".");
					fileList.removeAt(i);
				}
				else
				{
					for (int j = 0; j < fileList1.size(); j++)
					{
						if (!(fileList.contains(fileList1[j])))
							fileList.append(fileList1[j]);
					}
				}
			}
		}
	}
	return true;
}
QMutex* g_mutex;
QMutex* g_mutex_clear;
class SaveThread_Recon : public QThread
{
public:
	SaveThread_Recon() :QThread()
	{
		m_depths.reserve(reconstruction_dataholder.m_dparam.fusion_dumping_max_frame);
		m_ids.reserve(reconstruction_dataholder.m_dparam.fusion_dumping_max_frame);
		flag_run = true;
	}
	void set_path(QString path)
	{
		QMutexLocker locker(g_mutex);
		m_currentPath = path;
		QDir dir(m_currentPath);
		if (!dir.exists())
			mkdir(m_currentPath.toStdString());
		reconstruction_dataholder.m_dparam.save(fullfile(m_currentPath.toStdString(), "_param.param.txt").c_str());
	}
	void push_depth(const std::vector<dfusion::depthtype>& depth,
		const std::vector<dfusion::PixelRGBA>* color, int id, const FrameInfoRecon &info
		,LARGE_INTEGER depth_timestamp, LARGE_INTEGER color_timestamp
	)
	{
		QMutexLocker locker(g_mutex);
		m_depths.push_back(depth);
		m_ids.push_back(id);
		m_infos.push_back(info);
		m_depth_timestamps.push_back(depth_timestamp);
		m_color_timestamps.push_back(color_timestamp);
		if (color)
		{
			m_colors.push_back(QImage(dfusion::KINECT_WIDTH,
				dfusion::KINECT_HEIGHT, QImage::Format_RGBA8888));
			const dfusion::PixelRGBA* src = color->data();
			QImage& img = m_colors.back();
			for (int y = 0; y < dfusion::KINECT_HEIGHT; y++)
				for (int x = 0; x < dfusion::KINECT_WIDTH; x++)
				{
					dfusion::PixelRGBA p = src[y*dfusion::KINECT_WIDTH + x];
					img.setPixel(x, y, qRgba(p.r, p.g, p.b, p.a));
				}
		}
	}
	void clear_data()
	{
		m_depths.clear();
		m_colors.clear();
		m_ids.clear();
		m_infos.clear();
		m_depth_timestamps.clear();
		m_color_timestamps.clear();
	}
	void set_flag_run(bool flag)
	{
		flag_run = flag;
	}
protected:
	void run()
	{
		while (1)
		{
			QMutexLocker locker(g_mutex_clear);
			if (!m_depths.empty() && flag_run)
			{
				
				//printf("saving m_depth.size() = %d\n", m_depths.size());
				int id = 0;
				const std::vector<dfusion::depthtype>* depth = nullptr;
				FrameInfoRecon info;
				const QImage* color = nullptr;
				{
					QMutexLocker locker(g_mutex);
					id = m_ids.front();
					depth = &m_depths.front();
					info = m_infos.front();
					if (m_colors.size())
						color = &m_colors.front();
					//printf("get data\n");
				}

				QDir dir(m_currentPath);
				QString name = dir.absoluteFilePath(QString().sprintf("%08d.depth", id));
				reconstruction_dataholder.saveDepth(*depth, name.toStdString());
				//printf("saved: %s\n", name.toStdString().c_str());
				name = dir.absoluteFilePath(QString().sprintf("%08d.info", id));
				info.save(name);
				LARGE_INTEGER timestamp;
				name = dir.absoluteFilePath(QString().sprintf("depth%08d.timestamp", id));
				timestamp = m_depth_timestamps.front();
				reconstruction_dataholder.saveTimestamp(timestamp, name.toStdString().c_str());
				//printf("saved: %s\n", name.toStdString().c_str());

				name = dir.absoluteFilePath(QString().sprintf("color%08d.timestamp", id));
				timestamp = m_color_timestamps.front();
				reconstruction_dataholder.saveTimestamp(timestamp, name.toStdString().c_str());
				//printf("saved: %s\n", name.toStdString().c_str());
				if (color)
				{
					name = dir.absoluteFilePath(QString().sprintf("%08d.png", id));
					color->save(name);
					//printf("saved: %s\n", name.toStdString().c_str());
				}
				name = dir.absoluteFilePath(QString().sprintf("%08d.png", id));
				{
					QMutexLocker locker(g_mutex);
					m_ids.pop_front();
					m_depths.pop_front();
					m_infos.pop_front();
					if (m_colors.size())
						m_colors.pop_front();
					//printf("pop data\n");
				}
				printf("frame%08d saved\n", id);
			}
			
		}
	}

private:
	QQueue<std::vector<dfusion::depthtype>> m_depths;
	QQueue<QImage> m_colors;
	QQueue<int> m_ids;
	QQueue<FrameInfoRecon> m_infos;
	QQueue<LARGE_INTEGER> m_depth_timestamps;
	QQueue<LARGE_INTEGER> m_color_timestamps;
	QString m_currentPath;
	bool flag_run;
};
SaveThread_Recon g_saveThread;
Reconstruction::Reconstruction(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	setAcceptDrops(true);
	m_frameIndex = 0;
	//m_view_normalmap = false;
	m_currentPath = "../data/frame";
	m_lastState = Reconstruction::Live;
	m_state = Reconstruction::Live;
	//frameByFrame = false;
	if (reconstruction_dataholder.m_dparam.fusion_loading_mode)
		m_state = Reconstruction::Pause;

	reconstruction_dataholder.init();
	connectEvent();
	m_fpsTimerId = startTimer(30);
	m_autoResetTimerId = startTimer(reconstruction_dataholder.m_dparam.view_autoreset_seconds * 1000);
	initUIParam();
	//updateUIParam();
	g_mutex = new QMutex();
	g_mutex_clear = new QMutex();
	readColorList(defaultColorListFile);
	//g_dataholder.setUI(&ui);
	g_saveThread.set_path(m_currentPath);
	g_saveThread.start();
}

Reconstruction::~Reconstruction()
{
	g_saveThread.terminate();
	delete g_mutex;
	delete g_mutex_clear;
}
void Reconstruction::connectEvent()
{
	connect(ui.rb_mode_live, SIGNAL(clicked()), this, SLOT(clicked_action_rb_live()));
	connect(ui.rb_mode_live_saving, SIGNAL(clicked()), this, SLOT(clicked_action_rb_live_saving()));
	connect(ui.rb_mode_loading, SIGNAL(clicked()), this, SLOT(clicked_action_rb_loading()));
	connect(ui.pb_save_mesh, SIGNAL(clicked()), this, SLOT(clicked_action_pb_save_mesh()));
	connect(ui.pb_play_pause, SIGNAL(clicked()), this, SLOT(clicked_action_pb_play_pause()));
	connect(ui.pb_reset, SIGNAL(clicked()), this, SLOT(resetVolume()));
	connect(ui.pb_load_color_list, SIGNAL(clicked()), this, SLOT(clicked_action_pb_load_color_list()));
}
void Reconstruction::timerEvent(QTimerEvent* ev)
{
	if (m_fpsTimerId == ev->timerId())
	{
		gtime_t time_s = gtime_now();
		bool suc;
		try
		{
			//// process a new kinect frame.
			switch (m_state)
			{
			case Reconstruction::Loading:
				frameLoading();
				break;
			case Reconstruction::Saving:
			case Reconstruction::LiveSaving:
				frameLive();
				break;
			case Reconstruction::Live:
				frameLive();
				break;
			default:
				break;
			}

			//// process viewers
			switch (m_state)
			{
			case Reconstruction::Live:
			case Reconstruction::Loading:
				//case DFdeform::Pause:
			case Reconstruction::LiveSaving:
				updateKinectFusion(suc);
				break;
			default:
				break;
			}
			Camera cam;
			frame_info.icp_suc = suc;
			frame_info.rigid_transform = reconstruction_dataholder.m_processor.getRigidTransform();
			if( m_state == Reconstruction::LiveSaving || m_state == Reconstruction::Saving)
				frameSaving();
			// warped view
			ui.widgetRecon->getCameraInfo(cam);
			cam.setViewPort(0, ui.widgetRecon->width(), 0, ui.widgetRecon->height());
			reconstruction_dataholder.m_processor.shading(cam, reconstruction_dataholder.m_lights,
				reconstruction_dataholder.m_warpedview_shading, false);
			
			{
				ui.widgetRecon->setRayCastingShadingImage(reconstruction_dataholder.m_warpedview_shading);
			}		
			if (reconstruction_dataholder.m_processor.hasRawDepth() && m_state != Saving)
			{
				const dfusion::MapArr& nmap = reconstruction_dataholder.m_processor.getRawDepthNormal();
				ui.widgetDepth->setNormal_d(nmap);
			}
			else
				ui.widgetDepth->setImage_d(reconstruction_dataholder.m_depth_d);
			ui.widgetColor->setRayCastingShadingImage(reconstruction_dataholder.m_color_d);
			
		}
		catch (std::exception e)
		{
			std::cout << e.what() << std::endl;
		}

		gtime_t time_e = gtime_now();
		double sec = gtime_seconds(time_s, time_e);
		double fps = 1.0 / sec;
		if (m_state == Pause)
			return;
		if(m_state != Reconstruction::Loading )
		//if (suc)
			m_frameIndex++;
		setWindowTitle(QString().sprintf("[%d] FPS:%.1f",m_frameIndex, fps));
	}// end if fps timer id
}
void Reconstruction::frameLoading()
{
	QDir dir(m_currentPath);
	if (!dir.exists())
		throw std::exception(("error input path:" + m_currentPath.toStdString()).c_str());
	QString name = dir.absoluteFilePath(QString().sprintf("%08d.depth", m_frameIndex));

	try
	{
		reconstruction_dataholder.loadDepth(reconstruction_dataholder.m_depth_h, name.toStdString());

		name = dir.absoluteFilePath(QString().sprintf("%08d.png", m_frameIndex));
		QImage img(name);
		if (!img.isNull())
		{
			for (int y = 0; y < dfusion::KINECT_HEIGHT; y++)
				for (int x = 0; x < dfusion::KINECT_WIDTH; x++)
				{
					QRgb p = img.pixel(x, y);
					dfusion::PixelRGBA& d = reconstruction_dataholder.m_color_h[y*dfusion::KINECT_WIDTH + x];
					d.r = qRed(p);
					d.g = qGreen(p);
					d.b = qBlue(p);
					d.a = qAlpha(p);
				}
		}
		std::vector<dfusion::depthtype> depth_filtered;
		std::vector<dfusion::PixelRGBA> color_filtered;
		depth_filtered.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
		color_filtered.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
		depth_filtered = reconstruction_dataholder.m_depth_h;
		//color_filtered = g_dataholder.m_color_h;
		//reconstruction_dataholder.m_kinect.mapColor2DepthWithFilter(depth_filtered.data(), 
		//	(uchar*)color_filtered.data(), (uchar*)reconstruction_dataholder.m_color_h.data(), reconstruction_dataholder.handPixel
		////, ui.cb_color_filter->isChecked()
		//);
		reconstruction_dataholder.m_kinect.mapColor2Depth_GPU(depth_filtered.data(), (uchar*)color_filtered.data()
			, (uchar*)reconstruction_dataholder.m_color_h.data(), NULL
			, ui.cb_color_filter->isChecked()
		);
		//g_dataholder.m_kinect.colorFilter(depth_filtered.data(), (uchar*)color_filtered.data(), dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT);
#ifdef ENABLE_COLOR_FUSION



#endif
		reconstruction_dataholder.m_color_d.upload(color_filtered.data(),
			dfusion::KINECT_WIDTH * sizeof(dfusion::PixelRGBA),
			dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
		reconstruction_dataholder.m_depth_d.upload(depth_filtered.data(),
			dfusion::KINECT_WIDTH * sizeof(dfusion::depthtype),
			dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
		name = dir.absoluteFilePath(QString().sprintf("%08d.info", m_frameIndex));
		frame_info.load(name);
		//reconstruction_dataholder.setFlagFusion(frame_info.fusion);
	}
	catch (std::exception e)
	{
		setState(Pause);
		std::cout << e.what() << std::endl;
		//m_frameIndex += reconstruction_dataholder.m_dparam.load_frameIndx_plus_num;
	}
	if (m_state != Pause)
		m_frameIndex += 1;// reconstruction_dataholder.m_dparam.load_frameIndx_plus_num;
}

void Reconstruction::updateKinectFusion(bool &suc)
{
	if (m_state != Reconstruction::Pause)
		reconstruction_dataholder.m_processor.processFrame(reconstruction_dataholder.m_depth_d,
			reconstruction_dataholder.m_color_d,suc);

	
}
void Reconstruction::frameSaving()
{
	const std::vector<dfusion::PixelRGBA>* cl = nullptr;
#ifdef ENABLE_COLOR_FUSION

#endif
	cl = &reconstruction_dataholder.m_color_h;
	//frame_info.fusion = reconstruction_dataholder.m_processor.getFlagFusion();
	g_saveThread.push_depth(reconstruction_dataholder.m_depth_h, cl, m_frameIndex, frame_info
		,reconstruction_dataholder.m_kinect.depth_timestamp
		,reconstruction_dataholder.m_kinect.color_timestamp
		);

}

void Reconstruction::frameLive()
{
	reconstruction_dataholder.m_kinect.GetDepthColorIntoBuffer(reconstruction_dataholder.m_depth_h.data(),
		(uchar*)reconstruction_dataholder.m_color_h.data(), false, reconstruction_dataholder.m_dparam.mirror_input);
	std::vector<dfusion::depthtype> depth_filtered;
	std::vector<dfusion::PixelRGBA> color_filtered;
	depth_filtered.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	color_filtered.resize(dfusion::KINECT_WIDTH*dfusion::KINECT_HEIGHT);
	//color_filtered = reconstruction_dataholder.m_color_h;
	depth_filtered = reconstruction_dataholder.m_depth_h;
	gtime_t time1 = gtime_now();
	//if(ui.cb_color_filter->isChecked())
		/*reconstruction_dataholder.m_kinect.mapColor2DepthWithFilter(depth_filtered.data(), (uchar*)color_filtered.data()
			, (uchar*)reconstruction_dataholder.m_color_h.data(), reconstruction_dataholder.handPixel
			, ui.cb_color_filter->isChecked()
		);*/
	reconstruction_dataholder.m_kinect.mapColor2Depth_GPU(depth_filtered.data(), (uchar*)color_filtered.data()
		, (uchar*)reconstruction_dataholder.m_color_h.data(), NULL
		, ui.cb_color_filter->isChecked()
	);
	gtime_t time2 = gtime_now();
	printf("", gtime_seconds(time1, time2));
	//printf("color_list  size = %d \n", reconstruction_dataholder.m_kinect.colorList.size());
	//else
	//{
	//	
	//	reconstruction_dataholder.m_kinect.mapColor2Depth(depth_filtered.data(), (uchar*)color_filtered.data());
	//		//, reconstruction_dataholder.m_depth_h.data());
	//}
		
	//g_dataholder.m_kinect.colorFilter(depth_filtered.data(), (uchar*)color_filtered.data(), dfusion::KINECT_WIDTH, dfusion::KINECT_HEIGHT);
	reconstruction_dataholder.m_depth_d.upload(depth_filtered.data(), dfusion::KINECT_WIDTH * sizeof(dfusion::depthtype),
		dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);
	reconstruction_dataholder.m_color_d.upload(color_filtered.data(), dfusion::KINECT_WIDTH * sizeof(dfusion::PixelRGBA),
		dfusion::KINECT_HEIGHT, dfusion::KINECT_WIDTH);

}
void Reconstruction::resetVolume()
{
	reconstruction_dataholder.m_processor.updateParam(reconstruction_dataholder.m_dparam);
	/*if (g_dataholder.m_processor.getVolume())
	{
	dfusion::TsdfVolume *p;
	p = g_dataholder.m_processor.getVolume();
	delete p;
	p = nullptr;
	}*/
	reconstruction_dataholder.m_processor.init(reconstruction_dataholder.m_dparam);

	//g_dataholder.m_processor.reset();
	
	std::cout << "resetVolume" << std::endl;
	
	if (m_state == LiveSaving)
	{
		//QMutexLocker locker(g_mutex);
		//g_saveThread.set_flag_run(false);
		//g_saveThread.wait();
		QMutexLocker locker(g_mutex_clear);
		g_saveThread.clear_data();
		std::cout << "clear save list" << std::endl;
		removeFolderContent(m_currentPath);
		std::cout << "remove files" << std::endl;
	}
	m_frameIndex = 0;
}
void Reconstruction::readColorList(QString filename)
{
	//QFile file("../data/colorList.dat");

	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qDebug() << "Can't open the file!" << endl;
		return;
	}
	QTextStream in(&file);
	int n;
	int r, g, b;
	in >> n;
	RGB24Pixel rgb;
	reconstruction_dataholder.m_kinect.colorList.clear();
	for (int i = 0; i < n; i++)
	{
		in >> r >> g >> b;
		rgb.nRed = r;
		rgb.nGreen = g;
		rgb.nBlue = b;
		reconstruction_dataholder.m_kinect.colorList.push_back(rgb);
	}
	/*reconstruction_dataholder.m_kinect.colorList.clear();
	rgb.nRed = 0;
	rgb.nGreen = 0;
	rgb.nBlue = 255;
	reconstruction_dataholder.m_kinect.colorList.push_back(rgb);*/

	file.close();
	//updateColorLabel();
}
void Reconstruction:: saveVNMAPasObj()
{
	const std::vector<dfusion::MapArr>& d_vmap_pyd = reconstruction_dataholder.m_processor.getCurrVMAPpyd();
	const std::vector<dfusion::MapArr>& d_nmap_pyd = reconstruction_dataholder.m_processor.getCurrNMAPpyd();
	std::vector<float4> vlist;
	std::vector<float4> nlist;
	int num;
	ObjMesh mesh;
	char filename[100];
	for (int i = 0; i < 4; i++)
	{
		mesh.clear();
		num = d_vmap_pyd[i].cols()*d_vmap_pyd[i].rows();
		
		vlist.resize(num);
		nlist.resize(num);

		d_vmap_pyd[i].download(vlist.data(), d_vmap_pyd[i].cols() * sizeof(float4));
		d_nmap_pyd[i].download(nlist.data(), d_vmap_pyd[i].cols() * sizeof(float4));
		

		for (int i_v = 0; i_v < num; i_v++)
		{
			float4 vertex_point, normal_point;
			vertex_point = vlist[i_v];
			normal_point = nlist[i_v];
			mesh.vertex_list.push_back(Float3(vertex_point.x, vertex_point.y, vertex_point.z));
			mesh.vertex_list.push_back(Float3(normal_point.x, normal_point.y, normal_point.z));
		}

		sprintf(filename, "depth_lel%d.obj", i);
		mesh.saveObj(filename);
	}
	
}
void Reconstruction::saveCanoMesh(QString name, bool postProcess)
{

	try
	{
		if (!name.isEmpty())
		{
			if (!name.endsWith(".obj"))
				name.append(".obj");

			ObjMesh mesh;
			reconstruction_dataholder.m_processor.getCanoMesh()->toObjMesh(mesh);
			QString name_tmp = name;
			name_tmp.insert(name_tmp.length() - 4, "_tmp");
			mesh.saveObj(name_tmp.toStdString().c_str());

			meshlabScript mlScript;
			mlScript.mergeCloseVetices = ui.cb_remove_dup->isChecked();
			mlScript.removeIsoFace = ui.cb_remove_iso->isChecked();
			mlScript.quadEdgeCollapse = ui.cb_edge_collapse->isChecked();
			QString name_script = "PostProcess.mlx";
			mlScript.generateScript(name_script);
			char commond_str[200];
			sprintf(commond_str, "C:/\"Program Files\"/VCG/MeshLab/meshlabserver.exe -i %s -o %s -s %s\n", name_tmp.toStdString().c_str(), name.toStdString().c_str(), name_script.toStdString().c_str());
			system(commond_str);
		}

	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}
void Reconstruction::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_R)
	{
		resetVolume();
	}
	else if (event->key() == Qt::Key_S)
	{
		saveVNMAPasObj();
	}
	
}
void Reconstruction::clicked_action_rb_live()
{
	setState(Live);
	resetVolume();
	updateUIParam();
}
void Reconstruction::clicked_action_rb_live_saving()
{
	setState(LiveSaving);
	resetVolume();
	m_frameIndex = 0;
	updateUIParam();
}
void Reconstruction::clicked_action_rb_loading()
{
	setState(Loading);
	resetVolume();
	m_frameIndex = 0;
	updateUIParam();
}
void Reconstruction::clicked_action_pb_save_mesh()
{
	QString name = QFileDialog::getSaveFileName(this, "save mesh", "", ".obj");
	saveCanoMesh(name);
}
void Reconstruction::clicked_action_pb_play_pause()
{
	if (m_state == Pause)
		restoreState();
	else
		setState(Pause);
}
void Reconstruction::clicked_action_pb_load_color_list()
{
	QString file = QFileDialog::getOpenFileName(0, tr("load ColorList"),
		tr("."), tr("*.dat"));

	if (file.isEmpty())
		return;

	try
	{
		readColorList(file);
	}
	catch (std::string s)
	{
		std::cout << s << std::endl;
	}
	catch (std::exception e)
	{
		std::cout << e.what() << std::endl;
	}
}
void Reconstruction::initUIParam()
{
	ui.cb_remove_dup->setChecked(true);
	ui.cb_remove_iso->setChecked(true);
	ui.cb_edge_collapse->setChecked(true);
	ui.cb_color_filter->setChecked(true);
}
void Reconstruction::updateUIParam()
{
	
	switch (m_state)
	{
	case Reconstruction::Loading:
		ui.rb_mode_loading->setChecked(true);
		break;
	case Reconstruction::LiveSaving:
		ui.rb_mode_live_saving->setChecked(true);
		break;
	case Reconstruction::Live:
		ui.rb_mode_live->setChecked(true);
		break;
	default:
		break;
	}
}