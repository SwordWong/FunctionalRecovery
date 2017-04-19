#include "scanning.h"
#include <QDir>
#include <QQueue>
#include <QString>
#include <QFile>
#include <QTextstream>
#include <QMutex>
#include <QThread>
#include <QPixmap>
#include "definations.h"
#include "../algorithm/util.h"
#include "ScanningDataHold.h"
#include "ldpdef.h"
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
		m_depths.reserve(10000);
		m_ids.reserve(10000);
		flag_run = true;
	}
	void set_path(QString path)
	{
		QMutexLocker locker(g_mutex);
		m_currentPath = path;
		QDir dir(m_currentPath);
		if (!dir.exists())
			ldp::mkdir(m_currentPath.toStdString());
	}
	void push_depth(const std::vector<dfusion::depthtype>& depth,
		const QImage& color, int id
	)
	{
		QMutexLocker locker(g_mutex);
		m_colors.push_back(color);
		m_depths.push_back(depth);
		m_ids.push_back(id);
	}
	void clear_data()
	{
		m_depths.clear();
		m_colors.clear();
		m_ids.clear();
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
				int id = 0;
				const std::vector<dfusion::depthtype>* depth = nullptr;
				const QImage* color = nullptr;
				{
					QMutexLocker locker(g_mutex);
					id = m_ids.front();
					depth = &m_depths.front();
					if (m_colors.size())
						color = &m_colors.front();
				}
				QDir dir(m_currentPath);
				QString name = dir.absoluteFilePath(QString().sprintf("%08d.depth", id));
				scanning_data_holder.saveDepth(*depth, name.toStdString());
				
				name = dir.absoluteFilePath(QString().sprintf("%08d.png", id));
				color->save(name);
				{
					QMutexLocker locker(g_mutex);
					m_ids.pop_front();
					m_depths.pop_front();
					
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
	QString m_currentPath;
	bool flag_run;
};
SaveThread_Recon g_saveThread;

Scanning::Scanning(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	m_frameIndex = 0;
	m_currentPath = "../data/frame/Scanning";
	connectEvent();
	scanning_data_holder.init();
	m_fpsTimerId = startTimer(30);
	g_mutex = new QMutex();
	g_mutex_clear = new QMutex();
	g_saveThread.set_path(m_currentPath);
	g_saveThread.start();
}

Scanning::~Scanning()
{
	g_saveThread.terminate();
	delete g_mutex;
	delete g_mutex_clear;
}

void Scanning::timerEvent(QTimerEvent * ev)
{
	static gtime_t time_now;
	static gtime_t time_last = ldp::gtime_now();
	if (m_fpsTimerId == ev->timerId())
	{
		scanning_data_holder.getFrame();
		scanning_data_holder.getDepthShowImage();
		scanning_data_holder.getNormalShowImage();
		ui.label_color->setPixmap(QPixmap::fromImage(scanning_data_holder.m_color_qimage));
		ui.label_normal->setPixmap(QPixmap::fromImage(scanning_data_holder.m_normal_show_image));
		if (flag_saving)
			saving();
		time_now = ldp::gtime_now();
		double sec = ldp::gtime_seconds(time_last, time_now);
		double fps = 1.0 / sec;
		setWindowTitle(QString().sprintf("[%d] FPS:%.1f", m_frameIndex, fps));
		time_last = time_now;
	}
}

void Scanning::saving()
{
	g_saveThread.push_depth(
		scanning_data_holder.m_depth,
		scanning_data_holder.m_color_qimage, 
		m_frameIndex++);
}
void Scanning::connectEvent()
{
	connect(ui.pb_recording, SIGNAL(clicked()), this, SLOT(clicked_pb_recording()));
	connect(ui.pb_finish, SIGNAL(clicked()), this, SLOT(clicked_pb_finish()));
}
void Scanning::reset()
{
	{
		QMutexLocker locker(g_mutex_clear);
		g_saveThread.clear_data();
		std::cout << "clear save list" << std::endl;
		removeFolderContent(m_currentPath);
		std::cout << "remove files" << std::endl;
	}
	m_frameIndex = 0;
}
void Scanning::clicked_pb_finish()
{
	flag_saving = false;
}
void Scanning::clicked_pb_recording() 
{
	reset();
	flag_saving = true;
}