#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <QtWidgets/QMainWindow>
#include "ui_reconstruction.h"
#include "meshlabScript.h"
#include "FrameInfo.h"
class Reconstruction : public QMainWindow
{
	Q_OBJECT
		enum State
	{
		Pause,
		Loading,
		Saving,
		Live,
		LiveSaving,
		//ShowLoadedStaticVolume,
	};
public:
	Reconstruction(QWidget *parent = 0);
	~Reconstruction();

	void timerEvent(QTimerEvent* ev);
	void keyPressEvent(QKeyEvent  *event);
public slots:
	void resetVolume();

	void clicked_action_rb_live();
	void clicked_action_rb_live_saving();
	void clicked_action_rb_loading();

	void clicked_action_pb_save_mesh();
	void clicked_action_pb_play_pause();
	void clicked_action_pb_load_color_list();

protected:
	void frameLoading();
	void frameSaving();
	void frameLive();

	void updateKinectFusion(bool &suc);

	void setState(State s) { m_lastState = m_state;  m_state = s; }
	void restoreState() { m_state = m_lastState; }
	void initUIParam();
	void updateUIParam();
	void readColorList(QString filename);

	void saveVNMAPasObj();

	void saveCanoMesh(QString name, bool postProcess = false);

	void connectEvent();

	bool frameByFrame;
private:
	FrameInfoRecon frame_info;

	Ui::ReconstructionClass ui;
	State m_state;
	State m_lastState;
	QString m_currentPath;
	int m_fpsTimerId;
	int m_autoResetTimerId;
	int m_frameIndex;
};

#endif // RECONSTRUCTION_H
