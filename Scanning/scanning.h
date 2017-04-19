#ifndef SCANNING_H
#define SCANNING_H

#include <QtWidgets/QMainWindow>
#include "ui_scanning.h"

class Scanning : public QMainWindow
{
	Q_OBJECT

public:
	Scanning(QWidget *parent = 0);
	~Scanning();

	void timerEvent(QTimerEvent* ev);	
	void reset();
	
public slots:
	void clicked_pb_recording();
	void clicked_pb_finish();
protected:
	void saving();
	void connectEvent();
private:
	Ui::ScanningClass ui;

	bool flag_saving = false;

	QString m_currentPath;
	int m_fpsTimerId;
	int m_frameIndex;
};

#endif // SCANNING_H
